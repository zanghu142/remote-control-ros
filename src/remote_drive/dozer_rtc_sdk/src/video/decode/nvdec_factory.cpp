#include "nvdec_factory.h"

#include <absl/strings/match.h>
#include <api/video_codecs/video_decoder.h>
#include <common_video/include/i420_buffer_pool.h>
#include <modules/video_coding/codecs/h264/include/h264.h>
#include <NvDecoder/NvDecoder.h>
#include <libyuv/convert.h>
#include <windows.h>
#include <delayimp.h>

#include <limits>

#include "log/log_manager.h"

namespace webrtc {
namespace {

bool InitializeDriver(CUdevice* device, CUcontext* context) {
  __try {
    // Newer CUDA libraries load nvcuda.dll themselves, without a delay-import
    // entry. Call the driver API to initialize it; older delay imports remain
    // protected by the exception handler below.
    return SUCCEEDED(__HrLoadAllImportsForDll("nvcuvid.dll")) &&
           cuInit(0) == CUDA_SUCCESS &&
           cuDeviceGet(device, 0) == CUDA_SUCCESS &&
           cuDevicePrimaryCtxRetain(context, *device) == CUDA_SUCCESS;
  } __except ((GetExceptionCode() == VcppException(ERROR_SEVERITY_ERROR, ERROR_MOD_NOT_FOUND) ||
               GetExceptionCode() == VcppException(ERROR_SEVERITY_ERROR, ERROR_PROC_NOT_FOUND))
                  ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
    return false;
  }
}

class NvdecH264 final : public VideoDecoder {
 public:
  ~NvdecH264() override { Release(); }

  int32_t InitDecode(const VideoCodec* codec, int32_t) override {
    Release();
    if (!codec || codec->codecType != kVideoCodecH264) return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
    if (!InitializeDriver(&device_, &context_)) {
      LOG_ERROR("NVDEC: cannot initialize NVIDIA display driver / CUDA device 0");
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    try {
      decoder_ = MakeDecoder();
      if (codec->buffer_pool_size && !pool_.Resize(*codec->buffer_pool_size)) {
        Release();
        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
      }
      width_ = codec->width;
      height_ = codec->height;
    } catch (const std::exception& error) {
      LOG_ERROR("NVDEC initialization failed: %s", error.what());
      Release();
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    return WEBRTC_VIDEO_CODEC_OK;
  }

  int32_t Release() override {
    decoder_.reset();
    if (context_) {
      cuDevicePrimaryCtxRelease(device_);
      context_ = nullptr;
    }
    pool_.Release();
    return WEBRTC_VIDEO_CODEC_OK;
  }

  int32_t RegisterDecodeCompleteCallback(DecodedImageCallback* callback) override {
    callback_ = callback;
    return WEBRTC_VIDEO_CODEC_OK;
  }

  int32_t Decode(const EncodedImage& image, bool, int64_t) override {
    if (!decoder_ || !callback_) return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
    if (!image.data() || !image.size() || image.size() > std::numeric_limits<int>::max())
      return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
    try {
      if (image._encodedWidth && image._encodedHeight &&
          (width_ != image._encodedWidth || height_ != image._encodedHeight)) {
        decoder_ = MakeDecoder();
        width_ = image._encodedWidth;
        height_ = image._encodedHeight;
      }
      const int count = decoder_->Decode(image.data(), static_cast<int>(image.size()),
                                         CUVID_PKT_ENDOFPICTURE, image.Timestamp());
      for (int index = 0; index < count; ++index) {
        int64_t timestamp = 0;
        const auto* pixels = decoder_->GetFrame(&timestamp);
        const int width = decoder_->GetDecodeWidth();
        const int height = decoder_->GetHeight();
        const int pitch = decoder_->GetWidth();
        auto buffer = pool_.CreateBuffer(width, height);
        if (!pixels || !buffer) return WEBRTC_VIDEO_CODEC_ERROR;
        const auto* chroma = pixels + decoder_->GetLumaPlaneSize();
        int result = -1;
        if (decoder_->GetOutputFormat() == cudaVideoSurfaceFormat_NV12) {
          result = libyuv::NV12ToI420(pixels, pitch, chroma, pitch,
              buffer->MutableDataY(), buffer->StrideY(), buffer->MutableDataU(), buffer->StrideU(),
              buffer->MutableDataV(), buffer->StrideV(), width, height);
        } else if (decoder_->GetOutputFormat() == cudaVideoSurfaceFormat_YUV444) {
          result = libyuv::I444ToI420(pixels, pitch, chroma, pitch,
              chroma + decoder_->GetLumaPlaneSize(), pitch, buffer->MutableDataY(), buffer->StrideY(),
              buffer->MutableDataU(), buffer->StrideU(), buffer->MutableDataV(), buffer->StrideV(), width, height);
        }
        if (result != 0) return WEBRTC_VIDEO_CODEC_ERROR;
        auto frame = VideoFrame::Builder().set_video_frame_buffer(buffer)
                         .set_timestamp_rtp(static_cast<uint32_t>(timestamp)).build();
        callback_->Decoded(frame, absl::nullopt, absl::nullopt);
      }
    } catch (const std::exception& error) {
      LOG_ERROR("NVDEC H.264 decode failed: %s", error.what());
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    return WEBRTC_VIDEO_CODEC_OK;
  }

  bool PrefersLateDecoding() const override { return true; }
  const char* ImplementationName() const override { return "NvDecH264"; }

 private:
  std::unique_ptr<NvDecoder> MakeDecoder() const {
    return std::make_unique<NvDecoder>(context_, false, cudaVideoCodec_H264, true,
        false, nullptr, nullptr, 0, 0, 90000);
  }
  CUdevice device_{};
  CUcontext context_{};
  std::unique_ptr<NvDecoder> decoder_;
  I420BufferPool pool_{true};
  DecodedImageCallback* callback_{};
  uint32_t width_{}, height_{};
};

class NvdecFactory final : public VideoDecoderFactory {
 public:
  std::vector<SdpVideoFormat> GetSupportedFormats() const override {
    std::vector<SdpVideoFormat> formats;
    for (auto profile : {H264::kProfileBaseline, H264::kProfileConstrainedBaseline, H264::kProfileHigh}) {
      for (const char* mode : {"1", "0"})
        formats.push_back(CreateH264Format(profile, H264::kLevel5_1, mode));
    }
    return formats;
  }
  std::unique_ptr<VideoDecoder> CreateVideoDecoder(const SdpVideoFormat& format) override {
    if (!absl::EqualsIgnoreCase(format.name, cricket::kH264CodecName)) return nullptr;
    return std::make_unique<NvdecH264>();
  }
};
}

std::unique_ptr<VideoDecoderFactory> CreateNvdecH264Factory() {
  return std::make_unique<NvdecFactory>();
}
}
