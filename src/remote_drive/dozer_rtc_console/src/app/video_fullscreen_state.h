#pragma once

#include <cmath>
#include <string>

namespace rtc_console {

struct FullscreenVideoCrop {
  float left = 0;
  float top = 0;
  float right = 1;
  float bottom = 1;
};

class VideoFullscreenState {
 public:
  enum class Mode { Windowed, Monitor, Stream };

  Mode mode() const { return mode_; }
  bool active() const { return mode_ != Mode::Windowed; }
  bool returns_to_monitor() const { return return_to_monitor_; }
  bool returns_to_stream() const { return return_to_stream_; }
  const std::string& stream_key() const { return stream_key_; }
  const std::string& label() const { return label_; }
  const FullscreenVideoCrop& crop() const { return crop_; }

  void OpenMonitor() {
    Reset();
    mode_ = Mode::Monitor;
  }

  bool OpenStream(const std::string& key, const FullscreenVideoCrop& crop,
                  const std::string& label) {
    if (key.empty() || !std::isfinite(crop.left) || !std::isfinite(crop.top) ||
        !std::isfinite(crop.right) || !std::isfinite(crop.bottom) ||
        crop.left < 0 || crop.top < 0 || crop.right > 1 || crop.bottom > 1 ||
        crop.left >= crop.right || crop.top >= crop.bottom) {
      return false;
    }
    const bool nested_crop = mode_ == Mode::Stream && stream_key_ == key &&
        crop.left >= crop_.left && crop.top >= crop_.top &&
        crop.right <= crop_.right && crop.bottom <= crop_.bottom &&
        (crop.left > crop_.left || crop.top > crop_.top ||
         crop.right < crop_.right || crop.bottom < crop_.bottom);
    if (nested_crop) {
      parent_crop_ = crop_;
      parent_label_ = label_;
    }
    return_to_stream_ = nested_crop;
    return_to_monitor_ = mode_ == Mode::Monitor ||
        (mode_ == Mode::Stream && return_to_monitor_);
    mode_ = Mode::Stream;
    stream_key_ = key;
    crop_ = crop;
    label_ = label;
    return true;
  }

  void Back() {
    if (mode_ == Mode::Stream && return_to_stream_) {
      crop_ = parent_crop_;
      label_ = parent_label_;
      return_to_stream_ = false;
      parent_label_.clear();
    } else if (mode_ == Mode::Stream && return_to_monitor_) {
      OpenMonitor();
    } else {
      Reset();
    }
  }

  void Reset() {
    mode_ = Mode::Windowed;
    return_to_monitor_ = false;
    return_to_stream_ = false;
    parent_label_.clear();
    stream_key_.clear();
    label_.clear();
    crop_ = FullscreenVideoCrop{};
  }

 private:
  Mode mode_ = Mode::Windowed;
  bool return_to_monitor_ = false;
  bool return_to_stream_ = false;
  FullscreenVideoCrop parent_crop_;
  std::string parent_label_;
  std::string stream_key_;
  std::string label_;
  FullscreenVideoCrop crop_;
};

}
