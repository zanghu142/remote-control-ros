#include "stereo_fisheye_renderer.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <SDL3/SDL_opengl_glext.h>
#include <deque>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <map>
#include <tuple>

namespace rtc_console {
namespace {
constexpr const char* kVertexShader = R"glsl(#version 130
uniform mat4 ProjMtx;
in vec2 Position;
in vec2 UV;
in vec4 Color;
out vec2 Frag_UV;
out vec4 Frag_Color;
void main() {
  Frag_UV = UV;
  Frag_Color = Color;
  gl_Position = ProjMtx * vec4(Position.xy, 0, 1);
}
)glsl";

constexpr const char* kFragmentShader = R"glsl(#version 130
uniform sampler2D Texture;
uniform vec2 EyeSize;
uniform vec4 Crop;
uniform vec4 ValidCrop;
uniform float DrawWidth;
uniform float DrawHeight;
uniform int Interleave;
uniform int CorrectFisheye;
uniform int SingleCamera;
uniform vec4 SourceRegion;
uniform mat3 RayLeft;
uniform mat3 RayRight;
uniform vec4 KLeft;
uniform vec4 KRight;
uniform vec4 DLeft;
uniform vec4 DRight;
in vec2 Frag_UV;
in vec4 Frag_Color;
out vec4 Out_Color;
void main() {
  bool rightEye;
  vec2 uv;
  if (SingleCamera != 0) {
    rightEye = false;
    uv = (Frag_UV - SourceRegion.xy) / (SourceRegion.zw - SourceRegion.xy);
  } else if (Interleave != 0) {
    float column = floor((Frag_UV.x - Crop.x) / (Crop.z - Crop.x) * DrawWidth);
    rightEye = mod(column, 2.0) >= 1.0;
    float eyeColumns = rightEye ? floor(DrawWidth * 0.5) : ceil(DrawWidth * 0.5);
    float t = eyeColumns > 1.0 ? floor(column * 0.5) / (eyeColumns - 1.0) : 0.0;
    // Both eyes in a pair use the same rectified output column, matching the
    // original interleaver; alternating eyes must not introduce an x offset.
    float rectifiedX = mix(Crop.x * EyeSize.x, Crop.z * EyeSize.x - 1.0, t);
    uv = vec2((rectifiedX + 0.5) / EyeSize.x, Frag_UV.y);
  } else {
    rightEye = Frag_UV.x >= 0.5;
    uv = vec2(Frag_UV.x * 2.0 - (rightEye ? 1.0 : 0.0), Frag_UV.y);
  }
  if (CorrectFisheye == 0) {
    // Match the original CPU interleaver's nearest-pixel, endpoint-aligned
    // sampling without creating or uploading an interleaved intermediate image.
    float row = clamp(floor((Frag_UV.y - Crop.y) / (Crop.w - Crop.y) * DrawHeight),
                      0.0, DrawHeight - 1.0);
    float t = DrawHeight > 1.0 ? row / (DrawHeight - 1.0) : 0.0;
    float sourceY = mix(Crop.y * EyeSize.y, Crop.w * EyeSize.y - 1.0, t);
    vec2 pixel = floor(vec2(uv.x * EyeSize.x - 0.5, sourceY) + vec2(0.5));
    pixel = clamp(pixel, vec2(0), EyeSize - vec2(1));
    vec2 source = (pixel + vec2(0.5)) / EyeSize;
    source.x = (source.x + (rightEye ? 1.0 : 0.0)) * 0.5;
    Out_Color = Frag_Color * texture(Texture, source);
    return;
  }
  // Apply one common crop after selecting the eye and the interleaved column.
  // Both eyes use identical zoom and offset, retaining rectified epipolar lines.
  uv = mix(ValidCrop.xy, ValidCrop.zw, uv);
  mat3 h = rightEye ? RayRight : RayLeft;
  vec4 k = rightEye ? KRight : KLeft;
  vec4 d = rightEye ? DRight : DLeft;
  vec3 ray = h * vec3(uv * EyeSize - vec2(0.5), 1.0);
  if (ray.z <= 0.0) { Out_Color = vec4(0, 0, 0, Frag_Color.a); return; }
  vec2 plane = ray.xy / ray.z;
  float r = length(plane);
  float theta = atan(r);
  float t2 = theta * theta;
  float scale = r < 0.000001 ? 1.0 : theta *
      (1.0 + t2 * (d.x + t2 * (d.y + t2 * (d.z + t2 * d.w)))) / r;
  vec2 pixel = k.xy * plane * scale + k.zw;
  if (any(lessThan(pixel, vec2(-0.5))) || any(greaterThanEqual(pixel, EyeSize - vec2(0.5)))) {
    Out_Color = vec4(0, 0, 0, Frag_Color.a); return;
  }
  // Clamp INSIDE the selected eye, so bilinear filtering never crosses the seam.
  vec2 source = clamp((pixel + vec2(0.5)) / EyeSize,
                     vec2(0.5) / EyeSize, vec2(1.0) - vec2(0.5) / EyeSize);
  if (SingleCamera != 0) {
    // Sample only this camera's quadrant, even when the texture is a 2x2 grid.
    source = mix(SourceRegion.xy, SourceRegion.zw, source);
  } else {
    source.x = (source.x + (rightEye ? 1.0 : 0.0)) * 0.5;
  }
  Out_Color = Frag_Color * texture(Texture, source);
}
)glsl";
}

#define FISHEYE_GL_FUNCTIONS(X) \
  X(CreateShader, PFNGLCREATESHADERPROC) \
  X(ShaderSource, PFNGLSHADERSOURCEPROC) \
  X(CompileShader, PFNGLCOMPILESHADERPROC) \
  X(GetShaderiv, PFNGLGETSHADERIVPROC) \
  X(GetShaderInfoLog, PFNGLGETSHADERINFOLOGPROC) \
  X(DeleteShader, PFNGLDELETESHADERPROC) \
  X(CreateProgram, PFNGLCREATEPROGRAMPROC) \
  X(AttachShader, PFNGLATTACHSHADERPROC) \
  X(BindAttribLocation, PFNGLBINDATTRIBLOCATIONPROC) \
  X(LinkProgram, PFNGLLINKPROGRAMPROC) \
  X(GetProgramiv, PFNGLGETPROGRAMIVPROC) \
  X(GetProgramInfoLog, PFNGLGETPROGRAMINFOLOGPROC) \
  X(DeleteProgram, PFNGLDELETEPROGRAMPROC) \
  X(GetAttribLocation, PFNGLGETATTRIBLOCATIONPROC) \
  X(GetUniformLocation, PFNGLGETUNIFORMLOCATIONPROC) \
  X(UseProgram, PFNGLUSEPROGRAMPROC) \
  X(UniformMatrix4fv, PFNGLUNIFORMMATRIX4FVPROC) \
  X(UniformMatrix3fv, PFNGLUNIFORMMATRIX3FVPROC) \
  X(Uniform1i, PFNGLUNIFORM1IPROC) \
  X(Uniform1f, PFNGLUNIFORM1FPROC) \
  X(Uniform2f, PFNGLUNIFORM2FPROC) \
  X(Uniform4f, PFNGLUNIFORM4FPROC) \
  X(Uniform4fv, PFNGLUNIFORM4FVPROC)

struct StereoFisheyeRenderer::Impl {
#define DECLARE(name, type) type name = nullptr;
  FISHEYE_GL_FUNCTIONS(DECLARE)
#undef DECLARE
  GLuint program = 0;
  std::string error;
  enum Uniform {
    Projection, Texture, Interleave, CorrectFisheye, SingleCamera, SourceRegion,
    DrawWidth, DrawHeight, EyeSize, Crop, ValidCrop, RayLeft, RayRight,
    KLeft, KRight, DLeft, DRight, UniformCount
  };
  std::array<GLint, UniformCount> locations{};
  struct Mapping {
    std::array<FisheyeUniforms, 2> eyes;
    std::array<float, 4> crop;
  };
  // Calibration objects are immutable and live for the application's lifetime.
  std::map<std::tuple<const StereoFisheyeCalibration*, int, int>, Mapping> mappings;
  struct Draw {
    Impl* owner;
    const Mapping* mapping = nullptr;
    int width, height;
    float draw_width, draw_height;
    ImVec2 uv_min, uv_max;
    std::array<float, 16> projection;
    std::array<float, 4> source_region;
    bool interleave;
    bool single_camera;
  };
  // Stable addresses until ImGui has executed ALL callbacks for this frame.
  std::deque<Draw> draws;

  GLuint Compile(GLenum type, const char* source) {
    GLuint shader = CreateShader(type);
    ShaderSource(shader, 1, &source, nullptr);
    CompileShader(shader);
    GLint okay = 0;
    GetShaderiv(shader, GL_COMPILE_STATUS, &okay);
    if (!okay) {
      char log[2048]{};
      GetShaderInfoLog(shader, sizeof(log), nullptr, log);
      DeleteShader(shader);
      throw std::runtime_error(std::string("Fisheye shader compilation failed: ") + log);
    }
    return shader;
  }

  void Initialize(GLuint imgui_program) {
#define LOAD(name, type) \
    name = reinterpret_cast<type>(SDL_GL_GetProcAddress("gl" #name)); \
    if (!name) throw std::runtime_error("Missing OpenGL entry point: gl" #name);
    FISHEYE_GL_FUNCTIONS(LOAD)
#undef LOAD
    GLuint vertex = 0, fragment = 0;
    try {
      vertex = Compile(GL_VERTEX_SHADER, kVertexShader);
      fragment = Compile(GL_FRAGMENT_SHADER, kFragmentShader);
      program = CreateProgram();
      AttachShader(program, vertex);
      AttachShader(program, fragment);
      // Reuse the exact vertex layout of ImGui's currently bound VAO.
      for (const char* attribute : {"Position", "UV", "Color"}) {
        GLint location = GetAttribLocation(imgui_program, attribute);
        if (location < 0) throw std::runtime_error("Unsupported ImGui OpenGL vertex layout");
        BindAttribLocation(program, static_cast<GLuint>(location), attribute);
      }
      LinkProgram(program);
      GLint okay = 0;
      GetProgramiv(program, GL_LINK_STATUS, &okay);
      if (!okay) {
        char log[2048]{};
        GetProgramInfoLog(program, sizeof(log), nullptr, log);
        throw std::runtime_error(std::string("Fisheye shader link failed: ") + log);
      }
      const char* names[] = {"ProjMtx", "Texture", "Interleave", "CorrectFisheye",
          "SingleCamera", "SourceRegion", "DrawWidth", "DrawHeight", "EyeSize",
          "Crop", "ValidCrop", "RayLeft", "RayRight", "KLeft", "KRight", "DLeft", "DRight"};
      for (size_t i = 0; i < locations.size(); ++i) {
        locations[i] = GetUniformLocation(program, names[i]);
        if (locations[i] < 0) throw std::runtime_error(std::string("Missing shader uniform: ") + names[i]);
      }
    } catch (...) {
      if (vertex) DeleteShader(vertex);
      if (fragment) DeleteShader(fragment);
      if (program) DeleteProgram(program);
      program = 0;
      throw;
    }
    DeleteShader(vertex);
    DeleteShader(fragment);
  }

  static void Callback(const ImDrawList*, const ImDrawCmd* command) {
    auto& draw = *static_cast<Draw*>(command->UserCallbackData);
    auto& gl = *draw.owner;
    if (!gl.error.empty()) return;
    GLint previous = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &previous);
    try {
      if (!gl.program) gl.Initialize(static_cast<GLuint>(previous));
      gl.UseProgram(gl.program);
      const auto& location = gl.locations;
      gl.UniformMatrix4fv(location[Projection], 1, GL_FALSE, draw.projection.data());
      gl.Uniform1i(location[Texture], 0);
      gl.Uniform1i(location[Interleave], draw.interleave ? 1 : 0);
      gl.Uniform1i(location[CorrectFisheye], draw.mapping ? 1 : 0);
      gl.Uniform1i(location[SingleCamera], draw.single_camera ? 1 : 0);
      gl.Uniform4fv(location[SourceRegion], 1, draw.source_region.data());
      gl.Uniform1f(location[DrawWidth], draw.draw_width);
      gl.Uniform1f(location[DrawHeight], draw.draw_height);
      gl.Uniform2f(location[EyeSize], static_cast<float>(draw.width),
                   static_cast<float>(draw.height));
      gl.Uniform4f(location[Crop], draw.uv_min.x, draw.uv_min.y,
                   draw.uv_max.x, draw.uv_max.y);
      if (draw.mapping) {
        gl.Uniform4fv(location[ValidCrop], 1, draw.mapping->crop.data());
        for (int i = 0; i < 2; ++i) {
          gl.UniformMatrix3fv(location[i == 0 ? RayLeft : RayRight], 1, GL_FALSE,
                              draw.mapping->eyes[i].rectified_to_ray.data());
          gl.Uniform4fv(location[i == 0 ? KLeft : KRight], 1, draw.mapping->eyes[i].intrinsics.data());
          gl.Uniform4fv(location[i == 0 ? DLeft : DRight], 1, draw.mapping->eyes[i].distortion.data());
        }
      }
    } catch (const std::exception& exception) {
      gl.error = exception.what();
      if (gl.UseProgram) gl.UseProgram(previous);
    }
  }
};
#undef FISHEYE_GL_FUNCTIONS

StereoFisheyeRenderer::StereoFisheyeRenderer() : impl_(std::make_unique<Impl>()) {}
StereoFisheyeRenderer::~StereoFisheyeRenderer() = default;
void StereoFisheyeRenderer::BeginFrame() {
  impl_->draws.clear();
  // Drop old resolution variants only BETWEEN frames, after their callbacks ran.
  if (impl_->mappings.size() > 128) impl_->mappings.clear();
}
const std::string& StereoFisheyeRenderer::Error() const { return impl_->error; }
void StereoFisheyeRenderer::Shutdown() {
  if (impl_->program) impl_->DeleteProgram(impl_->program);
  impl_->program = 0;
  impl_->draws.clear();
  impl_->mappings.clear();
}

void StereoFisheyeRenderer::DrawImage(ImDrawList* list, ImTextureID texture,
    ImVec2 min, ImVec2 max, ImVec2 uv_min, ImVec2 uv_max,
    const StereoFisheyeCalibration* calibration, int width, int height, bool interleave,
    bool single_camera, std::array<float, 4> source_region) {
  const auto* viewport = ImGui::GetMainViewport();
  if (width <= 0 || height <= 0 || viewport->Size.x <= 0 || viewport->Size.y <= 0) return;
  auto& draw = impl_->draws.emplace_back();
  draw.owner = impl_.get();
  draw.width = width;
  draw.height = height;
  if (calibration) {
    const auto key = std::make_tuple(calibration, width, height);
    auto cached = impl_->mappings.find(key);
    if (cached == impl_->mappings.end()) {
      // At most a few receive sizes per camera are useful; never evict a map
      // during drawing because earlier callbacks in this frame reference it.
      Impl::Mapping mapping;
      for (int i = 0; i < 2; ++i) mapping.eyes[i] = MakeFisheyeUniforms(*calibration, i, width, height);
      mapping.crop = calibration->valid_crop;
      cached = impl_->mappings.emplace(key, mapping).first;
    }
    draw.mapping = &cached->second;
  }
  draw.uv_min = uv_min;
  draw.uv_max = uv_max;
  // Same top-left projection as the ImGui OpenGL backend, computed on the CPU
  // instead of querying the current program's uniform on every video draw.
  const float left = viewport->Pos.x, right = left + viewport->Size.x;
  const float top = viewport->Pos.y, bottom = top + viewport->Size.y;
  draw.projection = {{2 / (right - left), 0, 0, 0,
                      0, 2 / (top - bottom), 0, 0,
                      0, 0, -1, 0,
                      (right + left) / (left - right), (top + bottom) / (bottom - top), 0, 1}};
  draw.single_camera = single_camera;
  draw.source_region = source_region;
  draw.interleave = interleave;
  draw.draw_width = std::max(1.0f, std::round((max.x - min.x) *
      ImGui::GetIO().DisplayFramebufferScale.x));
  draw.draw_height = std::max(1.0f, std::round((max.y - min.y) *
      ImGui::GetIO().DisplayFramebufferScale.y));
  list->AddCallback(&Impl::Callback, &draw);
  list->AddImage(texture, min, max, uv_min, uv_max);
  list->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
}
}  // namespace rtc_console
