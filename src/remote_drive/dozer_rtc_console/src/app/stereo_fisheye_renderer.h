#pragma once

#include "stereo_fisheye.h"
#include <imgui.h>
#include <memory>
#include <string>

namespace rtc_console {
// Uses the current ImGui OpenGL context. Destroy GPU objects before that context.
class StereoFisheyeRenderer {
 public:
  StereoFisheyeRenderer();
  ~StereoFisheyeRenderer();
  void BeginFrame();
  void Shutdown();
  const std::string& Error() const;
  // A null calibration selects GPU raw stereo interleaving. Input textures
  // always contain original received pixels and are never rearranged per view.
  void DrawImage(ImDrawList* list, ImTextureID texture, ImVec2 min, ImVec2 max,
                 ImVec2 uv_min, ImVec2 uv_max, const StereoFisheyeCalibration* calibration,
                 int width, int height, bool interleave, bool single_camera = false,
                 std::array<float, 4> source_region = {{0, 0, 1, 1}});
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace rtc_console
