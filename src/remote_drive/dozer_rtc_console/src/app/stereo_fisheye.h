#pragma once

#include <array>
#include <string>

namespace rtc_console {

struct FisheyeEye {
  std::array<double, 9> camera{};
  std::array<double, 4> distortion{};
  std::array<double, 9> rotation{};
  std::array<double, 12> projection{};
  double max_theta = 1.5706963267948966;
};

struct StereoFisheyeCalibration {
  int width = 0;  // Calibration resolution of ONE eye.
  int height = 0;
  std::array<FisheyeEye, 2> eyes;
  // Normalized common rectangle in the rectified image, shared by both eyes.
  // Centered and uniform so cropping preserves aspect and stereo alignment.
  std::array<float, 4> valid_crop{{0, 0, 1, 1}};
};

struct FisheyeUniforms {
  std::array<float, 9> rectified_to_ray{};  // Column-major, for GLSL.
  std::array<float, 4> intrinsics{};        // fx, fy, cx, cy at received resolution.
  std::array<float, 4> distortion{};
  float max_theta = 1.5706963f;
};

StereoFisheyeCalibration LoadStereoFisheyeCalibration(const std::string& path);
// Single-camera YAML uses K/D/new_camera_matrix with an identity rotation.
// Both eye slots contain the same camera to share the mapping/cropping code.
StereoFisheyeCalibration LoadSingleFisheyeCalibration(const std::string& path);
bool FisheyeResolutionMatches(const StereoFisheyeCalibration& calibration, int width, int height);
FisheyeUniforms MakeFisheyeUniforms(const StereoFisheyeCalibration& calibration,
                                  int eye, int width, int height);
bool FisheyeRectifiedPoint(const FisheyeEye& eye, double source_x, double source_y,
                          double* rectified_x, double* rectified_y);

}  // namespace rtc_console
