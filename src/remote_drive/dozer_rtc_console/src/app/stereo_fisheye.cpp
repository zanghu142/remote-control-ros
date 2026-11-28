#include "stereo_fisheye.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>

namespace rtc_console {
namespace {
using YamlFields = std::map<std::string, std::string>;

std::string Trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

// Read the scalar fields and !!opencv-matrix blocks written by OpenCV
// FileStorage. This deliberately supports calibration YAML, not general YAML.
YamlFields ReadYamlFields(std::istream& input, bool matrix_fields = false) {
  YamlFields fields;
  std::string line, key;
  bool first_line = true;
  while (std::getline(input, line)) {
    if (first_line && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
    first_line = false;
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed[0] == '%' || trimmed == "---" || trimmed == "...") continue;
    const auto colon = trimmed.find(':');
    const bool starts_field = colon != std::string::npos &&
        (matrix_fields || !std::isspace(static_cast<unsigned char>(line[0])));
    if (starts_field) {
      key = Trim(trimmed.substr(0, colon));
      if (key.empty() || !std::all_of(key.begin(), key.end(), [](unsigned char ch) {
            return std::isalnum(ch) || ch == '_';
          }) || !fields.emplace(key, Trim(trimmed.substr(colon + 1))).second)
        throw std::runtime_error("Invalid or duplicate fisheye YAML field: " + key);
    } else {
      if (key.empty()) throw std::runtime_error("Invalid fisheye YAML content");
      fields.at(key) += '\n' + trimmed;
    }
  }
  return fields;
}

std::string ReadScalar(const YamlFields& fields, const std::string& name) {
  const auto found = fields.find(name);
  if (found == fields.end()) throw std::runtime_error("Missing fisheye YAML field: " + name);
  std::string value = Trim(found->second);
  if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
                           (value.front() == '\'' && value.back() == '\'')))
    value = value.substr(1, value.size() - 2);
  return value;
}

int ReadPositiveInteger(const YamlFields& fields, const std::string& name) {
  std::istringstream input(ReadScalar(fields, name));
  input.imbue(std::locale::classic());
  int value = 0;
  if (!(input >> value) || value <= 0 || value > 32768)
    throw std::runtime_error("Invalid fisheye YAML integer: " + name);
  input >> std::ws;
  if (!input.eof()) throw std::runtime_error("Invalid fisheye YAML integer: " + name);
  return value;
}

template <size_t Rows, size_t Cols>
std::array<double, Rows * Cols> ReadMatrix(const YamlFields& root, const std::string& name) {
  std::istringstream block(ReadScalar(root, name));
  std::string tag;
  std::getline(block, tag);
  if (Trim(tag) != "!!opencv-matrix")
    throw std::runtime_error("Expected OpenCV YAML matrix: " + name);
  const auto fields = ReadYamlFields(block, true);
  if (ReadPositiveInteger(fields, "rows") != Rows ||
      ReadPositiveInteger(fields, "cols") != Cols)
    throw std::runtime_error("Invalid fisheye matrix dimensions: " + name);
  const auto type = ReadScalar(fields, "dt");
  if (type != "d" && type != "f")
    throw std::runtime_error("Expected floating-point fisheye matrix: " + name);
  auto data = ReadScalar(fields, "data");
  if (data.size() < 2 || data.front() != '[' || data.back() != ']')
    throw std::runtime_error("Invalid fisheye matrix data: " + name);
  data = data.substr(1, data.size() - 2);
  std::replace(data.begin(), data.end(), ',', ' ');
  std::istringstream values(data);
  values.imbue(std::locale::classic());
  std::array<double, Rows * Cols> result{};
  for (auto& value : result) {
    if (!(values >> value) || !std::isfinite(value))
      throw std::runtime_error("Invalid fisheye matrix number: " + name);
  }
  values >> std::ws;
  if (!values.eof()) throw std::runtime_error("Too many fisheye matrix values: " + name);
  return result;
}

void ValidateEye(const FisheyeEye& eye) {
  if (eye.camera[0] <= 0 || eye.camera[4] <= 0 ||
      eye.projection[0] <= 0 || eye.projection[5] <= 0 ||
      std::abs(eye.camera[1]) > 1e-8 || std::abs(eye.camera[3]) > 1e-8 ||
      std::abs(eye.camera[6]) > 1e-8 || std::abs(eye.camera[7]) > 1e-8 ||
      std::abs(eye.camera[8] - 1) > 1e-8)
    throw std::runtime_error("Invalid fisheye camera intrinsics");
  if (std::abs(eye.projection[1]) > 1e-8 || std::abs(eye.projection[4]) > 1e-8 ||
      std::abs(eye.projection[8]) > 1e-8 || std::abs(eye.projection[9]) > 1e-8 ||
      std::abs(eye.projection[10] - 1) > 1e-8 || std::abs(eye.projection[11]) > 1e-8)
    throw std::runtime_error("Unsupported fisheye projection matrix");
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double dot = 0;
      for (int k = 0; k < 3; ++k) dot += eye.rotation[row * 3 + k] * eye.rotation[col * 3 + k];
      if (std::abs(dot - (row == col ? 1.0 : 0.0)) > 1e-5)
        throw std::runtime_error("Fisheye rectification rotation must be orthonormal");
    }
  }
  const auto& r = eye.rotation;
  const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) -
      r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
  if (std::abs(determinant - 1) > 1e-5)
    throw std::runtime_error("Fisheye rectification rotation must preserve orientation");
}

template <typename T>
double DistortedTheta(const std::array<T, 4>& d, double theta) {
  const double t2 = theta * theta;
  return theta * (1 + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3]))));
}

template <typename T>
double DistortedThetaDerivative(const std::array<T, 4>& d, double theta) {
  const double t2 = theta * theta;
  return 1 + t2 * (3*d[0] + t2 * (5*d[1] + t2 * (7*d[2] + t2 * 9*d[3])));
}

template <typename T>
double MonotonicThetaLimit(const std::array<T, 4>& d) {
  // Work only on the first invertible radial branch. A non-monotonic model can
  // fold distant pixels over the center even though all projected values fit.
  constexpr double maximum = 1.5706963267948966;
  constexpr int samples = 4096;
  for (int i = 1; i <= samples; ++i) {
    const double theta = maximum * i / samples;
    const double derivative = DistortedThetaDerivative(d, theta);
    if (!std::isfinite(derivative) || derivative <= 1e-6)
      return maximum * (i - 1) / samples;
  }
  return maximum;
}

std::array<double, 9> Inverse(const std::array<double, 9>& m) {
  const double determinant = m[0] * (m[4] * m[8] - m[5] * m[7]) -
      m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  if (!std::isfinite(determinant) || std::abs(determinant) < 1e-12)
    throw std::runtime_error("Singular fisheye rectification matrix");
  return {{(m[4]*m[8]-m[5]*m[7])/determinant, (m[2]*m[7]-m[1]*m[8])/determinant,
           (m[1]*m[5]-m[2]*m[4])/determinant, (m[5]*m[6]-m[3]*m[8])/determinant,
           (m[0]*m[8]-m[2]*m[6])/determinant, (m[2]*m[3]-m[0]*m[5])/determinant,
           (m[3]*m[7]-m[4]*m[6])/determinant, (m[1]*m[6]-m[0]*m[7])/determinant,
           (m[0]*m[4]-m[1]*m[3])/determinant}};
}

bool ValidSourcePixel(const FisheyeUniforms& u, double theta_limit,
                      double x, double y, int width, int height) {
  const auto& h = u.rectified_to_ray;
  const double z = h[2] * x + h[5] * y + h[8];
  if (z <= 1e-12) return false;
  const double a = (h[0] * x + h[3] * y + h[6]) / z;
  const double b = (h[1] * x + h[4] * y + h[7]) / z;
  const double radius = std::hypot(a, b);
  const double theta = std::atan(radius);
  if (!std::isfinite(theta) || theta >= theta_limit) return false;
  const auto& d = u.distortion;
  const double scale = radius < 1e-12 ? 1.0 : DistortedTheta(d, theta) / radius;
  const double source_x = u.intrinsics[0] * a * scale + u.intrinsics[2];
  const double source_y = u.intrinsics[1] * b * scale + u.intrinsics[3];
  // Leave a sampling margin inside each eye for bilinear filtering, floating
  // point error and changes in the received resolution.
  const double margin_x = std::max(2.0, width * 0.005);
  const double margin_y = std::max(2.0, height * 0.005);
  return std::isfinite(source_x) && std::isfinite(source_y) &&
      source_x >= margin_x && source_x <= width - 1 - margin_x &&
      source_y >= margin_y && source_y <= height - 1 - margin_y;
}

std::array<float, 4> CommonValidCrop(const StereoFisheyeCalibration& calibration) {
  const auto left = MakeFisheyeUniforms(calibration, 0, calibration.width, calibration.height);
  const auto right = MakeFisheyeUniforms(calibration, 1, calibration.width, calibration.height);
  const double left_limit = left.max_theta;
  const double right_limit = right.max_theta;
  const int columns = std::min(calibration.width, 1024);
  const int rows = std::min(calibration.height, 1024);
  double scale = 1.0;
  // Scan the whole image (including its interior), not just its border. Each
  // invalid sample limits the size of a centered rectangle with the same aspect.
  // Intersect the constraints from BOTH eyes to retain equal rectified y values.
  for (int row = 0; row <= rows; ++row) {
    const double v = static_cast<double>(row) / rows;
    for (int column = 0; column <= columns; ++column) {
      const double u = static_cast<double>(column) / columns;
      const double distance = std::max(std::abs(2 * u - 1), std::abs(2 * v - 1));
      if (distance >= scale) continue;
      const double x = u * calibration.width - 0.5;
      const double y = v * calibration.height - 0.5;
      if (!ValidSourcePixel(left, left_limit, x, y, calibration.width, calibration.height) ||
          !ValidSourcePixel(right, right_limit, x, y, calibration.width, calibration.height))
        scale = distance;
    }
  }
  // Inset by one grid cell to avoid retaining unsampled slivers of the curved edge.
  scale -= 2.0 / std::min(columns, rows);
  if (scale <= 0.05)
    throw std::runtime_error("Stereo fisheye calibration has no usable common central crop");
  const float inset = static_cast<float>((1 - scale) * 0.5);
  return {{inset, inset, 1 - inset, 1 - inset}};
}
}

StereoFisheyeCalibration LoadStereoFisheyeCalibration(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open stereo fisheye calibration: " + path);
  const auto root = ReadYamlFields(input);
  if (ReadScalar(root, "camera_model") != "opencv_fisheye_4")
    throw std::runtime_error("Expected opencv_fisheye_4 calibration: " + path);
  StereoFisheyeCalibration result;
  result.width = ReadPositiveInteger(root, "image_width");
  result.height = ReadPositiveInteger(root, "image_height");
  for (int i = 0; i < 2; ++i) {
    auto& eye = result.eyes[i];
    const std::string prefix = i == 0 ? "left" : "right";
    eye.camera = ReadMatrix<3, 3>(root, prefix + "_camera_matrix");
    eye.distortion = ReadMatrix<4, 1>(root, prefix + "_dist_coeffs");
    eye.rotation = ReadMatrix<3, 3>(root, i == 0 ? "R1" : "R2");
    eye.projection = ReadMatrix<3, 4>(root, i == 0 ? "P1" : "P2");
    ValidateEye(eye);
    eye.max_theta = MonotonicThetaLimit(eye.distortion);
    // Validate matrix invertibility before the first received frame.
    MakeFisheyeUniforms(result, i, result.width, result.height);
  }
  result.valid_crop = CommonValidCrop(result);
  return result;
}

StereoFisheyeCalibration LoadSingleFisheyeCalibration(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open surround fisheye calibration: " + path);
  const auto root = ReadYamlFields(input);
  if (ReadScalar(root, "camera_model") != "opencv_fisheye_4")
    throw std::runtime_error("Expected opencv_fisheye_4 calibration: " + path);
  StereoFisheyeCalibration result;
  result.width = ReadPositiveInteger(root, "image_width");
  result.height = ReadPositiveInteger(root, "image_height");
  auto& eye = result.eyes[0];
  eye.camera = ReadMatrix<3, 3>(root, "camera_matrix");
  eye.distortion = ReadMatrix<4, 1>(root, "dist_coeffs");
  const auto projection = ReadMatrix<3, 3>(root, "new_camera_matrix");
  eye.rotation = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      eye.projection[row * 4 + col] = projection[row * 3 + col];
  ValidateEye(eye);
  eye.max_theta = MonotonicThetaLimit(eye.distortion);
  MakeFisheyeUniforms(result, 0, result.width, result.height);
  result.eyes[1] = eye;
  result.valid_crop = CommonValidCrop(result);
  return result;
}

bool FisheyeResolutionMatches(const StereoFisheyeCalibration& calibration, int width, int height) {
  if (width <= 0 || height <= 0 || calibration.width <= 0 || calibration.height <= 0) return false;
  const double x_scale = static_cast<double>(width) / calibration.width;
  const double y_scale = static_cast<double>(height) / calibration.height;
  // Permit pixel rounding but reject cropped/rotated/stretched video modes.
  return std::abs(x_scale / y_scale - 1) <= 0.001;
}

FisheyeUniforms MakeFisheyeUniforms(const StereoFisheyeCalibration& calibration,
                                  int eye_index, int width, int height) {
  if (eye_index < 0 || eye_index > 1 || width <= 0 || height <= 0 ||
      calibration.width <= 0 || calibration.height <= 0)
    throw std::runtime_error("Invalid stereo fisheye frame dimensions");
  const auto& eye = calibration.eyes[eye_index];
  const double sx = static_cast<double>(width) / calibration.width;
  const double sy = static_cast<double>(height) / calibration.height;
  std::array<double, 9> product{};
  // H = scaled(P[0:3,0:3]) * R; its inverse maps an output pixel to a source ray.
  // P's fourth column describes baseline/depth, not the single-camera pixel map.
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      for (int k = 0; k < 3; ++k)
        product[row * 3 + col] += (row == 0 ? sx : row == 1 ? sy : 1.0) *
            eye.projection[row * 4 + k] * eye.rotation[k * 3 + col];
  const auto inverse = Inverse(product);
  FisheyeUniforms result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result.rectified_to_ray[col * 3 + row] = static_cast<float>(inverse[row * 3 + col]);
  result.intrinsics = {{static_cast<float>(eye.camera[0] * sx),
      static_cast<float>(eye.camera[4] * sy), static_cast<float>(eye.camera[2] * sx),
      static_cast<float>(eye.camera[5] * sy)}};
  for (size_t i = 0; i < eye.distortion.size(); ++i)
    result.distortion[i] = static_cast<float>(eye.distortion[i]);
  result.max_theta = static_cast<float>(std::min(eye.max_theta,
      MonotonicThetaLimit(result.distortion)));
  for (float value : result.rectified_to_ray)
    if (!std::isfinite(value)) throw std::runtime_error("Fisheye mapping exceeds GPU precision");
  for (float value : result.intrinsics)
    if (!std::isfinite(value)) throw std::runtime_error("Fisheye intrinsics exceed GPU precision");
  for (float value : result.distortion)
    if (!std::isfinite(value)) throw std::runtime_error("Fisheye distortion exceeds GPU precision");
  return result;
}

bool FisheyeRectifiedPoint(const FisheyeEye& eye, double x, double y,
                          double* rectified_x, double* rectified_y) {
  const double a = (x - eye.camera[2]) / eye.camera[0];
  const double b = (y - eye.camera[5]) / eye.camera[4];
  const double distorted_theta = std::hypot(a, b);
  const auto& d = eye.distortion;
  if (!std::isfinite(distorted_theta)) return false;
  // Safeguarded Newton iteration: keep each step in the physical angular
  // interval, falling back to bisection instead of jumping across radial folds.
  double low = 0, high = eye.max_theta;
  if (distorted_theta > DistortedTheta(d, high)) return false;
  double theta = std::min(distorted_theta, high);
  for (int i = 0; i < 40; ++i) {
    const double f = DistortedTheta(d, theta) - distorted_theta;
    if (std::abs(f) < 1e-10) break;
    if (f > 0) high = theta; else low = theta;
    const double derivative = DistortedThetaDerivative(d, theta);
    const double next = derivative > 1e-6 ? theta - f / derivative : -1;
    theta = std::isfinite(next) && next > low && next < high ? next : (low + high) * 0.5;
  }
  if (theta < 0 || theta > eye.max_theta ||
      std::abs(DistortedTheta(d, theta) - distorted_theta) > 1e-7) return false;
  const double scale = distorted_theta < 1e-12 ? 1.0 : std::tan(theta) / distorted_theta;
  const double ray[3] = {a * scale, b * scale, 1};
  double rotated[3]{};
  for (int row = 0; row < 3; ++row)
    for (int k = 0; k < 3; ++k) rotated[row] += eye.rotation[row*3+k] * ray[k];
  double projected[3]{};
  for (int row = 0; row < 3; ++row)
    for (int k = 0; k < 3; ++k) projected[row] += eye.projection[row*4+k] * rotated[k];
  if (projected[2] <= 1e-12) return false;
  *rectified_x = projected[0] / projected[2];
  *rectified_y = projected[1] / projected[2];
  return std::isfinite(*rectified_x) && std::isfinite(*rectified_y);
}
}  // namespace rtc_console
