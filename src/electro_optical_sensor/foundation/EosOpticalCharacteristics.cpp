#include "1q/electro_optical_sensor/foundation/EosOpticalCharacteristics.h"

#include <algorithm>
#include <cmath>

namespace electro_optical_sensor {
namespace foundation {
namespace optics {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float SafePositive(float value, float fallback) {
  if (std::isfinite(value) == 0 || value <= 0.0f) {
    return fallback;
  }
  return value;
}

}  // namespace

float ComputeFocalHeightRatio(float focal_length_m, float platform_altitude_m) {
  return SafePositive(focal_length_m, 0.1f) / SafePositive(platform_altitude_m, 1.0f);
}

float ComputeInstantaneousFovDeg(float detector_size_m, float focal_length_m) {
  const float safe_detector_size_m = SafePositive(detector_size_m, 1.0e-3f);
  const float safe_focal_length_m = SafePositive(focal_length_m, 0.1f);
  const float fov_rad = 2.0f * std::atan(0.5f * safe_detector_size_m / safe_focal_length_m);
  return fov_rad * 180.0f / kPi;
}

float ComputeGroundScanWidthM(float platform_altitude_m, float fov_deg) {
  const float safe_altitude_m = SafePositive(platform_altitude_m, 1.0f);
  const float safe_fov_deg = std::max(0.0f, fov_deg);
  const float half_fov_rad = 0.5f * safe_fov_deg * kPi / 180.0f;
  return 2.0f * safe_altitude_m * std::tan(half_fov_rad);
}

float ComputeGroundProjectionDistanceM(float platform_altitude_m, float look_angle_deg) {
  const float safe_altitude_m = SafePositive(platform_altitude_m, 1.0f);
  const float look_angle_rad = look_angle_deg * kPi / 180.0f;
  return safe_altitude_m * std::tan(look_angle_rad);
}

float ComputeDiffractionLimitedAngularResolutionRad(float wavelength_um, float aperture_diameter_m) {
  const float safe_wavelength_m = SafePositive(wavelength_um * 1.0e-6f, 4.0e-6f);
  const float safe_aperture_m = SafePositive(aperture_diameter_m, 0.1f);
  return 1.22f * safe_wavelength_m / safe_aperture_m;
}

float ComputeGroundSampleDistanceM(float range_m, float angular_resolution_rad) {
  const float safe_range_m = SafePositive(range_m, 1.0f);
  const float safe_angular_resolution_rad = std::max(0.0f, angular_resolution_rad);
  return safe_range_m * safe_angular_resolution_rad;
}

float ComputeDefocusCoefficient(float focus_distance_m, float target_distance_m) {
  const float safe_focus_distance_m = SafePositive(focus_distance_m, 1.0f);
  const float safe_target_distance_m = SafePositive(target_distance_m, 1.0f);
  return std::fabs((safe_target_distance_m - safe_focus_distance_m) / safe_focus_distance_m);
}

float ComputeCircleOfConfusionDiameterM(float aperture_diameter_m, float focal_length_m,
                                        float focus_distance_m, float target_distance_m) {
  const float safe_aperture_m = SafePositive(aperture_diameter_m, 0.1f);
  const float safe_focal_length_m = SafePositive(focal_length_m, 0.1f);
  const float defocus = ComputeDefocusCoefficient(focus_distance_m, target_distance_m);
  return safe_aperture_m * defocus * safe_focal_length_m / SafePositive(focus_distance_m, 1.0f);
}

float ComputeRefractiveShiftM(float range_m, float refractive_index) {
  const float safe_range_m = SafePositive(range_m, 1.0f);
  const float safe_refractive_index = std::max(refractive_index, 1.0f);
  return safe_range_m * (safe_refractive_index - 1.0f);
}

}  // namespace optics
}  // namespace foundation
}  // namespace electro_optical_sensor
