#include "electro_optical_sensor/foundation/EosOpticalCharacteristics.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

namespace electro_optical_sensor {
namespace foundation {
namespace optics {

namespace {

float ComputeSlantRangeByDepressionAngle(float platform_altitude_m, float depression_deg) {
  const float safe_altitude_m = oneq::common::numerics::SafePositive(platform_altitude_m, 1.0f);
  const float safe_depression_deg =
      oneq::common::numerics::Clamp(depression_deg, 1.0f, 89.0f);
  const float depression_rad = safe_depression_deg * constants::kPi / 180.0f;
  return safe_altitude_m / std::sin(depression_rad);
}

}  // namespace

float ComputeFocalHeightRatio(float focal_length_m, float platform_altitude_m) {
  return oneq::common::numerics::SafePositive(focal_length_m, 0.1f) / oneq::common::numerics::SafePositive(platform_altitude_m, 1.0f);
}

float ComputeInstantaneousFovDeg(float detector_size_m, float focal_length_m) {
  const float safe_detector_size_m = oneq::common::numerics::SafePositive(detector_size_m, 1.0e-3f);
  const float safe_focal_length_m = oneq::common::numerics::SafePositive(focal_length_m, 0.1f);
  const float fov_rad = 2.0f * std::atan(0.5f * safe_detector_size_m / safe_focal_length_m);
  return fov_rad * 180.0f / constants::kPi;
}

float ComputeGroundScanWidthM(float platform_altitude_m, float fov_deg) {
  const float safe_altitude_m = oneq::common::numerics::SafePositive(platform_altitude_m, 1.0f);
  const float safe_fov_deg = std::max(0.0f, fov_deg);
  const float half_fov_rad = 0.5f * safe_fov_deg * constants::kPi / 180.0f;
  return 2.0f * safe_altitude_m * std::tan(half_fov_rad);
}

float ComputeGroundProjectionDistanceM(float platform_altitude_m, float look_angle_deg) {
  const float safe_altitude_m = oneq::common::numerics::SafePositive(platform_altitude_m, 1.0f);
  const float clamped_look_angle_deg =
      oneq::common::numerics::Clamp(look_angle_deg, -89.9f, 89.9f);
  const float look_angle_rad = clamped_look_angle_deg * constants::kPi / 180.0f;
  return std::max(0.0f, safe_altitude_m * std::tan(look_angle_rad));
}

float ComputeDiffractionLimitedAngularResolutionRad(float wavelength_um, float aperture_diameter_m) {
  const float safe_wavelength_m = oneq::common::numerics::SafePositive(wavelength_um * 1.0e-6f, 4.0e-6f);
  const float safe_aperture_m = oneq::common::numerics::SafePositive(aperture_diameter_m, 0.1f);
  return 1.22f * safe_wavelength_m / safe_aperture_m;
}

float ComputeGroundSampleDistanceM(float range_m, float angular_resolution_rad) {
  const float safe_range_m = oneq::common::numerics::SafePositive(range_m, 1.0f);
  const float safe_angular_resolution_rad = std::max(0.0f, angular_resolution_rad);
  return safe_range_m * safe_angular_resolution_rad;
}

float ComputeMinimumDetectionRangeM(const DetectionRangeInputs& inputs) {
  const float half_fov_deg = 0.5f * std::max(0.0f, inputs.vertical_fov_deg);
  const float near_edge_depression_deg = inputs.boresight_depression_deg + half_fov_deg;
  const float clamped_depression_deg = oneq::common::numerics::Clamp(
      near_edge_depression_deg, inputs.min_depression_deg, inputs.max_depression_deg);
  return ComputeSlantRangeByDepressionAngle(inputs.platform_altitude_m, clamped_depression_deg);
}

float ComputeMaximumDetectionRangeM(const DetectionRangeInputs& inputs) {
  const float half_fov_deg = 0.5f * std::max(0.0f, inputs.vertical_fov_deg);
  const float far_edge_depression_deg = inputs.boresight_depression_deg - half_fov_deg;
  const float clamped_depression_deg = oneq::common::numerics::Clamp(
      far_edge_depression_deg, inputs.min_depression_deg, inputs.max_depression_deg);
  return ComputeSlantRangeByDepressionAngle(inputs.platform_altitude_m, clamped_depression_deg);
}

float ComputeDefocusCoefficient(float focus_distance_m, float target_distance_m) {
  const float safe_focus_distance_m = oneq::common::numerics::SafePositive(focus_distance_m, 1.0f);
  const float safe_target_distance_m = oneq::common::numerics::SafePositive(target_distance_m, 1.0f);
  return std::fabs((safe_target_distance_m - safe_focus_distance_m) / safe_focus_distance_m);
}

float ComputeCircleOfConfusionDiameterM(float aperture_diameter_m, float focal_length_m,
                                        float focus_distance_m, float target_distance_m) {
  const float safe_aperture_m = oneq::common::numerics::SafePositive(aperture_diameter_m, 0.1f);
  const float safe_focal_length_m = oneq::common::numerics::SafePositive(focal_length_m, 0.1f);
  const float defocus = ComputeDefocusCoefficient(focus_distance_m, target_distance_m);
  return safe_aperture_m * defocus * safe_focal_length_m / oneq::common::numerics::SafePositive(focus_distance_m, 1.0f);
}

float ComputeRefractiveShiftM(float range_m, float refractive_index) {
  const float safe_range_m = oneq::common::numerics::SafePositive(range_m, 1.0f);
  const float safe_refractive_index = std::max(refractive_index, 1.0f);
  return safe_range_m * (safe_refractive_index - 1.0f);
}

}  // namespace optics
}  // namespace foundation
}  // namespace electro_optical_sensor
