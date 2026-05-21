#include "electro_optical_sensor/foundation/EosStrayLight.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

namespace electro_optical_sensor {
namespace foundation {
namespace stray_light {

namespace {


float ComputeAngularSeparationDeg(float az0_deg, float el0_deg, float az1_deg, float el1_deg) {
  const float el0_rad = oneq::internal::numerics::DegToRad(el0_deg);
  const float el1_rad = oneq::internal::numerics::DegToRad(el1_deg);
  const float delta_az_rad = oneq::internal::numerics::DegToRad(
      oneq::internal::numerics::NormalizeAngle180(az0_deg - az1_deg));

  float cosine_angle = std::sin(el0_rad) * std::sin(el1_rad) +
                       std::cos(el0_rad) * std::cos(el1_rad) * std::cos(delta_az_rad);
  cosine_angle = oneq::internal::numerics::Clamp(cosine_angle, -1.0f, 1.0f);
  return std::acos(cosine_angle) * 180.0f / constants::kPi;
}

}  // namespace

StrayLightFilterResult EvaluateStrayLightFilter(const StrayLightFilterInputs& inputs) {
  StrayLightFilterResult result;
  const float cloud_ratio = oneq::internal::numerics::Clamp01(inputs.cloud_coverage_ratio);
  const float inner_half_angle_deg = std::max(0.0f, inputs.hood_inner_half_angle_deg);
  const float outer_half_angle_deg =
      std::max(inner_half_angle_deg + 1.0f, inputs.hood_outer_half_angle_deg);
  const float min_suppression_ratio =
      oneq::internal::numerics::Clamp(
          std::min(inputs.min_suppression_ratio, inputs.max_suppression_ratio), 0.0f, 1.0f);
  const float max_suppression_ratio =
      oneq::internal::numerics::Clamp(
          std::max(inputs.min_suppression_ratio, inputs.max_suppression_ratio), 0.0f, 1.0f);

  result.sun_separation_deg = ComputeAngularSeparationDeg(
      inputs.target_azimuth_deg, inputs.target_elevation_deg, inputs.sun_azimuth_deg,
      inputs.sun_altitude_deg);
  const float separation_span_deg = std::max(outer_half_angle_deg - inner_half_angle_deg, 0.001f);
  const float normalized_separation =
      oneq::internal::numerics::Clamp((result.sun_separation_deg - inner_half_angle_deg) /
                                          separation_span_deg,
                                      0.0f, 1.0f);

  const float clear_sky_contamination = 1.0f - normalized_separation;
  result.contamination_ratio = clear_sky_contamination * (1.0f - 0.7f * cloud_ratio);
  if (inputs.enabled) {
    result.suppression_ratio = min_suppression_ratio +
                               (max_suppression_ratio - min_suppression_ratio) *
                                   normalized_separation;
  } else {
    result.suppression_ratio = 0.0f;
  }

  const float residual_contamination =
      result.contamination_ratio * (1.0f - result.suppression_ratio);
  result.background_penalty_scale = 1.0f + 2.0f * residual_contamination;
  result.signal_transmission_scale =
      oneq::internal::numerics::Clamp(1.0f - 0.15f * residual_contamination, 0.7f, 1.0f);
  return result;
}

}  // namespace stray_light
}  // namespace foundation
}  // namespace electro_optical_sensor
