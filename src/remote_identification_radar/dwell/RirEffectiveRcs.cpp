#include "remote_identification_radar/dwell/RirEffectiveRcs.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"
#include "common/rcs/RcsPhysics.h"

namespace remote_identification_radar {
namespace dwell {
namespace {

float ComputeEquivalentRadiusM(float input_rcs_m2,
                             const config::hardware::RirRcsPhysicsConfig& rcs_config) {
  const float min_radius_m = std::max(rcs_config.min_equivalent_radius_m, 1.0e-3f);
  const float max_radius_m = std::max(rcs_config.max_equivalent_radius_m, min_radius_m);
  const float safe_input_rcs_m2 = std::max(input_rcs_m2, 0.0f);
  const float equivalent_radius_m =
      std::sqrt(safe_input_rcs_m2 / static_cast<float>(oneq::common::numerics::kPi));
  return oneq::common::numerics::Clamp(equivalent_radius_m, min_radius_m, max_radius_m);
}

}  // namespace

float ComputeEffectiveTargetRcsM2(const session::RirSceneTarget& target,
                                  const RirTargetLookAngles& look_angles,
                                  const config::hardware::RirRcsPhysicsConfig& rcs_config,
                                  float carrier_hz) {
  const float input_rcs_m2 = std::max(target.rcs, 0.0f);
  if (!rcs_config.enable_physical_rcs) {
    return input_rcs_m2;
  }

  const float mix_ratio = oneq::common::numerics::Clamp(rcs_config.physics_mix_ratio, 0.0f, 1.0f);
  if (mix_ratio <= 0.0f) {
    return input_rcs_m2;
  }

  if (carrier_hz <= 0.0f) {
    return input_rcs_m2;
  }

  const float wavenumber_k0 = 2.0f * static_cast<float>(oneq::common::numerics::kPi) *
                              carrier_hz /
                              static_cast<float>(oneq::common::numerics::kLightSpeed);
  if (wavenumber_k0 <= 0.0f) {
    return input_rcs_m2;
  }

  const float equivalent_radius_m = ComputeEquivalentRadiusM(input_rcs_m2, rcs_config);
  const float azimuth_deg =
      look_angles.has_look_angles ? std::fabs(look_angles.look_az_deg) : 0.0f;
  const float elevation_deg =
      look_angles.has_look_angles ? std::fabs(look_angles.look_el_deg) : 0.0f;
  const float psi_i_deg = oneq::common::numerics::Clamp(elevation_deg, 0.0f, 89.0f);
  const float psi_s_deg = oneq::common::numerics::Clamp(
      psi_i_deg + std::fabs(rcs_config.bistatic_psi_offset_deg), 0.0f, 89.0f);

  const float cylinder_rcs_m2 =
      oneq::common::rcs::ComputeCylinderRcs(equivalent_radius_m, wavenumber_k0);
  const float bistatic_rcs_m2 = oneq::common::rcs::ComputeBistaticCylinderRcs(
      wavenumber_k0, equivalent_radius_m, psi_i_deg, psi_s_deg, azimuth_deg);
  const float planar_rcs_m2 =
      oneq::common::rcs::ComputePlanarPlateRcs(wavenumber_k0, equivalent_radius_m, elevation_deg);

  const float cylinder_weight = oneq::common::numerics::Clamp(rcs_config.cylinder_weight, 0.0f, 1.0f);
  const float physical_rcs_m2 = cylinder_weight * (0.5f * (cylinder_rcs_m2 + bistatic_rcs_m2)) +
                                (1.0f - cylinder_weight) * planar_rcs_m2;

  const float min_rcs_m2 = std::max(rcs_config.min_rcs_m2, 0.0f);
  const float max_rcs_m2 = std::max(rcs_config.max_rcs_m2, min_rcs_m2);
  const float clamped_physical_rcs_m2 =
      oneq::common::numerics::Clamp(physical_rcs_m2, min_rcs_m2, max_rcs_m2);
  return input_rcs_m2 * (1.0f - mix_ratio) + clamped_physical_rcs_m2 * mix_ratio;
}

}  // namespace dwell
}  // namespace remote_identification_radar
