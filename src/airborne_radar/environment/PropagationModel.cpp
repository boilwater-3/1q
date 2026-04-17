#include "airborne_radar/environment/PropagationModel.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "common/atmosphere/AtmospherePhysics.h"
#include "common/rcs/RcsPhysics.h"

namespace airborne_radar {
namespace environment {

namespace {

constexpr float kInternalBasePropagationLossDb = 4.0f;
constexpr float kInternalAtmosphericAttenuationDb = 1.5f;
constexpr float kInternalTerrainReflectionDb = 1.0f;
constexpr float kInternalBaselineClutterPowerDb = 3.0f;
constexpr float kRadToDeg = 57.2957795f;

float ClampFloat(float value, float lower_bound, float upper_bound) {
  return std::min(std::max(value, lower_bound), upper_bound);
}

float DbToLinear(float db_value) { return std::pow(10.0f, db_value / 10.0f); }

float LinearToDb(float linear_value) {
  const float safe_linear_value = std::max(linear_value, 1.0e-12f);
  return 10.0f * std::log10(safe_linear_value);
}

VegetationScatterPhysicsConfig ResolveVegetationScatterConfig(
    const VegetationScatterPhysicsConfig& config) {
  VegetationScatterPhysicsConfig resolved = config;
  switch (config.cover_profile) {
    case VegetationCoverProfile::kDisabled:
      break;
    case VegetationCoverProfile::kOpenGrassland:
      resolved.leaf_size_m = 0.02f;
      resolved.dielectric_constant_real = 1.8f;
      resolved.leaf_count = 24U;
      resolved.canopy_radius_m = 1.5f;
      resolved.canopy_height_m = 0.8f;
      break;
    case VegetationCoverProfile::kSparseWoodland:
      resolved.leaf_size_m = 0.04f;
      resolved.dielectric_constant_real = 2.2f;
      resolved.leaf_count = 56U;
      resolved.canopy_radius_m = 2.8f;
      resolved.canopy_height_m = 3.0f;
      break;
    case VegetationCoverProfile::kDeciduousForest:
      resolved.leaf_size_m = 0.05f;
      resolved.dielectric_constant_real = 2.6f;
      resolved.leaf_count = 96U;
      resolved.canopy_radius_m = 4.0f;
      resolved.canopy_height_m = 6.0f;
      break;
    case VegetationCoverProfile::kConiferousForest:
      resolved.leaf_size_m = 0.035f;
      resolved.dielectric_constant_real = 2.4f;
      resolved.leaf_count = 120U;
      resolved.canopy_radius_m = 3.2f;
      resolved.canopy_height_m = 8.0f;
      break;
    case VegetationCoverProfile::kTropicalDense:
      resolved.leaf_size_m = 0.065f;
      resolved.dielectric_constant_real = 3.1f;
      resolved.leaf_count = 180U;
      resolved.canopy_radius_m = 4.8f;
      resolved.canopy_height_m = 9.0f;
      break;
  }
  return resolved;
}

float ResolveVegetationIncidenceDeg(const VegetationScatterPhysicsConfig& config) {
  const float canopy_slope_rad = std::atan2(std::max(config.canopy_height_m, 0.1f),
                                            std::max(config.canopy_radius_m, 0.1f));
  const float incidence_deg = 8.0f + canopy_slope_rad * kRadToDeg * 0.5f;
  return ClampFloat(incidence_deg, 8.0f, 75.0f);
}

float ResolveVegetationScatterDeg(const VegetationScatterPhysicsConfig& config,
                                  float incidence_deg) {
  const float canopy_aspect_ratio = std::max(config.canopy_height_m, 0.1f) /
                                    std::max(config.canopy_radius_m, 0.1f);
  const float anisotropy = std::fabs(canopy_aspect_ratio - 1.0f);
  const float scatter_deg = incidence_deg + 12.0f + 6.0f * ClampFloat(anisotropy, 0.0f, 2.0f);
  return ClampFloat(scatter_deg, 15.0f, 85.0f);
}

float ComputeVegetationClutterMultiplier(
    const VegetationScatterPhysicsConfig& config) {
  oneq::internal::rcs::TreeScattererConfig scatterer_config;
  scatterer_config.leaf_count = static_cast<std::size_t>(std::max(config.leaf_count, 1U));
  scatterer_config.canopy_radius_m = std::max(config.canopy_radius_m, 0.1f);
  scatterer_config.canopy_height_m = std::max(config.canopy_height_m, 0.1f);
  const oneq::internal::rcs::TreeScattererState scatterer_state =
      oneq::internal::rcs::InitTreeScatterer_AVX(scatterer_config);

  std::vector<float> x_param;
  std::vector<float> y_param;
  oneq::internal::rcs::ComputeLeavesParamEq_ymm8r4(
      scatterer_state, 0.8f, 0.5f,
      &x_param, &y_param);

  const float incidence_deg = ResolveVegetationIncidenceDeg(config);
  const float scatter_deg = ResolveVegetationScatterDeg(config, incidence_deg);
  const oneq::internal::rcs::LeafPhaseMatrices phase_matrices =
      oneq::internal::rcs::compute_leaf_phase_matrices(
          std::max(config.leaf_size_m, 0.0f), std::max(config.dielectric_constant_real, 1.0f),
          incidence_deg, scatter_deg);

  float average_extent = 0.0f;
  for (std::size_t i = 0; i < x_param.size(); ++i) {
    average_extent += std::sqrt(x_param[i] * x_param[i] + y_param[i] * y_param[i]);
  }
  if (!x_param.empty()) {
    average_extent /= static_cast<float>(x_param.size());
  }

  const float normalized_extent = average_extent / scatterer_config.canopy_radius_m;
  const float phase_energy =
      std::max(0.0f, phase_matrices.m11 + phase_matrices.m22 +
                         std::fabs(phase_matrices.m12) + std::fabs(phase_matrices.m21));
  const float density_term =
      std::log1p(static_cast<float>(scatterer_config.leaf_count)) * (1.0f + normalized_extent);

  const float unclamped_multiplier = 1.0f + 0.05f * phase_energy * density_term;
  return ClampFloat(unclamped_multiplier, 1.0f, 30.0f);
}

}  // namespace

PropagationResult PropagationModel::Evaluate(const EnvironmentSceneState& scene_state) const {
  PropagationResult result;
  float physical_loss_db = 0.0f;
  if (scene_state.atmospheric_physics.enable_physical_model) {
    oneq::internal::atmosphere::AtmosphericPropagationInputs inputs;
    inputs.enable_physics = true;
    inputs.pressure_hpa = scene_state.atmospheric_physics.pressure_hpa;
    inputs.temperature_k = scene_state.atmospheric_physics.temperature_k;
    inputs.relative_humidity = scene_state.atmospheric_physics.relative_humidity;
    inputs.k_factor = ResolveEffectiveKFactor(scene_state.atmospheric_context,
                                              scene_state.atmospheric_physics);
    inputs.day_of_year = ResolveEffectiveDayOfYear(scene_state.atmospheric_context);
    inputs.solar_flux_f107a = scene_state.atmospheric_context.solar_flux_f107a;
    inputs.solar_flux_f107 = scene_state.atmospheric_context.solar_flux_f107;
    inputs.geomagnetic_ap = scene_state.atmospheric_context.geomagnetic_ap;
    const oneq::internal::atmosphere::AtmosphericPropagationResult physics_result =
        oneq::internal::atmosphere::EvaluateAtmosphericPropagation(inputs);
    physical_loss_db = physics_result.total_physics_loss_db;
  }
  result.atmospheric_physics_loss_db = physical_loss_db;
  result.propagation_loss_db = kInternalBasePropagationLossDb +
                               kInternalAtmosphericAttenuationDb +
                               kInternalTerrainReflectionDb + physical_loss_db;
  float clutter_power_db = kInternalBaselineClutterPowerDb;
  const VegetationScatterPhysicsConfig vegetation_config =
      ResolveVegetationScatterConfig(scene_state.vegetation_scatter_physics);
  if (vegetation_config.enable_physical_model) {
    const float mix_ratio = 0.7f;
    if (mix_ratio > 0.0f) {
      const float baseline_clutter_w = DbToLinear(clutter_power_db);
      const float physical_multiplier =
          ComputeVegetationClutterMultiplier(vegetation_config);
      const float physical_clutter_w = baseline_clutter_w * physical_multiplier;
      const float mixed_clutter_w =
          baseline_clutter_w * (1.0f - mix_ratio) + physical_clutter_w * mix_ratio;
      clutter_power_db = LinearToDb(mixed_clutter_w);
    }
  }
  result.clutter_power_db = clutter_power_db;
  return result;
}

}  // namespace environment
}  // namespace airborne_radar
