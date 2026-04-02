#include "airborne_radar/environment/simulation/PropagationModel.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "common/atmosphere/AtmospherePhysics.h"
#include "common/rcs/RcsPhysics.h"

namespace airborne_radar {
namespace environment {
namespace simulation {

namespace {

float ClampFloat(float value, float lower_bound, float upper_bound) {
  return std::min(std::max(value, lower_bound), upper_bound);
}

float DbToLinear(float db_value) { return std::pow(10.0f, db_value / 10.0f); }

float LinearToDb(float linear_value) {
  const float safe_linear_value = std::max(linear_value, 1.0e-12f);
  return 10.0f * std::log10(safe_linear_value);
}

float ComputeVegetationClutterMultiplier(
    const environment::VegetationScatterPhysicsConfig& config) {
  oneq::internal::rcs::TreeScattererConfig scatterer_config;
  scatterer_config.leaf_count = static_cast<std::size_t>(std::max(config.leaf_count, 1U));
  scatterer_config.canopy_radius_m = std::max(config.canopy_radius_m, 0.1f);
  scatterer_config.canopy_height_m = std::max(config.canopy_height_m, 0.1f);
  const oneq::internal::rcs::TreeScattererState scatterer_state =
      oneq::internal::rcs::InitTreeScatterer_AVX(scatterer_config);

  std::vector<float> x_param;
  std::vector<float> y_param;
  oneq::internal::rcs::ComputeLeavesParamEq_ymm8r4(
      scatterer_state, std::max(config.x_axis_scale_m, 0.0f), std::max(config.y_axis_scale_m, 0.0f),
      &x_param, &y_param);

  const oneq::internal::rcs::LeafPhaseMatrices phase_matrices =
      oneq::internal::rcs::compute_leaf_phase_matrices(
          std::max(config.leaf_size_m, 0.0f), std::max(config.dielectric_constant_real, 1.0f),
          config.incidence_deg, config.scatter_deg);

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
  return ClampFloat(unclamped_multiplier, 1.0f, std::max(config.max_physical_multiplier, 1.0f));
}

}  // namespace

PropagationResult PropagationModel::Evaluate(const EnvironmentSceneState& scene_state) const {
  PropagationResult result;
  float physical_loss_db = 0.0f;
  if (scene_state.atmospheric_physics.enable_physical_model) {
    oneq::internal::atmosphere::AtmosphericPropagationInputs inputs;
    inputs.enable_physics = true;
    inputs.frequency_hz = scene_state.atmospheric_physics.frequency_hz;
    inputs.path_length_m = scene_state.atmospheric_physics.path_length_m;
    inputs.radar_altitude_m = scene_state.atmospheric_physics.radar_altitude_m;
    inputs.target_altitude_m = scene_state.atmospheric_physics.target_altitude_m;
    inputs.elevation_deg = scene_state.atmospheric_physics.elevation_deg;
    inputs.pressure_hpa = scene_state.atmospheric_physics.pressure_hpa;
    inputs.temperature_k = scene_state.atmospheric_physics.temperature_k;
    inputs.relative_humidity = scene_state.atmospheric_physics.relative_humidity;
    inputs.k_factor = scene_state.atmospheric_physics.k_factor;
    inputs.day_of_year = scene_state.atmospheric_physics.day_of_year;
    inputs.solar_flux_f107a = scene_state.atmospheric_physics.solar_flux_f107a;
    inputs.solar_flux_f107 = scene_state.atmospheric_physics.solar_flux_f107;
    inputs.geomagnetic_ap = scene_state.atmospheric_physics.geomagnetic_ap;
    const oneq::internal::atmosphere::AtmosphericPropagationResult physics_result =
        oneq::internal::atmosphere::EvaluateAtmosphericPropagation(inputs);
    physical_loss_db = physics_result.total_physics_loss_db;
  }
  // terrain_reflection_db 可为负值（多径增益），不对总传播损耗做下限钳位。
  result.propagation_loss_db = scene_state.base_propagation_loss_db +
                               scene_state.atmospheric_attenuation_db +
                               scene_state.terrain_reflection_db + physical_loss_db;
  float clutter_power_db = scene_state.clutter_power_db;
  if (scene_state.vegetation_scatter_physics.enable_physical_model) {
    const float mix_ratio =
        ClampFloat(scene_state.vegetation_scatter_physics.clutter_mix_ratio, 0.0f, 1.0f);
    if (mix_ratio > 0.0f) {
      const float baseline_clutter_w = DbToLinear(clutter_power_db);
      const float physical_multiplier =
          ComputeVegetationClutterMultiplier(scene_state.vegetation_scatter_physics);
      const float physical_clutter_w = baseline_clutter_w * physical_multiplier;
      const float mixed_clutter_w =
          baseline_clutter_w * (1.0f - mix_ratio) + physical_clutter_w * mix_ratio;
      clutter_power_db = LinearToDb(mixed_clutter_w);
    }
  }
  result.clutter_power_db = clutter_power_db;
  return result;
}

}  // namespace simulation
}  // namespace environment
}  // namespace airborne_radar
