#include "airborne_radar/environment/simulation/PropagationModel.h"

#include "common/atmosphere/AtmospherePhysics.h"

namespace airborne_radar {
namespace environment {
namespace simulation {

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
  result.clutter_power_db = scene_state.clutter_power_db;
  return result;
}

}  // namespace simulation
}  // namespace environment
}  // namespace airborne_radar
