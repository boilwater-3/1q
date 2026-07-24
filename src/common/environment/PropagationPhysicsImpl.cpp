/**
 * @file PropagationPhysicsImpl.cpp
 * @brief 实现大气传播物理算法公开 API（转发到 common/atmosphere）。
 */

#include "1q/environment/AtmosphericTypes.h"
#include "1q/environment/PropagationPhysics.h"

#include <cmath>

#include "common/atmosphere/AtmospherePhysics.h"

namespace oneq {
namespace environment {

namespace {

common::atmosphere::AtmosphericObservationRef ToInternalRef(const AtmosphericObservation& obs) {
  common::atmosphere::AtmosphericObservationRef ref;
  ref.pressure_hpa = obs.pressure_hpa;
  ref.temperature_k = obs.temperature_k;
  ref.relative_humidity = obs.relative_humidity;
  ref.k_factor = ResolveEffectiveKFactor(obs);
  ref.day_of_year = 172;
  ref.solar_flux_f107a = 150.0f;
  ref.solar_flux_f107 = 150.0f;
  ref.geomagnetic_ap = 4.0f;
  return ref;
}

}  // namespace

PropagationResult EvaluatePropagation(const PropagationInputs& inputs) {
  const auto ref = ToInternalRef(inputs.observation);
  const auto internal_inputs = common::atmosphere::BuildPropagationInputs(
      inputs.frequency_hz, inputs.path_length_m, inputs.radar_altitude_m,
      inputs.target_altitude_m, inputs.elevation_deg, ref);
  const auto internal_result =
      common::atmosphere::EvaluateAtmosphericPropagation(internal_inputs);

  PropagationResult result;
  result.blake_loss_db = internal_result.blake_loss_db;
  result.refractivity_index = internal_result.refractivity_index;
  result.refractivity_index_h = internal_result.refractivity_index_h;
  result.neutral_density_kg_m3 = internal_result.neutral_density_kg_m3;
  result.total_physics_loss_db = internal_result.total_physics_loss_db;
  return result;
}

PropagationInputs BuildPropagationInputs(
    float frequency_hz, float path_length_m, float radar_altitude_m,
    float target_altitude_m, float elevation_deg,
    const AtmosphericObservation& observation) {
  PropagationInputs inputs;
  inputs.frequency_hz = frequency_hz;
  inputs.path_length_m = path_length_m;
  inputs.radar_altitude_m = radar_altitude_m;
  inputs.target_altitude_m = target_altitude_m;
  inputs.elevation_deg = elevation_deg;
  inputs.observation = observation;
  return inputs;
}

float BlakeAtmosphericLoss(float altitude_m, float frequency_hz,
                           float elevation_deg, float range_m, float k_factor) {
  return common::atmosphere::blake_atmos_loss_r4_1(
      altitude_m, frequency_hz, elevation_deg, range_m, k_factor);
}

bool TryRefractivityIndex(const RefractivityInputs& inputs, float* refractivity_index) {
  constexpr float kKelvinOffset = 273.15f;
  constexpr float kTemperatureConsistencyToleranceK = 0.05f;
  const float expected_kelvin = inputs.temperature.celsius + kKelvinOffset;
  if (refractivity_index == nullptr || !std::isfinite(inputs.temperature.celsius) ||
      !std::isfinite(inputs.temperature.kelvin) || inputs.temperature.kelvin <= 0.0f ||
      std::fabs(inputs.temperature.kelvin - expected_kelvin) >
          kTemperatureConsistencyToleranceK ||
      !std::isfinite(inputs.partial_pressure_hpa) || inputs.partial_pressure_hpa < 0.0f ||
      !std::isfinite(inputs.total_pressure_hpa) || inputs.total_pressure_hpa < 0.0f ||
      !std::isfinite(inputs.relative_humidity) || inputs.relative_humidity < 0.0f ||
      inputs.relative_humidity > 1.0f ||
      (inputs.water_or_ice != 0 && inputs.water_or_ice != 1)) {
    return false;
  }

  const float candidate = common::atmosphere::refractivity_index_n_r4(
      inputs.temperature.celsius, inputs.temperature.kelvin,
      inputs.partial_pressure_hpa, inputs.total_pressure_hpa,
      inputs.relative_humidity, inputs.water_or_ice);
  if (!std::isfinite(candidate)) {
    return false;
  }
  *refractivity_index = candidate;
  return true;
}

}  // namespace environment
}  // namespace oneq
