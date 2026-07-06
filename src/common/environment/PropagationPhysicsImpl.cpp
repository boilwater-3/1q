/**
 * @file PropagationPhysicsImpl.cpp
 * @brief 实现大气传播物理算法公开 API（转发到 common/atmosphere）。
 */

#include "1q/environment/AtmosphericTypes.h"
#include "1q/environment/PropagationPhysics.h"

#include "common/atmosphere/AtmospherePhysics.h"

namespace oneq {
namespace environment {

namespace {

common::atmosphere::AtmosphericObservationRef ToInternalRef(
    const AtmosphericObservation& obs, const SpaceWeatherContext* ctx) {
  common::atmosphere::AtmosphericObservationRef ref;
  ref.pressure_hpa = obs.pressure_hpa;
  ref.temperature_k = obs.temperature_k;
  ref.relative_humidity = obs.relative_humidity;
  if (ctx != nullptr) {
    ref.k_factor = ResolveEffectiveKFactor(*ctx);
    ref.day_of_year = ResolveEffectiveDayOfYear(*ctx);
    ref.solar_flux_f107a = ctx->solar_flux_f107a;
    ref.solar_flux_f107 = ctx->solar_flux_f107;
    ref.geomagnetic_ap = ctx->geomagnetic_ap;
  } else {
    ref.k_factor = ResolveEffectiveKFactor(obs);
    ref.day_of_year = 172;
    ref.solar_flux_f107a = 150.0f;
    ref.solar_flux_f107 = 150.0f;
    ref.geomagnetic_ap = 4.0f;
  }
  return ref;
}

std::int64_t FloorDiv(std::int64_t numerator, std::int64_t denominator) {
  const std::int64_t quotient = numerator / denominator;
  const std::int64_t remainder = numerator % denominator;
  if (remainder != 0 && ((remainder > 0) != (denominator > 0))) {
    return quotient - 1;
  }
  return quotient;
}

}  // namespace

std::int32_t ResolveEffectiveDayOfYearFromUnix(std::int64_t unix_seconds) {
  const std::int64_t days_since_epoch = FloorDiv(unix_seconds, 86400);
  std::int64_t z = days_since_epoch + 719468;
  const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const std::int64_t doe = z - era * 146097;
  const std::int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const std::int64_t mp = (5 * doy + 2) / 153;
  const std::int32_t day = static_cast<std::int32_t>(doy - (153 * mp + 2) / 5 + 1);
  const std::int32_t month = static_cast<std::int32_t>(mp + (mp < 10 ? 3 : -9));
  const std::int32_t year = static_cast<std::int32_t>(yoe + era * 400 + (month <= 2 ? 1 : 0));

  const bool is_leap_year = (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
  static const std::int32_t cumulative_days_before_month[12] = {0,   31,  59,  90,  120, 151,
                                                                181, 212, 243, 273, 304, 334};
  const std::int32_t base = cumulative_days_before_month[month - 1];
  const std::int32_t leap_day = (is_leap_year && month > 2) ? 1 : 0;
  return base + day + leap_day;
}

PropagationResult EvaluatePropagation(const PropagationInputs& inputs) {
  const auto ref = ToInternalRef(
      inputs.observation,
      inputs.has_space_weather_context ? &inputs.space_weather_context : nullptr);
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

float RefractivityIndex(float tc_celsius, float tk_kelvin, float pd_hpa,
                        float p_hpa, float h_rel, int water_or_ice) {
  return common::atmosphere::refractivity_index_n_r4(
      tc_celsius, tk_kelvin, pd_hpa, p_hpa, h_rel, water_or_ice);
}

}  // namespace environment
}  // namespace oneq
