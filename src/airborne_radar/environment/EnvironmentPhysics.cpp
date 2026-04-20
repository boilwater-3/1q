#include <cmath>

#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "airborne_radar/environment/CalendarUtils.h"

namespace airborne_radar {
namespace environment {

JammingSensitivityProfile ResolveJammingSensitivityProfile(float threshold_db) {
  if (threshold_db <= 5.0f) {
    return JammingSensitivityProfile::kStrict;
  }
  if (threshold_db >= 7.0f) {
    return JammingSensitivityProfile::kRelaxed;
  }
  return JammingSensitivityProfile::kBalanced;
}

float ResolveEffectiveKFactor(const AtmosphericDerivedContext& context,
                              const AtmosphericPhysicsConfig& physics) {
  (void)context;

  const float temperature_k = (physics.temperature_k > 1.0f) ? physics.temperature_k : 288.15f;
  const float pressure_hpa = (physics.pressure_hpa > 0.0f) ? physics.pressure_hpa : 1013.25f;
  const float relative_humidity =
      physics.relative_humidity < 0.0f
          ? 0.0f
          : (physics.relative_humidity > 1.0f ? 1.0f : physics.relative_humidity);

  const float temperature_c = temperature_k - 273.15f;
  const float saturation_vapor_pressure_hpa =
      6.1121f * std::exp((17.502f * temperature_c) / (240.97f + temperature_c));
  const float water_vapor_pressure_hpa = relative_humidity * saturation_vapor_pressure_hpa;

  const float refractivity_n =
      77.6f * (pressure_hpa / temperature_k) +
      3.73e5f * (water_vapor_pressure_hpa / (temperature_k * temperature_k));

  const float refractivity_scale_height_km = 7.35f;
  const float dndh_n_per_km = -refractivity_n / refractivity_scale_height_km;

  const float denominator = 1.0f + dndh_n_per_km / 157.0f;
  if (denominator <= 0.1f) {
    return 4.0f / 3.0f;
  }
  const float derived_k_factor = 1.0f / denominator;
  if (derived_k_factor < 0.5f || derived_k_factor > 2.5f) {
    return 4.0f / 3.0f;
  }
  return derived_k_factor;
}

std::int32_t ResolveEffectiveDayOfYear(const AtmosphericDerivedContext& context) {
  if (!context.has_simulation_unix_seconds) {
    return 172;
  }
  return ResolveDayOfYearFromUnixSeconds(context.simulation_unix_seconds);
}

}  // namespace environment
}  // namespace airborne_radar
