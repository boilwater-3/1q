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
  return oneq::foundation::ResolveEffectiveKFactor(physics);
}

std::int32_t ResolveEffectiveDayOfYear(const AtmosphericDerivedContext& context) {
  if (!context.has_simulation_unix_seconds) {
    return 172;
  }
  return ResolveDayOfYearFromUnixSeconds(context.simulation_unix_seconds);
}

}  // namespace environment
}  // namespace airborne_radar
