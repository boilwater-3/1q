#include <cmath>

#include "1q/airborne_radar/config/RadarEnvironmentConfig.h"

namespace airborne_radar {
namespace config {

JammingSensitivityProfile ResolveJammingSensitivityProfile(float threshold_db) {
  if (threshold_db <= 5.0f) {
    return JammingSensitivityProfile::kStrict;
  }
  if (threshold_db >= 7.0f) {
    return JammingSensitivityProfile::kRelaxed;
  }
  return JammingSensitivityProfile::kBalanced;
}

}  // namespace config
}  // namespace airborne_radar
