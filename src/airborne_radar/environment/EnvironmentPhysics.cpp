#include <cmath>

#include "1q/airborne_radar/environment/EnvironmentConfig.h"

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

}  // namespace environment
}  // namespace airborne_radar
