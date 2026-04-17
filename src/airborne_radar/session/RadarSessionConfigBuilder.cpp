#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

#include <cmath>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace config {

session::RadarSessionConfig RadarSessionConfigBuilder::Build() const {
  if (config_.tracking.policy_profile == TrackingPolicyProfile::kRobustAntiJamming &&
      !config_.lifecycle.enable_imm_fusion) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] robust tracking policy is set while IMM fusion is disabled; "
        "consider enabling IMM for stronger anti-jamming stability.");
  }
  return config_;
}

}  // namespace config
}  // namespace airborne_radar
