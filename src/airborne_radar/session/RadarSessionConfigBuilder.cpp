#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

#include <cmath>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace config {

session::RadarSessionConfig RadarSessionConfigBuilder::Build() const {
  session::RadarSessionConfig built = config_;
  built.pipeline_config_model = PipelineConfigModel::kSemantic;
  if (built.tracking.policy_profile == semantic::TrackingPolicyProfile::kRobustAntiJamming &&
      !built.lifecycle.enable_imm_fusion) {
    PROJECT_LOG_WARN(
        "[RadarSessionConfigBuilder] robust tracking policy is set while IMM fusion is disabled; "
        "consider enabling IMM for stronger anti-jamming stability.");
  }
  return built;
}

}  // namespace config
}  // namespace airborne_radar
