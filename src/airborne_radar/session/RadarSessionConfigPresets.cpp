/**
 * @file RadarSessionConfigPresets.cpp
 * @brief 实现机载雷达公开预设配置工厂。
 */

#include "1q/airborne_radar/config/presets/RadarSessionConfigPresets.h"

#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/presets/PipelineConfigPresets.h"
#include "airborne_radar/session/SessionConfigBridge.h"

namespace airborne_radar {
namespace config {
namespace presets {

PipelineConfig MakeDetectionMissionPipelineConfig() {
  return session::internal::BuildDetectionMissionPresetConfig();
}

PipelineConfig MakeTrackingMissionPipelineConfig() {
  return session::internal::BuildTrackingMissionPresetConfig();
}

PipelineConfig MakeHighRobustnessPipelineConfig() {
  return session::internal::BuildHighRobustnessPresetConfig();
}

session::RadarSessionConfig MakeDefaultRadarSessionConfig() {
  return config::RadarSessionConfigBuilder().Build();
}

session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(semantic::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(semantic::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(semantic::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

}  // namespace presets
}  // namespace config
}  // namespace airborne_radar
