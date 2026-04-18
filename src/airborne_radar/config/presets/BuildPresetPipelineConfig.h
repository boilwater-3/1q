/**
 * @file BuildPresetPipelineConfig.h
 * @brief 定义机载雷达 preset 流水线配置的内部构造函数。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_PRESETS_BUILD_PRESET_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_CONFIG_PRESETS_BUILD_PRESET_PIPELINE_CONFIG_H_

#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

namespace airborne_radar {
namespace config {
namespace internal {

inline config::PipelineConfig BuildDetectionMissionPresetConfig() {
  const session::RadarSessionConfig session_config = config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::semantic::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::semantic::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::semantic::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
  return session::BuildLegacyPipelineConfig(session_config);
}

inline config::PipelineConfig BuildTrackingMissionPresetConfig() {
  const session::RadarSessionConfig session_config = config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(
          config::semantic::DetectionIntentProfile::kTrackStabilityPriority)
      .End()
      .Build();
  return session::BuildLegacyPipelineConfig(session_config);
}

inline config::PipelineConfig BuildHighRobustnessPresetConfig() {
  const session::RadarSessionConfig session_config = config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(
          config::semantic::DetectionIntentProfile::kTrackStabilityPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::semantic::TrackingPolicyProfile::kRobustAntiJamming)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::semantic::LifecyclePolicyProfile::kHighPersistence)
      .End()
      .Build();
  return session::BuildLegacyPipelineConfig(session_config);
}

}  // namespace internal
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_PRESETS_BUILD_PRESET_PIPELINE_CONFIG_H_
