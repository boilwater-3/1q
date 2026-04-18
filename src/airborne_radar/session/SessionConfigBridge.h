#ifndef AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_
#define AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_

#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

namespace airborne_radar {
namespace session {
namespace internal {

inline config::PipelineConfig BuildPipelineConfigFromSessionConfig(
    const RadarSessionConfig& config) {
  config::PipelineConfig pipeline_config;
  pipeline_config.expert.detection = config.hardware.detection;
  pipeline_config.expert.beam_control = config.policy.beam_control;
  pipeline_config.expert.association = config.policy.association;
  pipeline_config.expert.tracking = config.policy.tracking;
  pipeline_config.expert.lifecycle = config.policy.lifecycle;
  pipeline_config.expert.imm = config.policy.imm;
  pipeline_config.orientation = config.mission.orientation;
  return pipeline_config;
}

inline config::PipelineConfig BuildDetectionMissionPresetConfig() {
  const session::RadarSessionConfig session_config =
      config::RadarSessionConfigBuilder()
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
  return BuildPipelineConfigFromSessionConfig(session_config);
}

inline config::PipelineConfig BuildTrackingMissionPresetConfig() {
  const session::RadarSessionConfig session_config =
      config::RadarSessionConfigBuilder()
          .Detection()
          .WithDetectionIntentProfile(
              config::semantic::DetectionIntentProfile::kTrackStabilityPriority)
          .End()
          .Build();
  return BuildPipelineConfigFromSessionConfig(session_config);
}

inline config::PipelineConfig BuildHighRobustnessPresetConfig() {
  const session::RadarSessionConfig session_config =
      config::RadarSessionConfigBuilder()
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
  return BuildPipelineConfigFromSessionConfig(session_config);
}

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_
