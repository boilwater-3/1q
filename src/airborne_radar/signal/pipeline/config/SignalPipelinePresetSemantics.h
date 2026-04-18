/**
 * @file SignalPipelinePresetSemantics.h
 * @brief 定义 SignalPipeline 预设语义的内部单一真值源。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_CONFIG_SIGNAL_PIPELINE_PRESET_SEMANTICS_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_CONFIG_SIGNAL_PIPELINE_PRESET_SEMANTICS_H_

#include "1q/airborne_radar/config/PipelineConfig.h"

namespace airborne_radar {
namespace config {
namespace internal {

inline config::PipelineConfig BuildDetectionMissionPresetConfig() {
  config::PipelineConfig config;
  config.beam_control.radar_orientation.scan_start_position =
      oneq::foundation::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  config.detection.intent_profile = config::semantic::DetectionIntentProfile::kDetectionPriority;
  config.tracking.policy_profile = config::semantic::TrackingPolicyProfile::kFastAssociation;
  config.lifecycle.policy_profile = config::semantic::LifecyclePolicyProfile::kFastConfirm;
  return config;
}

inline config::PipelineConfig BuildTrackingMissionPresetConfig() {
  config::PipelineConfig config;
  config.beam_control.radar_orientation.scan_start_position =
      oneq::foundation::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  config.detection.intent_profile =
      config::semantic::DetectionIntentProfile::kTrackStabilityPriority;
  config.tracking.policy_profile = config::semantic::TrackingPolicyProfile::kBalanced;
  config.lifecycle.policy_profile = config::semantic::LifecyclePolicyProfile::kBalanced;
  return config;
}

inline config::PipelineConfig BuildHighRobustnessPresetConfig() {
  config::PipelineConfig config = BuildTrackingMissionPresetConfig();
  config.tracking.policy_profile = config::semantic::TrackingPolicyProfile::kRobustAntiJamming;
  config.lifecycle.policy_profile = config::semantic::LifecyclePolicyProfile::kHighPersistence;
  return config;
}

}  // namespace internal
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_CONFIG_SIGNAL_PIPELINE_PRESET_SEMANTICS_H_
