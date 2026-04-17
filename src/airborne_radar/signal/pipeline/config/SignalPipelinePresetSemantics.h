/**
 * @file SignalPipelinePresetSemantics.h
 * @brief 定义 SignalPipeline 预设语义的内部单一真值源。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_CONFIG_SIGNAL_PIPELINE_PRESET_SEMANTICS_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_CONFIG_SIGNAL_PIPELINE_PRESET_SEMANTICS_H_

#include "1q/airborne_radar/config/SignalPipelineConfig.h"

namespace airborne_radar {
namespace config {
namespace internal {

inline config::SignalPipelineConfig BuildDetectionMissionPresetConfig() {
  config::SignalPipelineConfig config;
  config.beam_control.radar_orientation.scan_start_position =
      oneq::foundation::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  config.detection.intent_profile = config::DetectionIntentProfile::kDetectionPriority;
  config.tracking.policy_profile = config::TrackingPolicyProfile::kFastAssociation;
  config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kFastConfirm;
  return config;
}

inline config::SignalPipelineConfig BuildTrackingMissionPresetConfig() {
  config::SignalPipelineConfig config;
  config.beam_control.radar_orientation.scan_start_position =
      oneq::foundation::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::foundation::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  config.detection.intent_profile = config::DetectionIntentProfile::kTrackStabilityPriority;
  config.tracking.policy_profile = config::TrackingPolicyProfile::kBalanced;
  config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kBalanced;
  return config;
}

inline config::SignalPipelineConfig BuildHighRobustnessPresetConfig() {
  config::SignalPipelineConfig config = BuildTrackingMissionPresetConfig();
  config.tracking.policy_profile = config::TrackingPolicyProfile::kRobustAntiJamming;
  config.lifecycle.policy_profile = config::LifecyclePolicyProfile::kHighPersistence;
  return config;
}

}  // namespace internal
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_CONFIG_SIGNAL_PIPELINE_PRESET_SEMANTICS_H_
