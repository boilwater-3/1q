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
      oneq::common::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  config.detection.min_detection_margin_db = -100.0f;
  config.lifecycle.lifecycle_config.confirm_hits = 1U;
  config.tracking.kalman_measurement_noise_std = 1.0f;
  return config;
}

inline config::SignalPipelineConfig BuildTrackingMissionPresetConfig() {
  config::SignalPipelineConfig config;
  config.beam_control.radar_orientation.scan_start_position =
      oneq::common::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTws;
  config.detection.min_detection_margin_db = -20.0f;
  config.lifecycle.lifecycle_config.confirm_hits = 2U;
  config.tracking.kalman_measurement_noise_std = 5.0f;
  return config;
}

inline config::SignalPipelineConfig BuildHighRobustnessPresetConfig() {
  config::SignalPipelineConfig config = BuildTrackingMissionPresetConfig();
  config.tracking.kalman_measurement_noise_std = 3.0f;
  config.lifecycle.lifecycle_config.max_miss_before_lost = 3U;
  config.lifecycle.lifecycle_config.max_lost_cycles = 8U;
  return config;
}

}  // namespace internal
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_CONFIG_SIGNAL_PIPELINE_PRESET_SEMANTICS_H_
