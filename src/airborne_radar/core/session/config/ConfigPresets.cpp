#include "1q/airborne_radar/config/ConfigPresets.h"

namespace airborne_radar {
namespace common {
namespace config {

SignalPipelineConfig MakeDetectionMissionSignalPipelineConfig() {
  SignalPipelineConfig config;
  config.beam_control.radar_orientation.scan_start_position = oneq::common::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = RadarWorkSubMode::kTws;
  config.detection.min_detection_margin_db = -100.0f;
  config.lifecycle.enable_auto_lifecycle_manager = true;
  config.lifecycle.lifecycle_config.confirm_hits = 1U;
  config.tracking.kalman_measurement_noise_std = 1.0f;
  return config;
}

SignalPipelineConfig MakeTrackingMissionSignalPipelineConfig() {
  SignalPipelineConfig config;
  config.beam_control.radar_orientation.scan_start_position = oneq::common::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence = oneq::common::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode = RadarWorkSubMode::kTws;
  config.detection.min_detection_margin_db = -20.0f;
  config.lifecycle.enable_auto_lifecycle_manager = true;
  config.lifecycle.lifecycle_config.confirm_hits = 2U;
  config.tracking.kalman_measurement_noise_std = 5.0f;
  return config;
}

SignalPipelineConfig MakeHighRobustnessSignalPipelineConfig() {
  SignalPipelineConfig config = MakeTrackingMissionSignalPipelineConfig();
  config.tracking.kalman_measurement_noise_std = 3.0f;
  config.lifecycle.lifecycle_config.max_miss_before_lost = 3U;
  config.lifecycle.lifecycle_config.max_lost_cycles = 8U;
  return config;
}

core::session::RadarSessionConfig MakeDefaultRadarSessionConfig() {
  core::session::RadarSessionConfig config;
  config.signal_pipeline_config.beam_control.radar_orientation.scan_start_position =
      oneq::common::ScanStartPosition::kLeftTop;
  config.signal_pipeline_config.beam_control.radar_orientation.scan_sequence =
      oneq::common::ScanSequence::kAzimuthFirst;
  config.signal_pipeline_config.beam_control.radar_orientation.work_sub_mode =
      RadarWorkSubMode::kTws;
  return config;
}

core::session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig() {
  core::session::RadarSessionConfig config;
  config.signal_pipeline_config = MakeDetectionMissionSignalPipelineConfig();
  return config;
}

}  // namespace config
}  // namespace common
}  // namespace airborne_radar
