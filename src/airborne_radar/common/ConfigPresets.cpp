#include "1q/airborne_radar/common/ConfigPresets.h"

namespace airborne_radar {
namespace common {

signal::pipeline::SignalPipelineConfig MakeDetectionMissionSignalPipelineConfig() {
  signal::pipeline::SignalPipelineConfig config;
  config.detection.min_detection_margin_db = -100.0f;
  config.lifecycle.enable_auto_lifecycle_manager = true;
  config.lifecycle.lifecycle_config.confirm_hits = 1U;
  config.tracking.kalman_measurement_noise_std = 1.0f;
  return config;
}

signal::pipeline::SignalPipelineConfig MakeTrackingMissionSignalPipelineConfig() {
  signal::pipeline::SignalPipelineConfig config;
  config.detection.min_detection_margin_db = -20.0f;
  config.lifecycle.enable_auto_lifecycle_manager = true;
  config.lifecycle.lifecycle_config.confirm_hits = 2U;
  config.tracking.kalman_measurement_noise_std = 5.0f;
  config.tracking.speed_decay_ratio_on_loss = 0.95f;
  config.tracking.rcs_decay_ratio_on_loss = 0.92f;
  return config;
}

signal::pipeline::SignalPipelineConfig MakeHighRobustnessSignalPipelineConfig() {
  signal::pipeline::SignalPipelineConfig config = MakeTrackingMissionSignalPipelineConfig();
  config.association.unassigned_cost = 12.0f;
  config.tracking.kalman_measurement_noise_std = 3.0f;
  config.lifecycle.lifecycle_config.max_miss_before_lost = 3U;
  config.lifecycle.lifecycle_config.max_lost_cycles = 8U;
  return config;
}

core::session::RadarSessionConfig MakeDefaultRadarSessionConfig() {
  core::session::RadarSessionConfig config;
  return config;
}

core::session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig() {
  core::session::RadarSessionConfig config;
  config.signal_pipeline_config = MakeDetectionMissionSignalPipelineConfig();
  return config;
}

}  // namespace common
}  // namespace airborne_radar
