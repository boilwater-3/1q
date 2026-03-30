#include "1q/airborne_radar/config/ConfigPresets.h"
#include "airborne_radar/common/config/SignalPipelinePresetSemantics.h"

namespace airborne_radar {
namespace common {
namespace config {

SignalPipelineConfig MakeDetectionMissionSignalPipelineConfig() {
  return internal::BuildDetectionMissionPresetConfig();
}

SignalPipelineConfig MakeTrackingMissionSignalPipelineConfig() {
  return internal::BuildTrackingMissionPresetConfig();
}

SignalPipelineConfig MakeHighRobustnessSignalPipelineConfig() {
  return internal::BuildHighRobustnessPresetConfig();
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
