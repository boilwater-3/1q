#include "1q/airborne_radar/core/session/RadarSessionConfigPresets.h"

#include "airborne_radar/common/config/SignalPipelinePresetSemantics.h"

namespace airborne_radar {
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
  config.beam_control.radar_orientation.scan_start_position =
      oneq::common::ScanStartPosition::kLeftTop;
  config.beam_control.radar_orientation.scan_sequence =
      oneq::common::ScanSequence::kAzimuthFirst;
  config.beam_control.radar_orientation.work_sub_mode =
      common::model::RadarWorkSubMode::kTws;
  return config;
}

core::session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig() {
  core::session::RadarSessionConfig config;
  const SignalPipelineConfig pipeline_config = MakeDetectionMissionSignalPipelineConfig();
  config.detection = pipeline_config.detection;
  config.beam_control = pipeline_config.beam_control;
  config.tracking = pipeline_config.tracking;
  config.lifecycle = pipeline_config.lifecycle;
  return config;
}

}  // namespace config
}  // namespace airborne_radar
