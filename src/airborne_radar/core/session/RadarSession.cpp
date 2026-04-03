#include "1q/airborne_radar/core/session/RadarSession.h"

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "airborne_radar/core/context/MutableRadarContext.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar {
namespace core {
namespace session {

struct RadarSession::Impl {
  explicit Impl(const RadarSessionConfig& config)
      : runtime_signal_pipeline_config(config.signal_pipeline_config),
        runtime_environment_model_config(config.environment_model_config),
        runtime_jamming_detection_threshold_db(config.jamming_detection_threshold_db),
        signal_pipeline(runtime_signal_pipeline_config),
        environment_service(runtime_environment_model_config),
        controller(radar_context, signal_pipeline, environment_service) {
    environment_service.SetJammingDetectionThresholdDb(runtime_jamming_detection_threshold_db);
  }
  RadarCycleResult BuildCycleResult() const {
    RadarCycleResult result;
    if (controller.HasLatestTrackOutputFrame()) {
      result.track_output_frame = controller.GetLatestTrackOutputFrame();
    }
    result.submitted_commands = radar_context.GetSubmittedCommands();
    result.validation_issues = controller.GetLastValidationIssues();
    result.has_validation_error = controller.HasValidationError();
    result.has_control_profile = radar_context.HasLatestControlProfile();
    if (result.has_control_profile) {
      result.control_profile = radar_context.GetLatestControlProfile();
    }
    result.association_quality_metrics = signal_pipeline.GetLastAssociationQualityMetrics();
    return result;
  }

  context::MutableRadarContext radar_context{};
  common::config::SignalPipelineConfig runtime_signal_pipeline_config{};
  environment::EnvironmentModelConfig runtime_environment_model_config{};
  float runtime_jamming_detection_threshold_db{6.0f};
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService environment_service;
  controller::RadarController controller;
};

RadarSession::RadarSession(const RadarSessionConfig& config) : impl_(new Impl(config)) {}

RadarSession::~RadarSession() = default;

common::output::TrackOutputFrame RadarSession::Step(const context::RadarCycleInput& input) {
  return StepWithResult(input).track_output_frame;
}

common::output::TrackOutputFrame RadarSession::Step(
    const context::RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  return StepWithResult(input, scene_state).track_output_frame;
}

RadarCycleResult RadarSession::StepWithResult(const context::RadarCycleInput& input) {
  impl_->radar_context.BeginCycle(input);
  impl_->controller.RunOnce();
  return impl_->BuildCycleResult();
}

RadarCycleResult RadarSession::StepWithResult(
    const context::RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  impl_->environment_service.UpdateSceneState(scene_state);
  return StepWithResult(input);
}

const std::vector<common::control::RadarCommand>& RadarSession::GetSubmittedCommands() const {
  return impl_->radar_context.GetSubmittedCommands();
}

bool RadarSession::HasLatestControlProfile() const {
  return impl_->radar_context.HasLatestControlProfile();
}

const common::control::RadarControlProfile& RadarSession::GetLatestControlProfile() const {
  return impl_->radar_context.GetLatestControlProfile();
}

signal::pipeline::AssociationQualityMetrics RadarSession::GetLastAssociationQualityMetrics() const {
  return impl_->signal_pipeline.GetLastAssociationQualityMetrics();
}

void RadarSession::UpdateSignalPipelineConfig(const common::config::SignalPipelineConfig& config) {
  impl_->runtime_signal_pipeline_config = config;
  impl_->signal_pipeline.UpdateConfig(impl_->runtime_signal_pipeline_config);
}

void RadarSession::UpdateEnvironmentModelConfig(const environment::EnvironmentModelConfig& config) {
  impl_->runtime_environment_model_config = config;
  impl_->environment_service.UpdateModelConfig(impl_->runtime_environment_model_config);
}

void RadarSession::SetJammingDetectionThresholdDb(float threshold_db) {
  impl_->runtime_jamming_detection_threshold_db = threshold_db;
  impl_->environment_service.SetJammingDetectionThresholdDb(
      impl_->runtime_jamming_detection_threshold_db);
}

void RadarSession::ApplyRuntimeConfig(const common::config::RadarRuntimeConfigPatch& patch) {
  bool should_update_signal_pipeline_config = false;
  if (patch.has_signal_pipeline_config) {
    impl_->runtime_signal_pipeline_config = patch.signal_pipeline_config;
    should_update_signal_pipeline_config = true;
  }

  common::config::SignalPipelineConfig* signal_config = &impl_->runtime_signal_pipeline_config;
  if (patch.has_signal_detection_config) {
    signal_config->detection = patch.signal_detection_config;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_rcs_enable_physical_override) {
    signal_config->detection.rcs_physics.enable_physical_rcs = patch.rcs_enable_physical_override;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_rcs_physics_frequency_hz) {
    signal_config->detection.rcs_physics.frequency_hz = patch.rcs_physics_frequency_hz;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_rcs_physics_mix_ratio) {
    signal_config->detection.rcs_physics.physics_mix_ratio = patch.rcs_physics_mix_ratio;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_rcs_physics_cylinder_weight) {
    signal_config->detection.rcs_physics.cylinder_weight = patch.rcs_physics_cylinder_weight;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_rcs_equivalent_radius_range) {
    signal_config->detection.rcs_physics.min_equivalent_radius_m =
        patch.rcs_min_equivalent_radius_m;
    signal_config->detection.rcs_physics.max_equivalent_radius_m =
        patch.rcs_max_equivalent_radius_m;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_rcs_output_range_m2) {
    signal_config->detection.rcs_physics.min_rcs_m2 = patch.rcs_min_rcs_m2;
    signal_config->detection.rcs_physics.max_rcs_m2 = patch.rcs_max_rcs_m2;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_rcs_bistatic_psi_offset_deg) {
    signal_config->detection.rcs_physics.bistatic_psi_offset_deg =
        patch.rcs_bistatic_psi_offset_deg;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_signal_beam_control_config) {
    signal_config->beam_control = patch.signal_beam_control_config;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_platform_attitude_deg) {
    signal_config->beam_control.platform_attitude_deg = patch.platform_attitude_deg;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_work_sub_mode) {
    signal_config->beam_control.radar_orientation.work_sub_mode = patch.work_sub_mode;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_scan_center_deg) {
    signal_config->beam_control.radar_orientation.scan_center_deg = patch.scan_center_deg;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_dwell_center_deg) {
    signal_config->beam_control.radar_orientation.dwell_center_deg = patch.dwell_center_deg;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_commanded_beamwidth_deg) {
    signal_config->beam_control.radar_orientation.commanded_beamwidth_deg =
        patch.commanded_beamwidth_deg;
    should_update_signal_pipeline_config = true;
  }
  if (patch.has_commanded_beamwidth_enabled) {
    signal_config->beam_control.radar_orientation.commanded_beamwidth_enabled =
        patch.commanded_beamwidth_enabled;
    should_update_signal_pipeline_config = true;
  }
  if (should_update_signal_pipeline_config) {
    impl_->signal_pipeline.UpdateConfig(impl_->runtime_signal_pipeline_config);
  }

  if (patch.has_environment_model_config) {
    impl_->runtime_environment_model_config = patch.environment_model_config;
    impl_->environment_service.UpdateModelConfig(impl_->runtime_environment_model_config);
  }

  if (patch.has_jamming_detection_threshold_db) {
    impl_->runtime_jamming_detection_threshold_db = patch.jamming_detection_threshold_db;
    impl_->environment_service.SetJammingDetectionThresholdDb(
        impl_->runtime_jamming_detection_threshold_db);
  }
}

}  // namespace session
}  // namespace core
}  // namespace airborne_radar
