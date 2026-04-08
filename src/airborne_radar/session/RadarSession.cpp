#include "1q/airborne_radar/session/RadarSession.h"

#include <utility>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/extension/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "airborne_radar/session/RadarSessionCompositionRoot.h"

namespace airborne_radar {
namespace session {

struct RadarSession::Impl {
  explicit Impl(internal::RadarSessionComposition composition)
      : runtime_signal_pipeline_config(composition.runtime_signal_pipeline_config),
        runtime_environment_model_config(composition.runtime_environment_model_config),
        runtime_jamming_detection_threshold_db(composition.runtime_jamming_detection_threshold_db),
        owned_radar_context(std::move(composition.owned_radar_context)),
        owned_signal_pipeline(std::move(composition.owned_signal_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)),
        radar_context(*composition.radar_context),
        signal_pipeline(*composition.signal_pipeline),
        environment_service(*composition.environment_service),
        controller(*composition.controller) {}

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

  config::SignalPipelineConfig runtime_signal_pipeline_config{};
  environment::EnvironmentModelConfig runtime_environment_model_config{};
  float runtime_jamming_detection_threshold_db{6.0f};
  std::unique_ptr<extension::IRadarContext> owned_radar_context;
  std::unique_ptr<extension::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<extension::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::RadarController> owned_controller;
  extension::IRadarContext& radar_context;
  extension::ISignalPipeline& signal_pipeline;
  extension::IEnvironmentService& environment_service;
  extension::RadarController& controller;
};

RadarSession::RadarSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

RadarSession::~RadarSession() = default;
RadarSession::RadarSession(RadarSession&&) noexcept = default;
RadarSession& RadarSession::operator=(RadarSession&&) noexcept = default;

RadarSession RadarSessionFactory::Create(const RadarSessionConfig& config) {
  return RadarSession(
      std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
          internal::RadarSessionCompositionRoot::ComposeDefault(config))));
}

RadarSession RadarSessionFactory::CreateWithSignalPipeline(
    const RadarSessionConfig& config, extension::ISignalPipeline& signal_pipeline) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      internal::RadarSessionCompositionRoot::ComposeWithSignalPipeline(config, signal_pipeline))));
}

RadarSession RadarSessionFactory::CreateWithEnvironmentService(
    const RadarSessionConfig& config, extension::IEnvironmentService& environment_service) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      internal::RadarSessionCompositionRoot::ComposeWithEnvironmentService(config,
                                                                           environment_service))));
}

RadarSession RadarSessionFactory::CreateWithController(const RadarSessionConfig& config,
                                                       extension::RadarController& controller) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      internal::RadarSessionCompositionRoot::ComposeWithController(config, controller))));
}

RadarSession RadarSessionFactory::CreateWithExternalChain(
    const RadarSessionConfig& config, extension::IRadarContext& radar_context,
    extension::ISignalPipeline& signal_pipeline,
    extension::IEnvironmentService& environment_service,
    extension::RadarController& controller) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      internal::RadarSessionCompositionRoot::ComposeWithExternalChain(
          config, radar_context, signal_pipeline, environment_service, controller))));
}

output::TrackOutputFrame RadarSession::Step(const RadarCycleInput& input) {
  return StepWithResult(input).track_output_frame;
}

output::TrackOutputFrame RadarSession::Step(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  return StepWithResult(input, scene_state).track_output_frame;
}

RadarCycleResult RadarSession::StepWithResult(const RadarCycleInput& input) {
  impl_->radar_context.BeginCycle(input);
  impl_->controller.RunOnce();
  return impl_->BuildCycleResult();
}

RadarCycleResult RadarSession::StepWithResult(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  impl_->environment_service.UpdateSceneState(scene_state);
  return StepWithResult(input);
}

const std::vector<extension::control::RadarCommand>& RadarSession::GetSubmittedCommands() const {
  return impl_->radar_context.GetSubmittedCommands();
}

bool RadarSession::HasLatestControlProfile() const {
  return impl_->radar_context.HasLatestControlProfile();
}

const extension::control::RadarControlProfile& RadarSession::GetLatestControlProfile() const {
  return impl_->radar_context.GetLatestControlProfile();
}

extension::AssociationQualityMetrics RadarSession::GetLastAssociationQualityMetrics() const {
  return impl_->signal_pipeline.GetLastAssociationQualityMetrics();
}

void RadarSession::UpdateSignalPipelineConfig(const config::SignalPipelineConfig& config) {
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

void RadarSession::ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch) {
  bool should_update_signal_pipeline_config = false;
  config::SignalPipelineConfig* signal_config = &impl_->runtime_signal_pipeline_config;
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

  if (patch.has_environment_runtime_config) {
    if (patch.environment_runtime_config.has_model_config) {
      impl_->runtime_environment_model_config = patch.environment_runtime_config.model_config;
      impl_->environment_service.UpdateModelConfig(impl_->runtime_environment_model_config);
    }
    if (patch.environment_runtime_config.has_jamming_detection_threshold_db) {
      impl_->runtime_jamming_detection_threshold_db =
          patch.environment_runtime_config.jamming_detection_threshold_db;
      impl_->environment_service.SetJammingDetectionThresholdDb(
          impl_->runtime_jamming_detection_threshold_db);
    }
  }
}

}  // namespace session
}  // namespace airborne_radar
