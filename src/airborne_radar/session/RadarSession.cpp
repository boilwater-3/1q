#include "1q/airborne_radar/session/RadarSession.h"

#include <utility>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "airborne_radar/session/RadarSessionCompositionRoot.h"
#include "airborne_radar/session/RuntimeConfigResolver.h"

namespace airborne_radar {
namespace session {

struct RadarSession::Impl {
  explicit Impl(internal::RadarSessionComposition composition)
      : runtime_state(),
        owned_radar_context(std::move(composition.owned_radar_context)),
        owned_signal_pipeline(std::move(composition.owned_signal_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)),
        radar_context(*composition.radar_context),
        signal_pipeline(*composition.signal_pipeline),
        environment_service(*composition.environment_service),
        controller(*composition.controller) {
    runtime_state.pipeline_config = composition.runtime_pipeline_config;
    runtime_state.environment_scenario_config = composition.runtime_environment_scenario_config;
    runtime_state.jamming_sensitivity_profile =
      composition.runtime_jamming_sensitivity_profile;
    pending_runtime_state = runtime_state;
  }

  RadarCycleResult BuildCycleResult() const {
    RadarCycleResult result;
    if (controller.HasLatestTrackOutputFrame()) {
      result.track_output_frame = controller.GetLatestTrackOutputFrame();
    }
    result.executed_this_cycle = controller.ExecutedLatestCycle();
    result.signal_cycle_abort_reason = controller.GetLastSignalCycleAbortReason();
    result.reused_previous_track_output = controller.ReusedPreviousTrackOutputLatestCycle();
    if (result.executed_this_cycle) {
      result.submitted_commands = radar_context.GetSubmittedCommands();
    }
    result.validation_issues = controller.GetLastValidationIssues();
    result.has_validation_error = controller.HasValidationError();
    result.has_control_profile =
        result.executed_this_cycle && radar_context.HasLatestControlProfile();
    if (result.has_control_profile) {
      result.control_profile = radar_context.GetLatestControlProfile();
    }
    if (result.executed_this_cycle) {
      result.association_quality_metrics = signal_pipeline.GetLastAssociationQualityMetrics();
    }
    return result;
  }

  ValidationIssueList ValidateInput(const RadarCycleInput& input) const {
    return ValidateRadarCycleInput(input);
  }

  RadarCycleResult BuildValidationErrorResult(const ValidationIssueList& issues) const {
    RadarCycleResult result;
    if (controller.HasLatestTrackOutputFrame()) {
      result.track_output_frame = controller.GetLatestTrackOutputFrame();
      result.reused_previous_track_output = true;
    }
    result.validation_issues = issues;
    result.has_validation_error = HasValidationError(issues);
    return result;
  }

  RadarCycleResult BuildExecutionAbortResult(
      extension::SignalCycleAbortReason abort_reason) const {
    RadarCycleResult result;
    if (controller.HasLatestTrackOutputFrame()) {
      result.track_output_frame = controller.GetLatestTrackOutputFrame();
      result.reused_previous_track_output = true;
    }
    result.signal_cycle_abort_reason = abort_reason;
    return result;
  }

  void CommitPendingRuntimeConfig() {
    if (!has_pending_runtime_update) {
      return;
    }

    signal_pipeline.UpdateConfig(pending_runtime_state.pipeline_config);
    environment_service.UpdateModelConfig(environment::BuildModelConfigFromScenario(
        pending_runtime_state.environment_scenario_config));
    environment_service.SetJammingSensitivityProfile(
        pending_runtime_state.jamming_sensitivity_profile);
  }

  void FinalizePendingRuntimeConfig() {
    if (!has_pending_runtime_update) {
      return;
    }
    runtime_state = pending_runtime_state;
    has_pending_runtime_update = false;
  }

  void RollbackFailedCycle(
      const extension::RadarContextRuntimeState& radar_context_state,
      const environment::EnvironmentServiceRuntimeState* environment_state,
      const extension::RadarControllerRuntimeState* controller_state) {
    radar_context.RestoreRuntimeState(radar_context_state);
    if (environment_state != nullptr) {
      environment_service.RestoreRuntimeState(*environment_state);
    }
    if (controller_state != nullptr) {
      controller.RestoreRuntimeState(*controller_state);
    }
  }

  internal::RuntimeConfigState runtime_state{};
  internal::RuntimeConfigState pending_runtime_state{};
  bool has_pending_runtime_update{false};
  std::unique_ptr<extension::IRadarContext> owned_radar_context;
  std::unique_ptr<extension::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<environment::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::RadarController> owned_controller;
  extension::IRadarContext& radar_context;
  extension::ISignalPipeline& signal_pipeline;
  environment::IEnvironmentService& environment_service;
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
    const RadarSessionConfig& config, environment::IEnvironmentService& environment_service) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      internal::RadarSessionCompositionRoot::ComposeWithEnvironmentService(config,
                                                                           environment_service))));
}

RadarSession RadarSessionFactory::CreateWithController(const RadarSessionConfig& config,
                                                       extension::RadarController& controller) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      internal::RadarSessionCompositionRoot::ComposeWithController(config, controller))));
}

output::TrackOutputFrame RadarSession::Step(const RadarCycleInput& input) {
  return StepWithResult(input).track_output_frame;
}

output::TrackOutputFrame RadarSession::Step(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  return StepWithResult(input, scene_state).track_output_frame;
}

RadarCycleResult RadarSession::StepWithResult(const RadarCycleInput& input) {
  const ValidationIssueList issues = impl_->ValidateInput(input);
  if (HasValidationError(issues)) {
    return impl_->BuildValidationErrorResult(issues);
  }

  const extension::RadarContextRuntimeState radar_context_state =
      impl_->radar_context.CaptureRuntimeState();
  const bool needs_environment_snapshot = impl_->has_pending_runtime_update;
  const bool needs_controller_snapshot = impl_->has_pending_runtime_update;
  const environment::EnvironmentServiceRuntimeState environment_state =
      needs_environment_snapshot ? impl_->environment_service.CaptureRuntimeState()
                                 : environment::EnvironmentServiceRuntimeState();
  const extension::RadarControllerRuntimeState controller_state =
      needs_controller_snapshot ? impl_->controller.CaptureRuntimeState()
                                : extension::RadarControllerRuntimeState();
  impl_->CommitPendingRuntimeConfig();
  impl_->radar_context.BeginCycle(input);
  impl_->controller.RunOnce();
  if (!impl_->controller.ExecutedLatestCycle()) {
    const extension::SignalCycleAbortReason abort_reason =
        impl_->controller.GetLastSignalCycleAbortReason();
    impl_->RollbackFailedCycle(radar_context_state,
                               needs_environment_snapshot ? &environment_state : nullptr,
                               needs_controller_snapshot ? &controller_state : nullptr);
    return impl_->BuildExecutionAbortResult(abort_reason);
  }
  impl_->FinalizePendingRuntimeConfig();
  return impl_->BuildCycleResult();
}

RadarCycleResult RadarSession::StepWithResult(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  const ValidationIssueList issues = impl_->ValidateInput(input);
  if (HasValidationError(issues)) {
    return impl_->BuildValidationErrorResult(issues);
  }

  const extension::RadarContextRuntimeState radar_context_state =
      impl_->radar_context.CaptureRuntimeState();
  const bool needs_environment_snapshot = true;
  const bool needs_controller_snapshot = impl_->has_pending_runtime_update;
  const environment::EnvironmentServiceRuntimeState environment_state =
      impl_->environment_service.CaptureRuntimeState();
  const extension::RadarControllerRuntimeState controller_state =
      needs_controller_snapshot ? impl_->controller.CaptureRuntimeState()
                                : extension::RadarControllerRuntimeState();
  impl_->CommitPendingRuntimeConfig();
  impl_->environment_service.UpdateSceneState(scene_state);
  impl_->radar_context.BeginCycle(input);
  impl_->controller.RunOnce();
  if (!impl_->controller.ExecutedLatestCycle()) {
    const extension::SignalCycleAbortReason abort_reason =
        impl_->controller.GetLastSignalCycleAbortReason();
    impl_->RollbackFailedCycle(radar_context_state,
                               needs_environment_snapshot ? &environment_state : nullptr,
                               needs_controller_snapshot ? &controller_state : nullptr);
    return impl_->BuildExecutionAbortResult(abort_reason);
  }
  impl_->FinalizePendingRuntimeConfig();
  return impl_->BuildCycleResult();
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

void RadarSession::ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch) {
  const internal::RuntimeConfigState& patch_base_state =
      impl_->has_pending_runtime_update ? impl_->pending_runtime_state : impl_->runtime_state;
  const internal::RuntimeConfigResolveResult resolved =
      internal::ResolveRuntimeConfigPatch(patch_base_state, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return;
  }
  impl_->pending_runtime_state = resolved.next_state;
  impl_->has_pending_runtime_update = true;
}

}  // namespace session
}  // namespace airborne_radar
