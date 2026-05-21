#include "1q/airborne_radar/session/RadarSession.h"

#include <utility>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/IOverrideControlStrategy.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "airborne_radar/config/mapping/RuntimePatchMapper.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/session/RadarSessionCompositionRoot.h"

namespace airborne_radar {
namespace session {
namespace {

environment::EnvironmentSceneState BuildSceneStateFromEnvironmentInput(
    const RadarEnvironmentInput& environment_input) {
  environment::EnvironmentSceneState scene_state;
  scene_state.atmospheric_physics = environment_input.atmospheric_observation;
  scene_state.atmospheric_context = environment_input.atmospheric_context;
  scene_state.vegetation_scatter_physics = environment_input.surface_observation;
  scene_state.jammer_emitters = environment_input.jammer_sources;
  return scene_state;
}

}  // namespace

struct RadarSession::Impl {
  explicit Impl(RadarSessionComposition composition)
      : runtime_state(),
        owned_radar_context(std::move(composition.owned_radar_context)),
        owned_signal_pipeline(std::move(composition.owned_signal_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)),
        radar_context(*composition.radar_context),
        signal_pipeline(*composition.signal_pipeline),
        environment_service(*composition.environment_service),
        controller(*composition.controller),
        pipeline_config_synced(composition.pipeline_config_synced) {
    session::RadarSessionConfig initial_session_config;
    initial_session_config.hardware = composition.runtime_hardware;
    initial_session_config.mission = composition.runtime_mission;
    initial_session_config.policy = composition.runtime_policy;
    runtime_state.execution_config = config::mapping::MapSessionToExecution(initial_session_config);
    runtime_state.environment_scenario_config = composition.runtime_environment_scenario_config;
    runtime_state.jamming_sensitivity_profile = composition.runtime_jamming_sensitivity_profile;
    pending_runtime_state = runtime_state;
  }

  RadarCycleResult BuildCycleResult(const RadarCycleInput& input) const {
    RadarCycleResult result;
    result.input_cycle_index = input.cycle_index;
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

  RadarCycleResult BuildValidationErrorResult(const RadarCycleInput& input,
                                              const ValidationIssueList& issues) const {
    RadarCycleResult result;
    result.input_cycle_index = input.cycle_index;
    if (controller.HasLatestTrackOutputFrame()) {
      result.track_output_frame = controller.GetLatestTrackOutputFrame();
    }
    result.reused_previous_track_output = controller.HasLatestTrackOutputFrame();
    result.validation_issues = issues;
    result.has_validation_error = HasValidationError(issues);
    return result;
  }

  RadarCycleResult BuildExecutionAbortResult(const RadarCycleInput& input,
                                             extension::SignalCycleAbortReason abort_reason) const {
    RadarCycleResult result;
    result.input_cycle_index = input.cycle_index;
    if (controller.HasLatestTrackOutputFrame()) {
      result.track_output_frame = controller.GetLatestTrackOutputFrame();
    }
    result.reused_previous_track_output = controller.HasLatestTrackOutputFrame();
    result.signal_cycle_abort_reason = abort_reason;
    return result;
  }

  /**
   * @brief 将已暂存的运行期配置提交到各子系统。
   *
   * 失败时调用方负责恢复已捕获的子系统快照。
   */
  bool CommitPendingRuntimeConfig() {
    const bool should_sync_pipeline =
        !pipeline_config_synced || (has_pending_runtime_update && pending_execution_config_changed);
    const bool should_sync_environment_model =
        has_pending_runtime_update && pending_environment_scenario_config_changed;
    const bool should_sync_jamming_sensitivity =
        has_pending_runtime_update && pending_jamming_sensitivity_profile_changed;

    if (!should_sync_pipeline && !should_sync_environment_model &&
        !should_sync_jamming_sensitivity) {
      return true;
    }

    if (should_sync_pipeline) {
      const config::mapping::RuntimeConfigState& state_to_commit =
          has_pending_runtime_update ? pending_runtime_state : runtime_state;
      const session::RadarSessionConfig pipeline_config =
          config::mapping::MapRuntimeStateToPipelineSession(state_to_commit);
      if (!signal_pipeline.UpdateConfig(pipeline_config)) {
        return false;
      }
      pipeline_config_synced = true;
    }

    if (should_sync_environment_model) {
      environment_service.UpdateModelConfig(environment::BuildModelConfigFromScenario(
          pending_runtime_state.environment_scenario_config));
    }
    if (should_sync_jamming_sensitivity) {
      environment_service.SetJammingSensitivityProfile(
          pending_runtime_state.jamming_sensitivity_profile);
    }
    return true;
  }

  void FinalizePendingRuntimeConfig() {
    if (!has_pending_runtime_update) {
      return;
    }
    runtime_state = pending_runtime_state;
    has_pending_runtime_update = false;
    pending_execution_config_changed = false;
    pending_environment_scenario_config_changed = false;
    pending_jamming_sensitivity_profile_changed = false;
  }

  RadarCycleResult RunCycle(const RadarCycleInput& input) {
    const ValidationIssueList issues = ValidateInput(input);
    if (HasValidationError(issues)) {
      return BuildValidationErrorResult(input, issues);
    }

    const extension::RadarContextRuntimeState radar_context_state =
        radar_context.CaptureRuntimeState();
    const extension::SignalPipelineRuntimeState pipeline_state =
        signal_pipeline.CaptureRuntimeState();
    const environment::EnvironmentServiceRuntimeState environment_state =
        environment_service.CaptureRuntimeState();
    const extension::RadarControllerRuntimeState controller_state =
        controller.CaptureRuntimeState();

    if (!CommitPendingRuntimeConfig()) {
      radar_context.RestoreRuntimeState(radar_context_state);
      signal_pipeline.RestoreRuntimeState(pipeline_state);
      environment_service.RestoreRuntimeState(environment_state);
      controller.RestoreRuntimeState(controller_state);
      return BuildExecutionAbortResult(
          input, extension::SignalCycleAbortReason::kRuntimePreparationFailed);
    }

    environment_service.UpdateSceneState(BuildSceneStateFromEnvironmentInput(input.environment));
    radar_context.BeginCycle(input);
    controller.RunOnce();

    if (!controller.ExecutedLatestCycle()) {
      const extension::SignalCycleAbortReason abort_reason =
          controller.GetLastSignalCycleAbortReason();
      radar_context.RestoreRuntimeState(radar_context_state);
      signal_pipeline.RestoreRuntimeState(pipeline_state);
      environment_service.RestoreRuntimeState(environment_state);
      controller.RestoreRuntimeState(controller_state);
      return BuildExecutionAbortResult(input, abort_reason);
    }

    FinalizePendingRuntimeConfig();
    return BuildCycleResult(input);
  }

  config::mapping::RuntimeConfigState runtime_state{};
  config::mapping::RuntimeConfigState pending_runtime_state{};
  bool has_pending_runtime_update{false};
  bool pending_execution_config_changed{false};
  bool pending_environment_scenario_config_changed{false};
  bool pending_jamming_sensitivity_profile_changed{false};
  bool pipeline_config_synced{true};
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
  return RadarSession(std::unique_ptr<RadarSession::Impl>(
      new RadarSession::Impl(RadarSessionCompositionRoot::ComposeDefault(config))));
}

RadarSession RadarSessionFactory::CreateWithSignalPipeline(
    const RadarSessionConfig& config, extension::ISignalPipeline& signal_pipeline) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      RadarSessionCompositionRoot::ComposeWithSignalPipeline(config, signal_pipeline))));
}

RadarSession RadarSessionFactory::CreateWithEnvironmentService(
    const RadarSessionConfig& config, environment::IEnvironmentService& environment_service) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(
      new RadarSession::Impl(RadarSessionCompositionRoot::ComposeWithEnvironmentService(
          config, environment_service))));
}

RadarSession RadarSessionFactory::CreateWithController(const RadarSessionConfig& config,
                                                       extension::RadarController& controller) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      RadarSessionCompositionRoot::ComposeWithController(config, controller))));
}

RadarSession RadarSessionFactory::CreateWithOverrideStrategy(
    const RadarSessionConfig& config, extension::IOverrideControlStrategy& override_strategy) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(
      new RadarSession::Impl(RadarSessionCompositionRoot::ComposeWithOverrideStrategy(
          config, override_strategy))));
}

RadarSession RadarSessionFactory::CreateWithAll(
    const RadarSessionConfig& config, extension::IRadarContext& radar_context,
    extension::ISignalPipeline& signal_pipeline,
    environment::IEnvironmentService& environment_service,
    extension::RadarController& controller) {
  return RadarSession(std::unique_ptr<RadarSession::Impl>(new RadarSession::Impl(
      RadarSessionCompositionRoot::ComposeAllExternal(config, radar_context, signal_pipeline,
                                                       environment_service, controller))));
}

session::TrackOutputFrame RadarSession::Step(const RadarCycleInput& input) {
  return impl_->RunCycle(input).track_output_frame;
}

RadarCycleResult RadarSession::StepWithResult(const RadarCycleInput& input) {
  return impl_->RunCycle(input);
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
  const config::mapping::RuntimeConfigState& patch_base_state =
      impl_->has_pending_runtime_update ? impl_->pending_runtime_state : impl_->runtime_state;
  const config::mapping::RuntimeConfigResolveResult resolved =
      config::mapping::ApplyRuntimePatch(patch_base_state, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return;
  }
  impl_->pending_runtime_state = resolved.next_state;
  impl_->has_pending_runtime_update = true;
  impl_->pending_execution_config_changed =
      impl_->pending_execution_config_changed || resolved.execution_config_changed;
  impl_->pending_environment_scenario_config_changed =
      impl_->pending_environment_scenario_config_changed ||
      resolved.environment_scenario_config_changed;
  impl_->pending_jamming_sensitivity_profile_changed =
      impl_->pending_jamming_sensitivity_profile_changed ||
      resolved.jamming_sensitivity_profile_changed;
}

}  // namespace session
}  // namespace airborne_radar
