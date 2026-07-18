#include "1q/airborne_radar/session/ArSession.h"

#include <utility>

#include "airborne_radar/config/mapping/RuntimePatchMapper.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/runtime/ArController.h"
#include "airborne_radar/session/MutableArContext.h"
#include "airborne_radar/session/ArSessionCompositionRoot.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar {
namespace session {
namespace {

session::EnvironmentSceneState BuildSceneStateFromEnvironmentInput(
    const ArEnvironmentInput& environment_input) {
  session::EnvironmentSceneState scene_state;
  scene_state.atmospheric_physics = environment_input.atmospheric_observation;
  scene_state.atmospheric_context = environment_input.atmospheric_context;
  scene_state.vegetation_scatter_physics = environment_input.surface_observation;
  scene_state.jammer_emitters = environment_input.jammer_sources;
  return scene_state;
}

}  // namespace

struct ArSession::Impl {
  explicit Impl(ArSessionComposition composition)
      : runtime_state(),
        pipeline_config_synced(composition.pipeline_config_synced),
        owned_ar_context(std::move(composition.owned_ar_context)),
        owned_signal_pipeline(std::move(composition.owned_signal_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)) {
    config::ArSessionConfig initial_session_config;
    initial_session_config.hardware = composition.runtime_hardware;
    initial_session_config.mission = composition.runtime_mission;
    initial_session_config.policy = composition.runtime_policy;
    initial_session_config.environment.scenario_config =
        composition.runtime_environment_scenario_config;
    initial_session_config.environment.jamming_sensitivity_profile =
        composition.runtime_jamming_sensitivity_profile;
    runtime_state.execution_config = config::mapping::MapSessionToExecution(initial_session_config);
    runtime_state.environment_scenario_config = composition.runtime_environment_scenario_config;
    runtime_state.jamming_sensitivity_profile = composition.runtime_jamming_sensitivity_profile;
    pending_runtime_state = runtime_state;

    concrete_signal_pipeline_ =
        static_cast<signal::pipeline::SignalPipeline*>(owned_signal_pipeline.get());
  }

  ArCycleResult BuildCycleResult(const ArCycleInput& input) const {
    ArCycleResult result;
    result.input_cycle_index = input.cycle_index;
    if (Controller().HasLatestTrackOutputFrame()) {
      result.track_output_frame = Controller().GetLatestTrackOutputFrame();
    }
    result.executed_this_cycle = Controller().ExecutedLatestCycle();
    result.abort_reason = Controller().GetLastSignalCycleAbortReason();
    result.reused_previous_output = Controller().ReusedPreviousTrackOutputLatestCycle();
    if (result.executed_this_cycle) {
      result.submitted_commands = RadarContext().GetSubmittedCommands();
    }
    result.validation_issues = Controller().GetLastValidationIssues();
    result.has_validation_error = Controller().HasValidationError();
    result.has_control_profile =
        result.executed_this_cycle && RadarContext().HasLatestControlProfile();
    if (result.has_control_profile) {
      result.control_profile = RadarContext().GetLatestControlProfile();
    }
    if (result.executed_this_cycle) {
      result.association_quality_metrics = SignalPipeline().GetLastAssociationQualityMetrics();
      result.has_decision_observation = Controller().HasLatestDecisionObservation();
      if (result.has_decision_observation) {
        result.decision_observation = Controller().GetLatestDecisionObservation();
      }
      result.applied_decision_source = Controller().GetLastAppliedDecisionSource();
      result.applied_decision_cycle_index = Controller().GetLastAppliedDecisionCycleIndex();
      result.applied_decision_batch_id = Controller().GetLastAppliedDecisionBatchId();
    }
    return result;
  }

  ValidationIssueList ValidateInput(const ArCycleInput& input) const {
    return ValidateArCycleInput(input);
  }

  ArCycleResult BuildValidationErrorResult(const ArCycleInput& input,
                                              const ValidationIssueList& issues) const {
    ArCycleResult result;
    result.input_cycle_index = input.cycle_index;
    if (Controller().HasLatestTrackOutputFrame()) {
      result.track_output_frame = Controller().GetLatestTrackOutputFrame();
    }
    result.reused_previous_output = Controller().HasLatestTrackOutputFrame();
    result.abort_reason = session::SignalCycleAbortReason::kValidationRejected;
    result.validation_issues = issues;
    result.has_validation_error = HasValidationError(issues);
    return result;
  }

  ArCycleResult BuildExecutionAbortResult(const ArCycleInput& input,
                                             session::SignalCycleAbortReason abort_reason) const {
    ArCycleResult result;
    result.input_cycle_index = input.cycle_index;
    if (Controller().HasLatestTrackOutputFrame()) {
      result.track_output_frame = Controller().GetLatestTrackOutputFrame();
    }
    result.reused_previous_output = Controller().HasLatestTrackOutputFrame();
    result.abort_reason = abort_reason;
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

      if (concrete_signal_pipeline_ != nullptr) {
        // 内部路径：直接传递 InternalExecutionConfig，避免经过公开类型的 round-trip 信息损失
        config::execution::InternalExecutionConfig exec_config = state_to_commit.execution_config;
        exec_config.detection.orientation.scan_center_deg.az_deg +=
            state_to_commit.dwell_center_deg.az_deg;
        exec_config.detection.orientation.scan_center_deg.el_deg +=
            state_to_commit.dwell_center_deg.el_deg;
        if (!concrete_signal_pipeline_->UpdateExecutionConfig(exec_config)) {
          return false;
        }
      } else {
        // 外部路径：通过公开接口合约传递，由外部 pipeline 自行管理内部配置
        const config::ArSessionConfig pipeline_config =
            config::mapping::MapRuntimeStateToPipelineSession(state_to_commit);
        if (!SignalPipeline().UpdateConfig(pipeline_config)) {
          return false;
        }
      }
      pipeline_config_synced = true;
    }

    if (should_sync_environment_model) {
      EnvironmentService().UpdateModelConfig(
          pending_runtime_state.environment_scenario_config);
    }
    if (should_sync_jamming_sensitivity) {
      EnvironmentService().SetJammingSensitivityProfile(
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

  ArCycleResult RunCycle(const ArCycleInput& input) {
    const ValidationIssueList issues = ValidateInput(input);
    if (HasValidationError(issues)) {
      return BuildValidationErrorResult(input, issues);
    }

    const ArContextRuntimeState radar_context_state = RadarContext().CaptureRuntimeState();
    const signal::SignalPipelineRuntimeState pipeline_state = SignalPipeline().CaptureRuntimeState();
    const environment::EnvironmentServiceRuntimeState environment_state =
        EnvironmentService().CaptureRuntimeState();
    const extension::ArControllerRuntimeState controller_state =
        Controller().CaptureRuntimeState();

    if (!CommitPendingRuntimeConfig()) {
      RadarContext().RestoreRuntimeState(radar_context_state);
      SignalPipeline().RestoreRuntimeState(pipeline_state);
      EnvironmentService().RestoreRuntimeState(environment_state);
      Controller().RestoreRuntimeState(controller_state);
      return BuildExecutionAbortResult(input,
                                       session::SignalCycleAbortReason::kRuntimePreparationFailed);
    }

    if (input.has_environment) {
      EnvironmentService().UpdateSceneState(BuildSceneStateFromEnvironmentInput(input.environment));
    }
    RadarContext().BeginCycle(input);
    Controller().RunOnce();

    if (!Controller().ExecutedLatestCycle()) {
      const session::SignalCycleAbortReason abort_reason =
          Controller().GetLastSignalCycleAbortReason();
      RadarContext().RestoreRuntimeState(radar_context_state);
      SignalPipeline().RestoreRuntimeState(pipeline_state);
      EnvironmentService().RestoreRuntimeState(environment_state);
      Controller().RestoreRuntimeState(controller_state);
      if (abort_reason == session::SignalCycleAbortReason::kSensorPoweredOff) {
        // 关机是已接受的非执行边界，不是 pipeline 故障。先恢复本周期消费的控制/环境状态，
        // 再单独对齐已验证的配置，使 pending 事务能够落定且外部决策仍留待下个成功周期。
        if (!CommitPendingRuntimeConfig()) {
          RadarContext().RestoreRuntimeState(radar_context_state);
          SignalPipeline().RestoreRuntimeState(pipeline_state);
          EnvironmentService().RestoreRuntimeState(environment_state);
          Controller().RestoreRuntimeState(controller_state);
          return BuildExecutionAbortResult(
              input, session::SignalCycleAbortReason::kRuntimePreparationFailed);
        }
        FinalizePendingRuntimeConfig();
      }
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
  std::unique_ptr<MutableArContext> owned_ar_context;
  std::unique_ptr<signal::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<environment::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::ArController> owned_controller;
  signal::pipeline::SignalPipeline* concrete_signal_pipeline_{nullptr};

  MutableArContext& RadarContext() const { return *owned_ar_context; }
  signal::ISignalPipeline& SignalPipeline() const { return *owned_signal_pipeline; }
  environment::IEnvironmentService& EnvironmentService() const {
    return *owned_environment_service;
  }
  extension::ArController& Controller() const { return *owned_controller; }
};

ArSession::ArSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

ArSession::ArSession()
    : impl_(new Impl(ArSessionCompositionRoot::ComposeDefault(config::ArSessionConfig{}))) {}

ArSession::~ArSession() = default;
ArSession::ArSession(ArSession&&) noexcept = default;
ArSession& ArSession::operator=(ArSession&&) noexcept = default;

ArSession ArSession::Create(const config::ArSessionConfig& config) {
  return ArSession(std::unique_ptr<ArSession::Impl>(
      new ArSession::Impl(ArSessionCompositionRoot::ComposeDefault(config))));
}

ArSession ArSession::CreateWithValidation(const config::ArSessionConfig& config,
                                                config::ValidationIssueList* issues) {
  const config::ValidationIssueList found = config::ValidateArSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

session::TrackOutputFrame ArSession::Step(const ArCycleInput& input) {
  return impl_->RunCycle(input).track_output_frame;
}

ArCycleResult ArSession::StepWithResult(const ArCycleInput& input) {
  return impl_->RunCycle(input);
}

const std::vector<session::ArCommand>& ArSession::GetSubmittedCommands() const {
  return impl_->RadarContext().GetSubmittedCommands();
}

bool ArSession::HasLatestControlProfile() const {
  return impl_->RadarContext().HasLatestControlProfile();
}

const session::ArControlProfile& ArSession::GetLatestControlProfile() const {
  return impl_->RadarContext().GetLatestControlProfile();
}

session::AssociationQualityMetrics ArSession::GetLastAssociationQualityMetrics() const {
  return impl_->SignalPipeline().GetLastAssociationQualityMetrics();
}

void ArSession::ApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool ArSession::TryApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch) {
  // 事务性提交类（见 docs/common/contract.md「运行期配置提交策略」）：本方法只写入
  // pending_runtime_state，不触碰 runtime_state；配置延迟到下个 StepWithResult 边界
  // 由 CommitPendingRuntimeConfig 原子提交，失败时 4 子系统 capture/restore 回滚，
  // 执行成功后才由 FinalizePendingRuntimeConfig 落定 pending→runtime。
  const config::mapping::RuntimeConfigState& patch_base_state =
      impl_->has_pending_runtime_update ? impl_->pending_runtime_state : impl_->runtime_state;
  const config::mapping::RuntimeConfigResolveResult resolved =
      config::mapping::ApplyRuntimePatch(patch_base_state, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return false;
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
  return true;
}

session::ExternalDecisionSubmitStatus ArSession::SubmitExternalDecision(
    const session::ExternalDecisionResponse& response) {
  return impl_->Controller().SubmitExternalDecision(response);
}

}  // namespace session
}  // namespace airborne_radar
