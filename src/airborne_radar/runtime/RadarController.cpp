#include "1q/airborne_radar/extension/RadarController.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "airborne_radar/runtime/ControlCommandMapper.h"
#include "airborne_radar/runtime/CycleTelemetryLogger.h"
#include "airborne_radar/runtime/RadarCycleOrchestrator.h"
#include "airborne_radar/decision/pipeline/ControlReducer.h"
#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"
#include "airborne_radar/signal/assembly/DataOutputManager.h"
#include "airborne_radar/signal/assembly/IDataOutputManager.h"
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace extension {

namespace {

/**
 * @brief AirborneRuntimeInput 描述单周期骨架执行需要的输入快照。
 */
struct AirborneRuntimeInput {
  const model::TargetFeatureList* target_features{
      nullptr};                                            /**< 当前周期目标输入只读视图。 */
  model::PlatformAttitudeDeg platform_attitude{}; /**< 当前平台姿态。 */
  float cycle_dt_sec{1.0f};                                /**< 当前周期步长。 */
};

}  // namespace

/**
 * @brief RadarController 内部实现体，持有所有运行时状态。
 */
struct RadarController::Impl {
  extension::IRadarContext& radar_context;
  extension::ISignalPipeline& signal_pipeline;
  extension::ITacticalDecisionEngine* decision_engine{nullptr};
  std::unique_ptr<extension::ITacticalDecisionEngine> owned_decision_engine;
  extension::IEnvironmentService& environment_service;
  std::unique_ptr<extension::control::RadarControlProfile> owned_control_profile;
  std::reference_wrapper<extension::control::RadarControlProfile> control_profile;
  std::unique_ptr<extension::TacticalStateStore> tactical_state_store;
  std::unique_ptr<decision::pipeline::ControlReducer> control_reducer;
  std::unique_ptr<signal::assembly::IDataOutputManager> output_manager;
  std::unique_ptr<extension::ControlCommandMapper> command_mapper;
  std::unique_ptr<extension::RadarCycleOrchestrator> cycle_orchestrator;
  oneq::internal::runtime::RuntimeCycleState<output::TrackOutputFrame,
                                             session::ValidationIssueList>
      runtime_state{};
  std::uint32_t cycle_index{1};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  extension::SignalCycleAbortReason last_signal_abort_reason{
      extension::SignalCycleAbortReason::kNone};

  Impl(extension::IRadarContext& ctx, extension::ISignalPipeline& sig,
       extension::IEnvironmentService& env)
      : radar_context(ctx),
        signal_pipeline(sig),
        owned_decision_engine(new decision::pipeline::TacticalCoordinator()),
        environment_service(env),
        owned_control_profile(new extension::control::RadarControlProfile()),
        control_profile(*owned_control_profile),
        tactical_state_store(new extension::TacticalStateStore()),
        control_reducer(new decision::pipeline::ControlReducer()),
        output_manager(new signal::assembly::DataOutputManager()),
        command_mapper(new extension::ControlCommandMapper(*control_reducer, ctx)) {
    decision_engine = owned_decision_engine.get();
    cycle_orchestrator.reset(new extension::RadarCycleOrchestrator(
        sig, decision_engine, tactical_state_store.get(), env, *output_manager));
  }

  Impl(extension::IRadarContext& ctx, extension::ISignalPipeline& sig,
       extension::ITacticalDecisionEngine& ext_engine,
       extension::IEnvironmentService& env)
      : radar_context(ctx),
        signal_pipeline(sig),
        decision_engine(&ext_engine),
        environment_service(env),
        owned_control_profile(new extension::control::RadarControlProfile()),
        control_profile(*owned_control_profile),
        tactical_state_store(new extension::TacticalStateStore()),
        control_reducer(new decision::pipeline::ControlReducer()),
        output_manager(new signal::assembly::DataOutputManager()),
        command_mapper(new extension::ControlCommandMapper(*control_reducer, ctx)),
        cycle_orchestrator(new extension::RadarCycleOrchestrator(
            sig, &ext_engine, tactical_state_store.get(), env, *output_manager)) {}
};

RadarController::RadarController(extension::IRadarContext& radar_context,
                                 extension::ISignalPipeline& signal_pipeline,
                                 extension::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service)) {}

RadarController::RadarController(extension::IRadarContext& radar_context,
                                 extension::ISignalPipeline& signal_pipeline,
                                 extension::ITacticalDecisionEngine& decision_engine,
                                 extension::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, decision_engine, environment_service)) {}

RadarController::~RadarController() = default;

void RadarController::RunOnce() {
  const extension::EnvironmentServiceRuntimeState environment_state =
      impl_->environment_service.CaptureRuntimeState();
  const output::TrackOutputFrame previous_output = impl_->runtime_state.latest_output;
  const bool had_previous_output = impl_->runtime_state.has_latest_output;
  const std::uint64_t previous_batch_id = impl_->runtime_state.next_batch_id;
  const std::uint32_t previous_cycle_index = impl_->cycle_index;
  const extension::SignalPipelineRuntimeState pipeline_state =
      impl_->signal_pipeline.CaptureRuntimeState();

  impl_->last_cycle_executed = false;
  impl_->last_cycle_reused_previous_output = false;
  impl_->last_signal_abort_reason = extension::SignalCycleAbortReason::kNone;
  AirborneRuntimeInput runtime_input;
  runtime_input.target_features = &impl_->radar_context.GetTargetFeatures();
  runtime_input.platform_attitude = impl_->radar_context.GetPlatformAttitude();
  runtime_input.cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();

  struct AirborneRuntimeHooks {
    Impl* impl{nullptr};

    oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> Validate(
        const AirborneRuntimeInput& input) const {
      oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> result;
      result.issues = session::ValidateRadarCycleDeltaTime(input.cycle_dt_sec);
      if (input.target_features != nullptr) {
        const session::ValidationIssueList target_issues =
            session::ValidateTargetFeatures(*input.target_features);
        result.issues.insert(result.issues.end(), target_issues.begin(), target_issues.end());
      }
      result.has_error = session::HasValidationError(result.issues);
      return result;
    }

    void FreezeEnvironment(const AirborneRuntimeInput& input,
                           const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      if (impl == nullptr) {
        return;
      }
      impl->cycle_orchestrator->FreezeEnvironment(input.cycle_dt_sec, stamp);
    }

    output::TrackOutputFrame Execute(
        const AirborneRuntimeInput& input,
        const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      const CycleExecutionResult exec_result = impl->cycle_orchestrator->Execute(
          input.target_features, input.platform_attitude, impl->control_profile.get(), stamp);
      impl->last_cycle_executed = exec_result.signal_result.executed_this_cycle;
      impl->last_signal_abort_reason = exec_result.signal_result.abort_reason;
      impl->last_cycle_reused_previous_output =
          !impl->last_cycle_executed && impl->runtime_state.has_latest_output;
      if (!impl->last_cycle_executed) {
        return BuildErrorOutput(input, stamp);
      }

      const extension::ControlReductionResult reduction_result =
          impl->command_mapper->Apply(&impl->control_profile.get(),
                                      exec_result.decision_result.proposals);

      const std::size_t input_target_count =
          input.target_features != nullptr ? input.target_features->size() : 0u;
      CycleTelemetryLogger::LogCycleSummary(
          CycleTelemetryPayload(
              stamp, input_target_count,
              exec_result.signal_result.decision_frame.tracks.size(),
              reduction_result.applied_directives.size(),
              exec_result.signal_result.decision_frame.environment_jamming_detected,
              impl->control_profile.get().version,
              exec_result.signal_result.decision_frame.perception_quality_info,
              exec_result.signal_result.association_quality_metrics));
      return exec_result.track_output_frame;
    }

    output::TrackOutputFrame BuildErrorOutput(
        const AirborneRuntimeInput& input,
        const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      (void)input;
      if (impl != nullptr) {
        impl->last_cycle_executed = false;
        impl->last_cycle_reused_previous_output = impl->runtime_state.has_latest_output;
      }
      if (impl != nullptr && impl->runtime_state.has_latest_output) {
        return impl->runtime_state.latest_output;
      }
      output::TrackOutputFrame frame;
      frame.cycle_index = stamp.cycle_index;
      frame.batch_id = stamp.batch_id;
      return frame;
    }
  };

  AirborneRuntimeHooks hooks;
  hooks.impl = impl_.get();
  oneq::internal::runtime::ExecuteRuntimeCycle(runtime_input, impl_->cycle_index,
                                               &impl_->runtime_state, &hooks);
  if (!impl_->last_cycle_executed) {
    impl_->environment_service.RestoreRuntimeState(environment_state);
    impl_->signal_pipeline.RestoreRuntimeState(pipeline_state);
    impl_->runtime_state.latest_output = previous_output;
    impl_->runtime_state.has_latest_output = had_previous_output;
    impl_->runtime_state.next_batch_id = previous_batch_id;
    impl_->cycle_index = previous_cycle_index;
    return;
  }
  ++impl_->cycle_index;
}

void RadarController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce();
  }
}

void RadarController::UpdateControlReducerConfig(
    const extension::ControlReducerConfig& config) {
  if (impl_->control_reducer == nullptr) {
    return;
  }
  impl_->control_reducer->UpdateConfig(config);
  PROJECT_LOG_INFO(
      "[RadarController] control reducer config updated: "
      "lpi_power_scale={} dwell_scale={} burnthrough_gain={} "
      "burnthrough_power_floor={} lpi_hold={} eccm_hold={} "
      "lpi_cooldown={} eccm_cooldown={} prefer_survivability_power={} "
      "prefer_survivability_beam={}",
      config.lpi_power_scale_on_reduction, config.lpi_dwell_scale, config.eccm_burnthrough_gain,
      config.burnthrough_lpi_power_floor, config.lpi_hold_cycles_after_request,
      config.eccm_hold_cycles_after_request, config.lpi_cooldown_cycles_after_release,
      config.eccm_cooldown_cycles_after_release,
      config.prefer_survivability_in_power_conflict ? "true" : "false",
      config.prefer_survivability_in_beam_conflict ? "true" : "false");
}

bool RadarController::HasLatestTrackOutputFrame() const {
  return impl_->runtime_state.has_latest_output;
}

const output::TrackOutputFrame& RadarController::GetLatestTrackOutputFrame() const {
  return impl_->runtime_state.latest_output;
}

const session::ValidationIssueList& RadarController::GetLastValidationIssues() const {
  return impl_->runtime_state.last_validation_issues;
}

bool RadarController::HasValidationError() const {
  return session::HasValidationError(impl_->runtime_state.last_validation_issues);
}

bool RadarController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

bool RadarController::ReusedPreviousTrackOutputLatestCycle() const {
  return impl_->last_cycle_reused_previous_output;
}

extension::SignalCycleAbortReason RadarController::GetLastSignalCycleAbortReason() const {
  return impl_->last_signal_abort_reason;
}

extension::RadarControllerRuntimeState RadarController::CaptureRuntimeState() const {
  extension::RadarControllerRuntimeState state;
  state.latest_output = impl_->runtime_state.latest_output;
  state.has_latest_output = impl_->runtime_state.has_latest_output;
  state.last_validation_issues = impl_->runtime_state.last_validation_issues;
  state.next_batch_id = impl_->runtime_state.next_batch_id;
  state.cycle_index = impl_->cycle_index;
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_cycle_reused_previous_output = impl_->last_cycle_reused_previous_output;
  state.last_signal_abort_reason = impl_->last_signal_abort_reason;
  state.signal_pipeline_state = impl_->signal_pipeline.CaptureRuntimeState();
  return state;
}

void RadarController::RestoreRuntimeState(const extension::RadarControllerRuntimeState& state) {
  impl_->runtime_state.latest_output = state.latest_output;
  impl_->runtime_state.has_latest_output = state.has_latest_output;
  impl_->runtime_state.last_validation_issues = state.last_validation_issues;
  impl_->runtime_state.next_batch_id = state.next_batch_id;
  impl_->cycle_index = state.cycle_index;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_cycle_reused_previous_output = state.last_cycle_reused_previous_output;
  impl_->last_signal_abort_reason = state.last_signal_abort_reason;
  impl_->signal_pipeline.RestoreRuntimeState(state.signal_pipeline_state);
}

extension::IRadarContext& RadarController::GetRadarContext() { return impl_->radar_context; }

extension::ISignalPipeline& RadarController::GetSignalPipeline() {
  return impl_->signal_pipeline;
}

extension::IEnvironmentService& RadarController::GetEnvironmentService() {
  return impl_->environment_service;
}

}  // namespace extension
}  // namespace airborne_radar
