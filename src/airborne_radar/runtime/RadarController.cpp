#include "1q/airborne_radar/extension/RadarController.h"

#include <cstdint>
#include <functional>
#include <memory>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/IOverrideControlStrategy.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/TacticalCoordinator.h"
#include "airborne_radar/runtime/ControlCommandMapper.h"
#include "airborne_radar/runtime/RadarCycleOrchestrator.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace extension {

namespace {

bool IsCompatibleSignalPipelineRuntimeState(const extension::SignalPipelineRuntimeState& state,
                                            const extension::ISignalPipeline& pipeline) {
  return state.owner_identity == &pipeline && state.schema_version == 1U;
}

}  // namespace

/**
 * @brief 单周期内部快照，用于执行失败时的原子回滚。
 */
struct CycleSnapshot {
  environment::EnvironmentServiceRuntimeState environment_state{};
  session::TrackOutputFrame previous_output{};
  bool had_previous_output{false};
  std::uint64_t previous_batch_id{0U};
  extension::SignalPipelineRuntimeState pipeline_state{};
};

/**
 * @brief 控制器内部自有组件，生命周期由 Impl 管理。
 *
 * 统一封装需要默认构建的决策子系统；外部注入决策引擎时此结构为空。
 */
struct OwnedDecisionComponents {
  std::unique_ptr<extension::ITacticalDecisionEngine> decision_engine;
  std::unique_ptr<extension::TacticalStateStore> tactical_state_store;
  std::unique_ptr<decision::ControlReducer> control_reducer;
};

namespace {

/**
 * @brief 构建使用默认 TacticalCoordinator 的内部决策组件。
 * @param override_strategy 可选的外部策略覆盖接口，nullptr 时使用内部评估器。
 */
OwnedDecisionComponents BuildDecisionComponents(
    extension::IOverrideControlStrategy* override_strategy) {
  OwnedDecisionComponents components;
  components.decision_engine.reset(new decision::TacticalCoordinator(nullptr, override_strategy));
  components.tactical_state_store.reset(new extension::TacticalStateStore());
  components.control_reducer.reset(new decision::ControlReducer());
  return components;
}

}  // namespace

/**
 * @brief RadarController 内部实现体，持有所有运行时状态。
 */
struct RadarController::Impl {
  // -- 外部引用（生命周期由 Session 管理）
  extension::IRadarContext& radar_context;
  extension::ISignalPipeline& signal_pipeline;
  environment::IEnvironmentService& environment_service;

  // -- 决策子系统（默认路径自建；外部注入路径为空）
  OwnedDecisionComponents owned_decision_components;
  extension::ITacticalDecisionEngine* decision_engine{nullptr};

  // -- 控制状态
  extension::control::RadarControlProfile control_profile{};

  // -- 独立生命周期组件
  std::unique_ptr<extension::ControlCommandMapper> command_mapper;
  std::unique_ptr<extension::RadarCycleOrchestrator> cycle_orchestrator;

  // -- 周期运行时状态
  oneq::internal::runtime::RuntimeCycleState<session::TrackOutputFrame,
                                             session::ValidationIssueList>
      runtime_state{};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  extension::SignalCycleAbortReason last_signal_abort_reason{
      extension::SignalCycleAbortReason::kNone};

  /** @brief 构造使用默认 TacticalCoordinator 的控制器（可选注入 override_strategy）。 */
  Impl(extension::IRadarContext& ctx, extension::ISignalPipeline& sig,
       environment::IEnvironmentService& env,
       extension::IOverrideControlStrategy* override_strategy)
      : radar_context(ctx),
        signal_pipeline(sig),
        environment_service(env),
        owned_decision_components(BuildDecisionComponents(override_strategy)) {
    decision_engine = owned_decision_components.decision_engine.get();
    // 显式 upcast 表明 IRadarContext 同时满足两个写入接口
    command_mapper.reset(new extension::ControlCommandMapper(
        *owned_decision_components.control_reducer, static_cast<extension::IRadarCommandBus&>(ctx),
        static_cast<extension::IRadarControlProfileStore&>(ctx)));
    cycle_orchestrator.reset(new extension::RadarCycleOrchestrator(
        sig, decision_engine, owned_decision_components.tactical_state_store.get(), env));
  }

  /** @brief 构造使用外部决策引擎的控制器。 */
  Impl(extension::IRadarContext& ctx, extension::ISignalPipeline& sig,
       extension::ITacticalDecisionEngine& ext_engine, environment::IEnvironmentService& env)
      : radar_context(ctx),
        signal_pipeline(sig),
        environment_service(env),
        decision_engine(&ext_engine) {
    // 外部注入决策引擎时，仍需内部 ControlReducer 处理归并与指令提交
    owned_decision_components.tactical_state_store.reset(new extension::TacticalStateStore());
    owned_decision_components.control_reducer.reset(new decision::ControlReducer());
    command_mapper.reset(new extension::ControlCommandMapper(
        *owned_decision_components.control_reducer, static_cast<extension::IRadarCommandBus&>(ctx),
        static_cast<extension::IRadarControlProfileStore&>(ctx)));
    cycle_orchestrator.reset(new extension::RadarCycleOrchestrator(
        sig, decision_engine, owned_decision_components.tactical_state_store.get(), env));
  }

  /** @brief 在执行开始前为本周期捕获可回滚快照。 */
  CycleSnapshot CaptureSnapshot() const {
    CycleSnapshot snapshot;
    snapshot.environment_state = environment_service.CaptureRuntimeState();
    snapshot.previous_output = runtime_state.latest_output;
    snapshot.had_previous_output = runtime_state.has_latest_output;
    snapshot.previous_batch_id = runtime_state.next_batch_id;
    snapshot.pipeline_state = signal_pipeline.CaptureRuntimeState();
    return snapshot;
  }

  /** @brief 重置每周期可变标志位。 */
  void ResetPerCycleFlags() {
    last_cycle_executed = false;
    last_cycle_reused_previous_output = false;
    last_signal_abort_reason = extension::SignalCycleAbortReason::kNone;
  }

  /** @brief 将环境与信号流水线回滚至本周期执行前的状态。 */
  void RestoreFromFailedCycle(const CycleSnapshot& snapshot) {
    environment_service.RestoreRuntimeState(snapshot.environment_state);
    signal_pipeline.RestoreRuntimeState(snapshot.pipeline_state);
    runtime_state.latest_output = snapshot.previous_output;
    runtime_state.has_latest_output = snapshot.had_previous_output;
    runtime_state.next_batch_id = snapshot.previous_batch_id;
  }
};

// -- 构造函数

RadarController::RadarController(extension::IRadarContext& radar_context,
                                 extension::ISignalPipeline& signal_pipeline,
                                 environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service, nullptr)) {}

RadarController::RadarController(extension::IRadarContext& radar_context,
                                 extension::ISignalPipeline& signal_pipeline,
                                 environment::IEnvironmentService& environment_service,
                                 extension::IOverrideControlStrategy& override_strategy)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service, &override_strategy)) {}

RadarController::RadarController(extension::IRadarContext& radar_context,
                                 extension::ISignalPipeline& signal_pipeline,
                                 extension::ITacticalDecisionEngine& decision_engine,
                                 environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, decision_engine, environment_service)) {}

RadarController::~RadarController() = default;

void RadarController::RunOnce() {
  const CycleSnapshot snapshot = impl_->CaptureSnapshot();
  impl_->ResetPerCycleFlags();

  const session::RadarSceneTargetList& scene_targets_ref = impl_->radar_context.GetSceneTargets();
  const session::RadarSceneTargetList* scene_targets = &scene_targets_ref;
  const model::PlatformAttitudeDeg platform_attitude = impl_->radar_context.GetPlatformAttitude();
  const float platform_altitude_m = impl_->radar_context.GetPlatformAltitudeM();
  const float cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();
  const std::uint32_t cycle_index = impl_->radar_context.GetCycleIndex();

  const oneq::internal::runtime::RuntimeCycleStamp stamp =
      oneq::internal::runtime::MakeRuntimeCycleStamp(cycle_index,
                                                     impl_->runtime_state.next_batch_id);

  // 校验
  session::ValidationIssueList issues = session::ValidateRadarCycleDeltaTime(cycle_dt_sec);
  if (scene_targets != nullptr) {
    const session::ValidationIssueList target_issues =
        session::ValidateRadarSceneTargets(*scene_targets);
    issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  }
  impl_->runtime_state.last_validation_issues = issues;

  if (session::HasValidationError(issues)) {
    impl_->last_cycle_reused_previous_output = impl_->runtime_state.has_latest_output;
    impl_->RestoreFromFailedCycle(snapshot);
    return;
  }

  // 冻结环境
  impl_->cycle_orchestrator->FreezeEnvironment(cycle_dt_sec, stamp);

  // 执行信号流水线与决策引擎
  const extension::CycleExecutionResult exec_result = impl_->cycle_orchestrator->Execute(
      scene_targets, platform_attitude, platform_altitude_m, impl_->control_profile, stamp);
  impl_->last_cycle_executed = exec_result.signal_result.executed_this_cycle;
  impl_->last_signal_abort_reason = exec_result.signal_result.abort_reason;

  if (!impl_->last_cycle_executed) {
    impl_->last_cycle_reused_previous_output = impl_->runtime_state.has_latest_output;
    impl_->RestoreFromFailedCycle(snapshot);
    return;
  }

  // 应用控制指令
  const extension::ControlReductionResult reduction_result =
      impl_->command_mapper->Apply(&impl_->control_profile, exec_result.decision_result.proposals);
  (void)reduction_result;

  impl_->runtime_state.latest_output = exec_result.track_output_frame;
  impl_->runtime_state.has_latest_output = true;
  impl_->last_cycle_reused_previous_output = false;
  ++impl_->runtime_state.next_batch_id;
}

void RadarController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce();
  }
}

void RadarController::UpdateControlReducerConfig(const extension::ControlReducerConfig& config) {
  if (impl_->owned_decision_components.control_reducer == nullptr) {
    return;
  }
  impl_->owned_decision_components.control_reducer->UpdateConfig(config);
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

const session::TrackOutputFrame& RadarController::GetLatestTrackOutputFrame() const {
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
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_cycle_reused_previous_output = impl_->last_cycle_reused_previous_output;
  state.last_signal_abort_reason = impl_->last_signal_abort_reason;
  state.signal_pipeline_state = impl_->signal_pipeline.CaptureRuntimeState();
  return state;
}

void RadarController::RestoreRuntimeState(const extension::RadarControllerRuntimeState& state) {
  if (!IsCompatibleSignalPipelineRuntimeState(state.signal_pipeline_state,
                                              impl_->signal_pipeline)) {
    PROJECT_LOG_ERROR(
        "[RadarController] signal pipeline runtime state restore rejected because "
        "snapshot owner or schema does not match the bound pipeline instance.");
    return;
  }
  impl_->runtime_state.latest_output = state.latest_output;
  impl_->runtime_state.has_latest_output = state.has_latest_output;
  impl_->runtime_state.last_validation_issues = state.last_validation_issues;
  impl_->runtime_state.next_batch_id = state.next_batch_id;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_cycle_reused_previous_output = state.last_cycle_reused_previous_output;
  impl_->last_signal_abort_reason = state.last_signal_abort_reason;
  impl_->signal_pipeline.RestoreRuntimeState(state.signal_pipeline_state);
}

extension::IRadarContext& RadarController::GetRadarContext() { return impl_->radar_context; }

extension::ISignalPipeline& RadarController::GetSignalPipeline() { return impl_->signal_pipeline; }

environment::IEnvironmentService& RadarController::GetEnvironmentService() {
  return impl_->environment_service;
}

}  // namespace extension
}  // namespace airborne_radar
