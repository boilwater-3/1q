#include "1q/airborne_radar/extension/RadarController.h"

#include <cstdint>
#include <functional>
#include <memory>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/TacticalCoordinator.h"
#include "airborne_radar/runtime/ControlCommandMapper.h"
#include <algorithm>
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace extension {

namespace {

bool IsCompatibleSignalPipelineRuntimeState(const extension::SignalPipelineRuntimeState& state,
                                            const extension::ISignalPipeline& pipeline) {
  return state.owner_identity == &pipeline && state.schema_version == 1U;
}

}  // namespace

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

  // -- 周期运行时状态
  oneq::internal::runtime::RuntimeCycleState<session::TrackOutputFrame,
                                             session::ValidationIssueList>
      cycle_state{};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  extension::SignalCycleAbortReason last_signal_abort_reason{
      extension::SignalCycleAbortReason::kNone};

  /** @brief 构造使用默认 TacticalCoordinator 的控制器。 */
  Impl(extension::IRadarContext& ctx, extension::ISignalPipeline& sig,
       environment::IEnvironmentService& env)
      : radar_context(ctx),
        signal_pipeline(sig),
        environment_service(env) {
    owned_decision_components.decision_engine.reset(
        new decision::TacticalCoordinator(nullptr));
    owned_decision_components.tactical_state_store.reset(new extension::TacticalStateStore());
    owned_decision_components.control_reducer.reset(new decision::ControlReducer());
    decision_engine = owned_decision_components.decision_engine.get();
    // 显式 upcast 表明 IRadarContext 同时满足两个写入接口
    command_mapper.reset(new extension::ControlCommandMapper(
        *owned_decision_components.control_reducer, static_cast<extension::IRadarCommandBus&>(ctx),
        static_cast<extension::IRadarControlProfileStore&>(ctx)));
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
  }

  /** @brief 重置每周期可变标志位。 */
  void ResetPerCycleFlags() {
    last_cycle_executed = false;
    last_cycle_reused_previous_output = false;
    last_signal_abort_reason = extension::SignalCycleAbortReason::kNone;
  }
};

// -- 构造函数

RadarController::RadarController(extension::IRadarContext& radar_context,
                                 extension::ISignalPipeline& signal_pipeline,
                                 environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service)) {}

RadarController::RadarController(extension::IRadarContext& radar_context,
                                 extension::ISignalPipeline& signal_pipeline,
                                 extension::ITacticalDecisionEngine& decision_engine,
                                 environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, decision_engine, environment_service)) {}

RadarController::~RadarController() = default;

void RadarController::RunOnce() {
  impl_->ResetPerCycleFlags();

  const session::RadarSceneTargetList& scene_targets = impl_->radar_context.GetSceneTargets();
  const model::PlatformAttitudeDeg platform_attitude = impl_->radar_context.GetPlatformAttitude();
  const float platform_altitude_m = impl_->radar_context.GetPlatformAltitudeM();
  const float cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();
  const std::uint32_t cycle_index = impl_->radar_context.GetCycleIndex();

  const oneq::internal::runtime::RuntimeCycleStamp stamp =
      oneq::internal::runtime::MakeRuntimeCycleStamp(cycle_index,
                                                     impl_->cycle_state.next_batch_id);

  // 校验
  session::ValidationIssueList issues = session::ValidateRadarCycleDeltaTime(cycle_dt_sec);
  {
    const session::ValidationIssueList target_issues =
        session::ValidateRadarSceneTargets(scene_targets);
    issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  }
  impl_->cycle_state.last_validation_issues = issues;

  if (session::HasValidationError(issues)) {
    impl_->last_cycle_reused_previous_output = impl_->cycle_state.has_latest_output;
    return;
  }

  // 冻结环境
  environment::EnvironmentCycleContext environment_cycle_context;
  environment_cycle_context.cycle_index = stamp.cycle_index;
  environment_cycle_context.dt_sec = cycle_dt_sec;
  impl_->environment_service.BeginCycle(environment_cycle_context);

  // 执行信号流水线与决策引擎
  const session::RadarSceneTargetList& targets = scene_targets;

  impl_->signal_pipeline.SetControlProfile(impl_->control_profile);
  impl_->signal_pipeline.UpdatePlatformAttitude(platform_attitude);
  impl_->signal_pipeline.UpdatePlatformAltitudeM(platform_altitude_m);

  extension::SignalCycleResult signal_result =
      impl_->signal_pipeline.RunCycle(targets, impl_->environment_service);

  impl_->last_cycle_executed = signal_result.executed_this_cycle;
  impl_->last_signal_abort_reason = signal_result.abort_reason;

  if (!impl_->last_cycle_executed) {
    impl_->last_cycle_reused_previous_output = impl_->cycle_state.has_latest_output;
    return;
  }

  model::DecisionInputFrame decision_frame = signal_result.decision_frame;
  decision_frame.cycle_index = stamp.cycle_index;
  decision_frame.batch_id = stamp.batch_id;

  session::TrackOutputFrame track_output_frame;
  track_output_frame.cycle_index = stamp.cycle_index;
  track_output_frame.batch_id = stamp.batch_id;
  track_output_frame.tracks = decision_frame.tracks;

  extension::TacticalDecisionResult decision_result;
  if (impl_->decision_engine != nullptr &&
      impl_->owned_decision_components.tactical_state_store != nullptr) {
    decision_result = impl_->decision_engine->Evaluate(
        decision_frame, *impl_->owned_decision_components.tactical_state_store);

    // 将目标分类结果回填到轨迹输出帧
    const auto& classifications = decision_result.target_classification_result;
    auto& output_tracks = track_output_frame.tracks;
    const std::size_t count = std::min(classifications.size(), output_tracks.size());
    for (std::size_t i = 0; i < count; ++i) {
      output_tracks[i].target_type = classifications[i].target_type;
      output_tracks[i].target_probability = classifications[i].probability;
    }
  }

  signal_result.decision_frame = decision_frame;

  // 应用控制指令
  const extension::ControlReductionResult reduction_result =
      impl_->command_mapper->Apply(&impl_->control_profile, decision_result.proposals);
  (void)reduction_result;

  impl_->cycle_state.latest_output = track_output_frame;
  impl_->cycle_state.has_latest_output = true;
  impl_->last_cycle_reused_previous_output = false;
  ++impl_->cycle_state.next_batch_id;
}

void RadarController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce();
  }
}

bool RadarController::HasLatestTrackOutputFrame() const {
  return impl_->cycle_state.has_latest_output;
}

const session::TrackOutputFrame& RadarController::GetLatestTrackOutputFrame() const {
  return impl_->cycle_state.latest_output;
}

const session::ValidationIssueList& RadarController::GetLastValidationIssues() const {
  return impl_->cycle_state.last_validation_issues;
}

bool RadarController::HasValidationError() const {
  return session::HasValidationError(impl_->cycle_state.last_validation_issues);
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
  state.owner_identity = this;
  state.schema_version = 1U;
  state.latest_output = impl_->cycle_state.latest_output;
  state.has_latest_output = impl_->cycle_state.has_latest_output;
  state.last_validation_issues = impl_->cycle_state.last_validation_issues;
  state.next_batch_id = impl_->cycle_state.next_batch_id;
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_cycle_reused_previous_output = impl_->last_cycle_reused_previous_output;
  state.last_signal_abort_reason = impl_->last_signal_abort_reason;
  state.signal_pipeline_state = impl_->signal_pipeline.CaptureRuntimeState();
  return state;
}

bool RadarController::RestoreRuntimeState(const extension::RadarControllerRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 1U) {
    PROJECT_LOG_ERROR(
        "[RadarController] controller runtime state restore rejected: "
        "owner/schema mismatch.");
    return false;
  }
  if (!IsCompatibleSignalPipelineRuntimeState(state.signal_pipeline_state,
                                              impl_->signal_pipeline)) {
    PROJECT_LOG_ERROR(
        "[RadarController] signal pipeline runtime state restore rejected because "
        "snapshot owner or schema does not match the bound pipeline instance.");
    return false;
  }
  impl_->cycle_state.latest_output = state.latest_output;
  impl_->cycle_state.has_latest_output = state.has_latest_output;
  impl_->cycle_state.last_validation_issues = state.last_validation_issues;
  impl_->cycle_state.next_batch_id = state.next_batch_id;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_cycle_reused_previous_output = state.last_cycle_reused_previous_output;
  impl_->last_signal_abort_reason = state.last_signal_abort_reason;
  impl_->signal_pipeline.RestoreRuntimeState(state.signal_pipeline_state);
  return true;
}

extension::IRadarContext& RadarController::GetRadarContext() { return impl_->radar_context; }

extension::ISignalPipeline& RadarController::GetSignalPipeline() { return impl_->signal_pipeline; }

environment::IEnvironmentService& RadarController::GetEnvironmentService() {
  return impl_->environment_service;
}

}  // namespace extension
}  // namespace airborne_radar
