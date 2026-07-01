#include "airborne_radar/runtime/ArController.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>

#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/TacticalCoordinator.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/runtime/ControlCommandMapper.h"
#include "airborne_radar/session/MutableArContext.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace extension {

namespace {

bool IsCompatibleSignalPipelineRuntimeState(const signal::SignalPipelineRuntimeState& state,
                                            const signal::ISignalPipeline& pipeline) {
  return state.owner_identity == &pipeline && state.schema_version == 1U;
}

}  // namespace

/**
 * @brief 控制器内部自有组件，生命周期由 Impl 管理。
 *
 * 统一封装需要默认构建的决策子系统；外部注入决策引擎时此结构为空。
 */
struct OwnedDecisionComponents {
  std::unique_ptr<session::ITacticalDecisionEngine> decision_engine;
  std::unique_ptr<session::TacticalStateStore> tactical_state_store;
  std::unique_ptr<decision::ControlReducer> control_reducer;
};

/**
 * @brief ArController 内部实现体，持有所有运行时状态。
 */
struct ArController::Impl {
  // -- 外部引用（生命周期由 Session 管理）
  session::MutableArContext& radar_context;
  signal::ISignalPipeline& signal_pipeline;
  environment::IEnvironmentService& environment_service;

  // -- 决策子系统（默认路径自建；外部注入路径为空）
  OwnedDecisionComponents owned_decision_components;
  session::ITacticalDecisionEngine* decision_engine{nullptr};

  // -- 控制状态
  session::ArControlProfile control_profile{};

  // -- 独立生命周期组件
  std::unique_ptr<extension::ControlCommandMapper> command_mapper;

  // -- 周期运行时状态
  oneq::internal::runtime::RuntimeCycleState<session::TrackOutputFrame,
                                             session::ValidationIssueList>
      cycle_state{};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  session::SignalCycleAbortReason last_signal_abort_reason{session::SignalCycleAbortReason::kNone};

  /** @brief 构造使用默认 TacticalCoordinator 的控制器。 */
  Impl(session::MutableArContext& ctx, signal::ISignalPipeline& sig,
       environment::IEnvironmentService& env)
      : radar_context(ctx), signal_pipeline(sig), environment_service(env) {
    owned_decision_components.decision_engine.reset(new decision::TacticalCoordinator(nullptr));
    owned_decision_components.tactical_state_store.reset(new session::TacticalStateStore());
    owned_decision_components.control_reducer.reset(new decision::ControlReducer());
    decision_engine = owned_decision_components.decision_engine.get();
    command_mapper.reset(
        new extension::ControlCommandMapper(*owned_decision_components.control_reducer, ctx));
  }

  /** @brief 构造使用外部决策引擎的控制器。 */
  Impl(session::MutableArContext& ctx, signal::ISignalPipeline& sig,
       session::ITacticalDecisionEngine& ext_engine, environment::IEnvironmentService& env)
      : radar_context(ctx),
        signal_pipeline(sig),
        environment_service(env),
        decision_engine(&ext_engine) {
    // 外部注入决策引擎时，仍需内部 ControlReducer 处理归并与指令提交
    owned_decision_components.tactical_state_store.reset(new session::TacticalStateStore());
    owned_decision_components.control_reducer.reset(new decision::ControlReducer());
    command_mapper.reset(
        new extension::ControlCommandMapper(*owned_decision_components.control_reducer, ctx));
  }

  /** @brief 重置每周期可变标志位。 */
  void ResetPerCycleFlags() {
    last_cycle_executed = false;
    last_cycle_reused_previous_output = false;
    last_signal_abort_reason = session::SignalCycleAbortReason::kNone;
  }
};

// -- 构造函数

ArController::ArController(session::MutableArContext& radar_context,
                           signal::ISignalPipeline& signal_pipeline,
                           environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service)) {}

ArController::ArController(session::MutableArContext& radar_context,
                           signal::ISignalPipeline& signal_pipeline,
                           session::ITacticalDecisionEngine& decision_engine,
                           environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, decision_engine, environment_service)) {}

ArController::~ArController() = default;

void ArController::RunOnce() {
  impl_->ResetPerCycleFlags();

  const session::ArSceneTargetList& scene_targets = impl_->radar_context.GetSceneTargets();
  const config::PlatformAttitudeDeg platform_attitude = impl_->radar_context.GetPlatformAttitude();
  const float platform_altitude_m = impl_->radar_context.GetPlatformAltitudeM();
  const float cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();
  const std::uint32_t cycle_index = impl_->radar_context.GetCycleIndex();

  const oneq::internal::runtime::RuntimeCycleStamp stamp =
      oneq::internal::runtime::MakeRuntimeCycleStamp(cycle_index, impl_->cycle_state.next_batch_id);

  // 校验
  session::ValidationIssueList issues = session::ValidateArCycleDeltaTime(cycle_dt_sec);
  {
    const session::ValidationIssueList target_issues =
        session::ValidateArSceneTargets(scene_targets);
    issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  }
  impl_->cycle_state.last_validation_issues = issues;

  if (session::HasValidationError(issues)) {
    impl_->last_cycle_reused_previous_output = impl_->cycle_state.has_latest_output;
    return;
  }

  // 冻结环境
  session::EnvironmentCycleContext environment_cycle_context;
  environment_cycle_context.cycle_index = stamp.cycle_index;
  environment_cycle_context.dt_sec = cycle_dt_sec;
  impl_->environment_service.BeginCycle(environment_cycle_context);

  // 执行信号流水线与决策引擎
  const session::ArSceneTargetList& targets = scene_targets;

  impl_->signal_pipeline.SetControlProfile(impl_->control_profile);
  impl_->signal_pipeline.UpdatePlatformAttitude(platform_attitude);
  impl_->signal_pipeline.UpdatePlatformAltitudeM(platform_altitude_m);

  session::SignalCycleResult signal_result =
      impl_->signal_pipeline.RunCycle(targets, impl_->environment_service);

  impl_->last_cycle_executed = signal_result.executed_this_cycle;
  impl_->last_signal_abort_reason = signal_result.abort_reason;

  if (!impl_->last_cycle_executed) {
    impl_->last_cycle_reused_previous_output = impl_->cycle_state.has_latest_output;
    return;
  }

  session::DecisionInputFrame decision_frame = signal_result.decision_frame;
  decision_frame.cycle_index = stamp.cycle_index;
  decision_frame.batch_id = stamp.batch_id;

  session::TrackOutputFrame track_output_frame;
  track_output_frame.cycle_index = stamp.cycle_index;
  track_output_frame.batch_id = stamp.batch_id;
  track_output_frame.tracks = decision_frame.tracks;

  session::TacticalDecisionResult decision_result;
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

void ArController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce();
  }
}

bool ArController::HasLatestTrackOutputFrame() const {
  return impl_->cycle_state.has_latest_output;
}

const session::TrackOutputFrame& ArController::GetLatestTrackOutputFrame() const {
  return impl_->cycle_state.latest_output;
}

const session::ValidationIssueList& ArController::GetLastValidationIssues() const {
  return impl_->cycle_state.last_validation_issues;
}

bool ArController::HasValidationError() const {
  return session::HasValidationError(impl_->cycle_state.last_validation_issues);
}

bool ArController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

bool ArController::ReusedPreviousTrackOutputLatestCycle() const {
  return impl_->last_cycle_reused_previous_output;
}

session::SignalCycleAbortReason ArController::GetLastSignalCycleAbortReason() const {
  return impl_->last_signal_abort_reason;
}

extension::ArControllerRuntimeState ArController::CaptureRuntimeState() const {
  extension::ArControllerRuntimeState state;
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

bool ArController::RestoreRuntimeState(const extension::ArControllerRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 1U) {
    PROJECT_LOG_ERROR(
        "[ArController] controller runtime state restore rejected: "
        "owner/schema mismatch.");
    return false;
  }
  if (!IsCompatibleSignalPipelineRuntimeState(state.signal_pipeline_state,
                                              impl_->signal_pipeline)) {
    PROJECT_LOG_ERROR(
        "[ArController] signal pipeline runtime state restore rejected because "
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

}  // namespace extension
}  // namespace airborne_radar
