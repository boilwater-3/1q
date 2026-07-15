#include "airborne_radar/runtime/ArController.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>

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

bool IsLpiDirective(session::ControlDirectiveType type) {
  return type == session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION ||
         type == session::ControlDirectiveType::REQUEST_LPI_BEAMFORMING ||
         type == session::ControlDirectiveType::REQUEST_LPI_DWELL;
}

bool IsEccmDirective(session::ControlDirectiveType type) {
  return type == session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER ||
         type == session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING ||
         type == session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY ||
         type == session::ControlDirectiveType::REQUEST_ECCM_REJITTER ||
         type == session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN;
}

bool HasValidRequestedValue(const session::ControlDirective& directive) {
  if (!directive.has_requested_value || !std::isfinite(directive.requested_value)) {
    return false;
  }
  switch (directive.type) {
    case session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      return directive.requested_value > 0.0f && directive.requested_value <= 1.0f;
    case session::ControlDirectiveType::REQUEST_LPI_DWELL:
      return directive.requested_value >= 0.25f && directive.requested_value <= 1.0f;
    case session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return directive.requested_value > 1.0f && directive.requested_value <= 2.0f;
    default:
      return !directive.has_requested_value;
  }
}

bool IsValidExternalProposal(const session::TacticalProposal& proposal) {
  const session::ControlDirective& directive = proposal.directive;
  if (!IsLpiDirective(directive.type) && !IsEccmDirective(directive.type)) {
    return false;
  }
  if (IsLpiDirective(directive.type) &&
      directive.source != session::ControlDirectiveSource::EMISSION_CONTROL) {
    return false;
  }
  if (IsEccmDirective(directive.type) &&
      directive.source != session::ControlDirectiveSource::SURVIVABILITY) {
    return false;
  }
  const bool requires_value =
      directive.type == session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION ||
      directive.type == session::ControlDirectiveType::REQUEST_LPI_DWELL ||
      directive.type == session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN;
  return requires_value ? HasValidRequestedValue(directive) : !directive.has_requested_value;
}

}  // namespace

/**
 * @brief 控制器内部自有组件，生命周期由 Impl 管理。
 *
 * 统一封装默认决策子系统。
 */
struct OwnedDecisionComponents {
  std::unique_ptr<decision::TacticalCoordinator> decision_engine;
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

  // -- 决策子系统
  OwnedDecisionComponents owned_decision_components;

  // -- 控制状态
  session::ArControlProfile control_profile{};

  // -- 独立生命周期组件
  std::unique_ptr<extension::ControlCommandMapper> command_mapper;

  // -- 周期运行时状态
  oneq::common::runtime::RuntimeCycleState<session::TrackOutputFrame,
                                          session::ValidationIssueList>
      cycle_state{};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  session::SignalCycleAbortReason last_signal_abort_reason{session::SignalCycleAbortReason::kNone};

  bool has_pending_internal_decision{false};
  std::uint32_t pending_internal_cycle_index{0U};
  std::uint64_t pending_internal_batch_id{0U};
  std::vector<session::TacticalProposal> pending_internal_proposals{};
  bool has_pending_external_decision{false};
  session::ExternalDecisionResponse pending_external_decision{};
  bool has_latest_decision_observation{false};
  session::DecisionObservation latest_decision_observation{};
  session::DecisionControlSource last_applied_decision_source{
      session::DecisionControlSource::kNone};
  std::uint32_t last_applied_decision_cycle_index{0U};
  std::uint64_t last_applied_decision_batch_id{0U};

  /** @brief 构造使用默认 TacticalCoordinator 的控制器。 */
  Impl(session::MutableArContext& ctx, signal::ISignalPipeline& sig,
       environment::IEnvironmentService& env)
      : radar_context(ctx), signal_pipeline(sig), environment_service(env) {
    owned_decision_components.decision_engine.reset(new decision::TacticalCoordinator(nullptr));
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
    last_applied_decision_source = session::DecisionControlSource::kNone;
    last_applied_decision_cycle_index = 0U;
    last_applied_decision_batch_id = 0U;
  }

  void ApplyPendingDecisionControl() {
    if (!has_pending_internal_decision) {
      return;
    }
    const std::vector<session::TacticalProposal>& selected_proposals =
        has_pending_external_decision ? pending_external_decision.proposals
                                      : pending_internal_proposals;
    command_mapper->Apply(&control_profile, selected_proposals);
    last_applied_decision_source = has_pending_external_decision
                                       ? session::DecisionControlSource::kExternal
                                       : session::DecisionControlSource::kInternal;
    last_applied_decision_cycle_index = pending_internal_cycle_index;
    last_applied_decision_batch_id = pending_internal_batch_id;
    has_pending_internal_decision = false;
    pending_internal_proposals.clear();
    has_pending_external_decision = false;
    pending_external_decision = session::ExternalDecisionResponse();
  }
};

// -- 构造函数

ArController::ArController(session::MutableArContext& radar_context,
                           signal::ISignalPipeline& signal_pipeline,
                           environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service)) {}

ArController::~ArController() = default;

void ArController::RunOnce() {
  impl_->ResetPerCycleFlags();

  const session::ArSceneTargetList& scene_targets = impl_->radar_context.GetSceneTargets();
  const config::PlatformAttitudeDeg platform_attitude = impl_->radar_context.GetPlatformAttitude();
  const float platform_altitude_m = impl_->radar_context.GetPlatformAltitudeM();
  const float cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();
  const std::uint32_t cycle_index = impl_->radar_context.GetCycleIndex();

  const oneq::common::runtime::RuntimeCycleStamp stamp =
      oneq::common::runtime::MakeRuntimeCycleStamp(cycle_index, impl_->cycle_state.next_batch_id);

  // 校验
  session::ValidationIssueList issues = session::ValidateArCycleDeltaTime(cycle_dt_sec);
  {
    const session::ValidationIssueList target_issues =
        session::ValidateArSceneTargets(scene_targets);
    issues.insert(issues.end(), target_issues.begin(), target_issues.end());
  }
  impl_->cycle_state.last_validation_issues = issues;

  if (session::HasValidationError(issues)) {
    impl_->last_signal_abort_reason = session::SignalCycleAbortReason::kValidationRejected;
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

  impl_->ApplyPendingDecisionControl();
  impl_->radar_context.UpdateRadarControlProfile(impl_->control_profile);
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
  if (impl_->owned_decision_components.decision_engine != nullptr &&
      impl_->owned_decision_components.tactical_state_store != nullptr) {
    decision_result = impl_->owned_decision_components.decision_engine->Evaluate(
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

  impl_->has_pending_internal_decision = true;
  impl_->pending_internal_cycle_index = stamp.cycle_index;
  impl_->pending_internal_batch_id = stamp.batch_id;
  impl_->pending_internal_proposals = decision_result.proposals;
  impl_->has_pending_external_decision = false;
  impl_->pending_external_decision = session::ExternalDecisionResponse();
  impl_->latest_decision_observation.input_frame = decision_frame;
  impl_->latest_decision_observation.active_control_profile = impl_->control_profile;
  impl_->has_latest_decision_observation = true;

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

const session::DecisionObservation& ArController::GetLatestDecisionObservation() const {
  return impl_->latest_decision_observation;
}

bool ArController::HasLatestDecisionObservation() const {
  return impl_->has_latest_decision_observation;
}

session::ExternalDecisionSubmitStatus ArController::SubmitExternalDecision(
    const session::ExternalDecisionResponse& response) {
  if (!impl_->has_pending_internal_decision || !impl_->has_latest_decision_observation) {
    return session::ExternalDecisionSubmitStatus::kNoPendingObservation;
  }
  if (response.source_cycle_index != impl_->pending_internal_cycle_index ||
      response.source_batch_id != impl_->pending_internal_batch_id) {
    return session::ExternalDecisionSubmitStatus::kSourceMismatch;
  }
  if (impl_->has_pending_external_decision) {
    return session::ExternalDecisionSubmitStatus::kAlreadySubmitted;
  }
  std::set<session::ControlDirectiveType> directive_types;
  for (std::size_t i = 0; i < response.proposals.size(); ++i) {
    const session::TacticalProposal& proposal = response.proposals[i];
    if (!IsValidExternalProposal(proposal) ||
        !directive_types.insert(proposal.directive.type).second) {
      return session::ExternalDecisionSubmitStatus::kInvalidProposal;
    }
  }
  impl_->pending_external_decision = response;
  impl_->has_pending_external_decision = true;
  return session::ExternalDecisionSubmitStatus::kAccepted;
}

session::DecisionControlSource ArController::GetLastAppliedDecisionSource() const {
  return impl_->last_applied_decision_source;
}

std::uint32_t ArController::GetLastAppliedDecisionCycleIndex() const {
  return impl_->last_applied_decision_cycle_index;
}

std::uint64_t ArController::GetLastAppliedDecisionBatchId() const {
  return impl_->last_applied_decision_batch_id;
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
  state.control_profile = impl_->control_profile;
  state.control_reducer_state = impl_->owned_decision_components.control_reducer->GetRuntimeState();
  state.has_pending_internal_decision = impl_->has_pending_internal_decision;
  state.pending_internal_cycle_index = impl_->pending_internal_cycle_index;
  state.pending_internal_batch_id = impl_->pending_internal_batch_id;
  state.pending_internal_proposals = impl_->pending_internal_proposals;
  state.has_pending_external_decision = impl_->has_pending_external_decision;
  state.pending_external_decision = impl_->pending_external_decision;
  state.has_latest_decision_observation = impl_->has_latest_decision_observation;
  state.latest_decision_observation = impl_->latest_decision_observation;
  state.last_applied_decision_source = impl_->last_applied_decision_source;
  state.last_applied_decision_cycle_index = impl_->last_applied_decision_cycle_index;
  state.last_applied_decision_batch_id = impl_->last_applied_decision_batch_id;
  return state;
}

bool ArController::RestoreRuntimeState(const extension::ArControllerRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 1U) {
    PROJECT_LOG_ERROR(
        "[ArController] controller runtime state restore rejected: "
        "owner/schema mismatch.");
    return false;
  }
  impl_->cycle_state.latest_output = state.latest_output;
  impl_->cycle_state.has_latest_output = state.has_latest_output;
  impl_->cycle_state.last_validation_issues = state.last_validation_issues;
  impl_->cycle_state.next_batch_id = state.next_batch_id;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_cycle_reused_previous_output = state.last_cycle_reused_previous_output;
  impl_->last_signal_abort_reason = state.last_signal_abort_reason;
  impl_->control_profile = state.control_profile;
  impl_->owned_decision_components.control_reducer->RestoreRuntimeState(
      state.control_reducer_state);
  impl_->has_pending_internal_decision = state.has_pending_internal_decision;
  impl_->pending_internal_cycle_index = state.pending_internal_cycle_index;
  impl_->pending_internal_batch_id = state.pending_internal_batch_id;
  impl_->pending_internal_proposals = state.pending_internal_proposals;
  impl_->has_pending_external_decision = state.has_pending_external_decision;
  impl_->pending_external_decision = state.pending_external_decision;
  impl_->has_latest_decision_observation = state.has_latest_decision_observation;
  impl_->latest_decision_observation = state.latest_decision_observation;
  impl_->last_applied_decision_source = state.last_applied_decision_source;
  impl_->last_applied_decision_cycle_index = state.last_applied_decision_cycle_index;
  impl_->last_applied_decision_batch_id = state.last_applied_decision_batch_id;
  return true;
}

}  // namespace extension
}  // namespace airborne_radar
