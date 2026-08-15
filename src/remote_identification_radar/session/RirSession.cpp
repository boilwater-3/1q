/**
 * @file RirSession.cpp
 * @brief 远程识别雷达会话门面实现。
 *
 * 周期语义对齐 AR `ArSession`（审计基线 96de367c）：非执行周期不复用上一帧、
 * 校验拒绝返回 `kRejectedInvalidInput` + 明细、关机 `kPoweredOff` 只推进时间、
 * 运行期补丁在下一次成功周期边界提交。
 */

#include "1q/remote_identification_radar/session/RirSession.h"

#include <utility>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "remote_identification_radar/runtime/RirController.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"

namespace remote_identification_radar {
namespace session {

struct RirSession::Impl {
  config::RirSessionConfig config{};
  runtime::RirController controller{};
  config::RirRuntimeConfigPatch pending_patch{};
  bool has_pending_patch{false};

  explicit Impl(const config::RirSessionConfig& session_config) : config(session_config) {
    controller.SetHardware(config.hardware);
    controller.UpdateRuntime(config.mission.work_mode, config.policy);
  }
};

RirSession::RirSession() : impl_(new Impl(config::RirSessionConfig{})) {}
RirSession::RirSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
RirSession::~RirSession() = default;
RirSession::RirSession(RirSession&&) noexcept = default;
RirSession& RirSession::operator=(RirSession&&) noexcept = default;

RirOutputFrame RirSession::Step(const RirCycleInput& input) {
  return StepWithResult(input).output_frame;
}

RirCycleResult RirSession::StepWithResult(const RirCycleInput& input) {
  RirCycleResult result;
  result.input_cycle_index = input.input_cycle_index;
  result.output_frame.input_cycle_index = input.input_cycle_index;
  result.output_frame.batch_id = input.batch_id;

  // 关机：非执行周期，只记录状态，不推进识别状态（tracker 状态不被触碰）。
  if (!impl_->config.sensor_enabled) {
    result.status = RirCycleStatus::kPoweredOff;
    result.abort_reason = RirCycleAbortReason::kPoweredOff;
    return result;
  }

  // 校验拒绝：不执行流水线，问题明细入 issues。
  const RirIssueList validation_issues = ValidateRirCycleInput(input);
  if (HasValidationError(validation_issues)) {
    result.status = RirCycleStatus::kRejectedInvalidInput;
    result.abort_reason = RirCycleAbortReason::kValidationRejected;
    result.issues = validation_issues;
    return result;
  }

  // 补丁提交（下一个成功周期边界）：电源/工作模式/识别策略整域。
  if (impl_->has_pending_patch) {
    const config::RirRuntimeConfigPatch& patch = impl_->pending_patch;
    if (patch.has_work_mode) {
      impl_->config.mission.work_mode = patch.work_mode;
    }
    if (patch.has_policy) {
      impl_->config.policy = patch.policy;
    }
    if (patch.has_sensor_enabled) {
      impl_->config.sensor_enabled = patch.sensor_enabled;
    }
    impl_->controller.UpdateRuntime(impl_->config.mission.work_mode, impl_->config.policy);
    impl_->has_pending_patch = false;
  }

  impl_->controller.RunCycle(input, &result.output_frame);
  result.status = RirCycleStatus::kCompleted;
  result.abort_reason = RirCycleAbortReason::kNone;
  if (impl_->controller.HasLatestSummary()) {
    result.has_recognition_summary = true;
    result.recognition_summary = impl_->controller.GetLatestSummary();
  }
  return result;
}

bool RirSession::TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch) {
  impl_->pending_patch = patch;
  impl_->has_pending_patch = true;
  return true;
}

bool RirSession::HasLatestRecognitionSummary() const {
  return impl_->controller.HasLatestSummary();
}

const RirRecognitionCycleSummary& RirSession::GetLatestRecognitionSummary() const {
  return impl_->controller.GetLatestSummary();
}

RirSession RirSession::Create(const config::RirSessionConfig& config) {
  return RirSession(std::unique_ptr<Impl>(new Impl(config)));
}

RirSession RirSession::CreateWithDiagnostics(const config::RirSessionConfig& config,
                                             RirIssueList* issues) {
  if (issues != nullptr) {
    *issues = config::ValidateRirSessionConfig(config);
  }
  return RirSession(std::unique_ptr<Impl>(new Impl(config)));
}

RirSessionReplayState RirSessionReplayAccess::CaptureSessionState(const RirSession& session) {
  RirSessionReplayState replay_state;
  replay_state.active_database_version = session.impl_->controller.ActiveDatabaseVersion();
  replay_state.detection_random_seed = session.impl_->controller.DetectionRandomSeed();
  return replay_state;
}

}  // namespace session
}  // namespace remote_identification_radar
