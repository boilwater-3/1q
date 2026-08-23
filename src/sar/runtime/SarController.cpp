#include "sar/runtime/SarController.h"

#include <utility>

#include "1q/sar/session/SarInputValidation.h"
#include "1q/sar/session/SarIssueCodes.h"
#include "common/logging/ProjectLog.h"
#include "sar/session/SarDiagnosticUtils.h"
#include "sar/session/SarFocusedImageAssembler.h"
#include "sar/session/SarRawHistoryBuilder.h"
#include "sar/session/SarRuntimeConfigResolver.h"
#include "sar/session/SarRuntimeConfigValidation.h"

namespace sar {
namespace extension {

namespace {

constexpr std::uint32_t kControllerRuntimeStateSchemaVersion = 1U;

void ApplyDiagnosticsPolicy(const config::SarPolicyConfig& policy,
                            session::SarCycleResult* result) {
  if (policy.enable_diagnostics || result == nullptr) {
    return;
  }
  session::SarIssueList errors;
  for (const session::SarIssue& issue : result->issues) {
    if (issue.severity == session::SarIssueSeverity::kError) {
      errors.push_back(issue);
    }
  }
  result->issues.swap(errors);
}

}  // namespace

struct SarController::Impl {
  Impl(pipeline::SarProcessingPipeline& pipeline_ref,
       const config::SarSessionConfig& initial_config)
      : pipeline(pipeline_ref), runtime_config(initial_config) {}

  void Finish(session::SarCycleResult result) {
    ApplyDiagnosticsPolicy(runtime_config.policy, &result);
    latest_result = result;
  }

  pipeline::SarProcessingPipeline& pipeline;
  config::SarSessionConfig runtime_config;
  session::SarCycleResult latest_result{};
};

SarController::SarController(pipeline::SarProcessingPipeline& pipeline,
                             const config::SarSessionConfig& initial_config)
    : impl_(new Impl(pipeline, initial_config)) {}

SarController::~SarController() = default;

void SarController::RunOnce(const session::SarCycleInput& input) {
  session::SarCycleResult result;
  result.input_cycle_index = input.cycle_index;

  const session::SarIssueList input_issues = session::ValidateSarCycleInput(input);
  if (session::HasValidationError(input_issues)) {
    // 校验拒绝（规则 9/14）：校验问题本身就是 error 级诊断（写二），直接进入统一
    // 问题列表；abort_reason（写一）与日志（写三）在此补齐，不调用 RecordAbort。
    result.issues = input_issues;
    result.abort_reason = session::SarPipelineAbortReason::kValidationRejected;
    result.status = session::SarCycleStatus::kRejectedInvalidInput;
    // 中译：SAR 输入校验拒绝（周期号）。
    // 标识：三写之三（人读日志）——调用方输入非法，本周期未执行；
    //       仅用于人读，不用于状态判断（规则 3）。
    PROJECT_LOG_WARN("SAR validation rejected for cycle_index={}", input.cycle_index);
    // 非执行周期：output_frame 保持默认空帧，不复用上一有效输出。
    impl_->Finish(result);
    return;
  }

  const bool has_external_raw_iq = session::HasExternalRawIq(input);
  if (!session::ValidateRuntimeConfigForStep(impl_->runtime_config, has_external_raw_iq, &result)) {
    // 非执行周期：output_frame 保持默认空帧，不复用上一有效输出。
    impl_->Finish(result);
    return;
  }

  // 电源短路（COMMON-OQ-4 字段提升）：关机周期不写输出帧元数据（严格默认
  // 空帧，与校验失败路径同形），跨周期状态不推进。关机是合法非执行状态——
  // 不是校验错误也不是执行失败。三写手动补齐：RecordAbort 强制
  // status=kRejectedExecution，与关机语义冲突，不走统一入口。
  if (!impl_->runtime_config.sensor_enabled) {
    result.abort_reason = session::SarPipelineAbortReason::kSensorPoweredOff;
    result.status = session::SarCycleStatus::kPoweredOff;
    session::SarIssue issue;
    issue.severity = session::SarIssueSeverity::kError;
    issue.phase = session::SarIssuePhase::kExecution;
    issue.code = session::codes::kSensorPoweredOff;
    issue.message = "SAR cycle skipped: sensor disabled.";
    result.issues.push_back(issue);
    // 中译：传感器已关闭，本周期短路（周期号）。
    // 标识：电源关闭状态——周期不执行、输出为空帧；属预期行为而非执行
    //       失败，但按规则 9c 中止路径日志级别以 WARN 记录。
    PROJECT_LOG_WARN("[SarController] sensor disabled, cycle_index={} skipped.",
                     input.cycle_index);
    impl_->Finish(result);
    return;
  }

  // 所有前置校验通过后才写入输出帧元数据，确保失败周期输出严格为默认空帧。
  result.product.output_frame.cycle_index = input.cycle_index;
  session::InitializeOutputFrameMetadata(impl_->runtime_config, &result.product.output_frame);

  const pipeline::SarProcessingPipelineRuntimeState pipeline_state =
      impl_->pipeline.CaptureRuntimeState();
  if (!impl_->pipeline.RunCycle(impl_->runtime_config, input, &result)) {
    if (!impl_->pipeline.RestoreRuntimeState(pipeline_state)) {
      session::RecordAbort(&result, session::SarPipelineAbortReason::kRuntimeStateRestoreRejected,
                           session::codes::kRuntimeStateRestoreRejected,
                           "SAR failed to restore pipeline state after cycle abort.");
    }
    // 非执行周期：output_frame 保持默认空帧，不复用上一有效输出。
    impl_->Finish(result);
    return;
  }

  impl_->Finish(result);
}

session::SarCycleResult SarController::BuildCycleResult() const {
  return impl_->latest_result;
}

bool SarController::TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  const session::SarRuntimeConfigResolveResult resolved =
      session::ResolveSarRuntimeConfigPatch(impl_->runtime_config, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return false;
  }
  impl_->runtime_config = resolved.next_config;
  return true;
}

SarControllerRuntimeState SarController::CaptureRuntimeState() const {
  SarControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = kControllerRuntimeStateSchemaVersion;
  state.runtime_config = impl_->runtime_config;
  state.latest_result = impl_->latest_result;
  state.pipeline_state = impl_->pipeline.CaptureRuntimeState();
  return state;
}

bool SarController::RestoreRuntimeState(const SarControllerRuntimeState& state) {
  if (state.owner_identity != this ||
      state.schema_version != kControllerRuntimeStateSchemaVersion ||
      !impl_->pipeline.RestoreRuntimeState(state.pipeline_state)) {
    return false;
  }
  impl_->runtime_config = state.runtime_config;
  impl_->latest_result = state.latest_result;
  return true;
}

}  // namespace extension
}  // namespace sar
