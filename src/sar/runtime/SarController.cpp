#include "sar/runtime/SarController.h"

#include <sstream>
#include <utility>

#include "1q/sar/session/SarInputValidation.h"
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
  session::SarDiagnosticIssueList errors;
  for (const session::SarDiagnosticIssue& issue : result->diagnostics) {
    if (issue.severity == session::SarDiagnosticSeverity::kError) {
      errors.push_back(issue);
    }
  }
  result->diagnostics.swap(errors);
}

std::string BuildInputValidationAbortMessage(const session::ValidationIssueList& issues) {
  std::ostringstream message;
  message << "SAR cycle input validation failed";
  if (!issues.empty()) {
    message << ": " << issues.front().field << ": " << issues.front().message;
  }
  return message.str();
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

  const session::ValidationIssueList input_issues = session::ValidateSarCycleInput(input);
  if (session::HasValidationError(input_issues)) {
    session::RecordValidationAbort(&result, session::SarPipelineAbortReason::kValidationRejected,
                         "invalid_cycle_input",
                         BuildInputValidationAbortMessage(input_issues));
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

  // 所有前置校验通过后才写入输出帧元数据，确保失败周期输出严格为默认空帧。
  result.output_frame.cycle_index = input.cycle_index;
  session::InitializeOutputFrameMetadata(impl_->runtime_config, &result.output_frame);

  const pipeline::SarProcessingPipelineRuntimeState pipeline_state =
      impl_->pipeline.CaptureRuntimeState();
  if (!impl_->pipeline.RunCycle(impl_->runtime_config, input, &result)) {
    if (!impl_->pipeline.RestoreRuntimeState(pipeline_state)) {
      session::RecordAbort(&result, session::SarPipelineAbortReason::kRuntimeStateRestoreRejected,
                           "runtime_state_restore_rejected",
                           "SAR failed to restore pipeline state after cycle abort.");
    }
    // 非执行周期：output_frame 保持默认空帧，不复用上一有效输出。
    impl_->Finish(result);
    return;
  }

  impl_->Finish(result);
}

session::SarCycleResult SarController::BuildCycleResult(const session::SarCycleInput& input) const {
  (void)input;
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
