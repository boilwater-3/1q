#include "electro_optical_sensor/runtime/EosController.h"

#include <cstddef>
#include <memory>

#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electro_optical_sensor/pipeline/EosPipeline.h"
#include "electro_optical_sensor/session/EosDiagnosticUtils.h"

namespace electro_optical_sensor {
namespace extension {

namespace {

constexpr std::uint32_t kControllerRuntimeStateSchemaVersion = 1U;

session::EosCycleStatus DeriveCycleStatus(session::EosPipelineAbortReason reason) {
  switch (reason) {
    case session::EosPipelineAbortReason::kNone:
      return session::EosCycleStatus::kCompleted;
    case session::EosPipelineAbortReason::kSensorPoweredOff:
      return session::EosCycleStatus::kPoweredOff;
    case session::EosPipelineAbortReason::kValidationRejected:
      return session::EosCycleStatus::kRejectedInvalidInput;
    default:
      return session::EosCycleStatus::kRejectedExecution;
  }
}

bool IsCompatibleControllerRuntimeState(const extension::EosControllerRuntimeState& state,
                                        const void* owner_identity) {
  return state.owner_identity == owner_identity &&
         state.schema_version == kControllerRuntimeStateSchemaVersion;
}

}  // namespace

struct EosController::Impl {
  explicit Impl(signal::pipeline::EosPipeline& pipeline_ref) : pipeline(pipeline_ref) {}

  void ResetPerCycleFlags() {
    last_cycle_executed = false;
    last_abort_reason = session::EosPipelineAbortReason::kNone;
  }

  signal::pipeline::EosPipeline& pipeline;
  session::EosOutputFrame latest_output{};
  attribution::EosDetectionAttributionRecordList latest_detection_attributions{};
  session::ValidationIssueList last_validation_issues{};
  bool has_latest_output{false};
  bool has_validation_error{false};
  bool last_cycle_executed{false};
  session::EosPipelineAbortReason last_abort_reason{session::EosPipelineAbortReason::kNone};
};

EosController::EosController(signal::pipeline::EosPipeline& pipeline) : impl_(new Impl(pipeline)) {}

EosController::~EosController() = default;

namespace {

bool IsEosExecuteResultContractValid(
    const extension::EosPipelineExecuteResult& execute_result,
    const ::electro_optical_sensor::session::EosCycleInput& input) {
  if (!execute_result.executed_this_cycle) {
    return false;
  }
  return execute_result.abort_reason == session::EosPipelineAbortReason::kNone;
}

}  // namespace

void EosController::RunOnce(const ::electro_optical_sensor::session::EosCycleInput& input) {
  const extension::EosPipelineRuntimeState previous_pipeline_state =
      impl_->pipeline.CaptureRuntimeState();
  impl_->ResetPerCycleFlags();

  const session::ValidationIssueList issues =
      session::ValidateEosCycleInput(input, impl_->pipeline.GetFrameRateHz());
  impl_->last_validation_issues = issues;
  impl_->has_validation_error = session::HasValidationError(issues);

  if (impl_->has_validation_error) {
    impl_->last_abort_reason = session::EosPipelineAbortReason::kValidationRejected;
    PROJECT_LOG_WARN("EOS validation rejected for cycle_index={}", input.cycle_index);
    return;
  }

  const extension::EosPipelineExecuteResult execute_result = impl_->pipeline.RunCycle(input);

  if (!execute_result.executed_this_cycle &&
      execute_result.abort_reason == session::EosPipelineAbortReason::kSensorPoweredOff) {
    impl_->last_cycle_executed = false;
    impl_->last_abort_reason = session::EosPipelineAbortReason::kSensorPoweredOff;
    return;
  }

  if (!IsEosExecuteResultContractValid(execute_result, input)) {
    const bool restore_ok = impl_->pipeline.RestoreRuntimeState(previous_pipeline_state);
    if (!restore_ok) {
      impl_->latest_output = session::EosOutputFrame{};
      impl_->latest_detection_attributions.clear();
      impl_->has_latest_output = false;
      impl_->last_cycle_executed = false;
      impl_->last_abort_reason = session::EosPipelineAbortReason::kRuntimeStateRestoreRejected;
      PROJECT_LOG_ERROR("EOS pipeline rollback failed for cycle_index={}", input.cycle_index);
      return;
    }
    impl_->latest_output = session::EosOutputFrame{};
    impl_->latest_detection_attributions.clear();
    impl_->has_latest_output = false;
    impl_->last_cycle_executed = false;
    impl_->last_abort_reason =
        execute_result.abort_reason == session::EosPipelineAbortReason::kNone
            ? session::EosPipelineAbortReason::kOutputContractViolation
            : execute_result.abort_reason;
    return;
  }

  session::EosOutputFrame assembled_frame;
  assembled_frame.cycle_index = input.cycle_index;
  assembled_frame.scan_azimuth_deg = execute_result.scan_azimuth_deg;
  assembled_frame.detections = std::move(execute_result.detections);
  impl_->latest_output = assembled_frame;
  impl_->latest_detection_attributions = std::move(execute_result.detection_attributions);
  impl_->has_latest_output = true;
  impl_->last_cycle_executed = true;
  PROJECT_LOG_DEBUG("[EosController] cycle_index={} executed detections={}", input.cycle_index,
                    assembled_frame.detections.size());
}

const session::ValidationIssueList& EosController::GetLastValidationIssues() const {
  return impl_->last_validation_issues;
}

bool EosController::HasValidationError() const { return impl_->has_validation_error; }

bool EosController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

session::EosPipelineAbortReason EosController::GetLastDetectionCycleAbortReason() const {
  return impl_->last_abort_reason;
}

::electro_optical_sensor::session::EosCycleResult EosController::BuildCycleResult(
    const ::electro_optical_sensor::session::EosCycleInput& input) const {
  ::electro_optical_sensor::session::EosCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.validation_issues = impl_->last_validation_issues;
  result.has_validation_error = impl_->has_validation_error;
  result.executed_this_cycle = impl_->last_cycle_executed;
  result.status = DeriveCycleStatus(impl_->last_abort_reason);
  result.abort_reason = impl_->last_abort_reason;
  if (impl_->last_cycle_executed && impl_->has_latest_output) {
    result.output_frame = impl_->latest_output;
    result.detection_attributions = impl_->latest_detection_attributions;
  }

  // 三写：对所有非 kNone 的 abort_reason 写入 diagnostics + 日志
  if (impl_->last_abort_reason != session::EosPipelineAbortReason::kNone) {
    const bool is_validation =
        (impl_->last_abort_reason == session::EosPipelineAbortReason::kValidationRejected);
    const char* detail_code = "unknown";
    switch (impl_->last_abort_reason) {
      case session::EosPipelineAbortReason::kValidationRejected:
        detail_code = "input_validation";
        break;
      case session::EosPipelineAbortReason::kSensorPoweredOff:
        detail_code = "sensor_powered_off";
        break;
      case session::EosPipelineAbortReason::kOutputContractViolation:
        detail_code = "pipeline_contract_violation";
        break;
      case session::EosPipelineAbortReason::kRuntimeStateRestoreRejected:
        detail_code = "runtime_state_restore_rejected";
        break;
      default:
        break;
    }
    session::RecordAbort(&result, impl_->last_abort_reason, detail_code,
                         "EOS cycle aborted.", is_validation);
  }

  return result;
}

extension::EosControllerRuntimeState EosController::CaptureRuntimeState() const {
  extension::EosControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = kControllerRuntimeStateSchemaVersion;
  state.latest_output = impl_->latest_output;
  state.latest_detection_attributions = impl_->latest_detection_attributions;
  state.last_validation_issues = impl_->last_validation_issues;
  state.has_latest_output = impl_->has_latest_output;
  state.has_validation_error = impl_->has_validation_error;
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_abort_reason = impl_->last_abort_reason;
  state.pipeline_state = impl_->pipeline.CaptureRuntimeState();
  return state;
}

bool EosController::RestoreRuntimeState(const extension::EosControllerRuntimeState& state) {
  if (!IsCompatibleControllerRuntimeState(state, this)) {
    return false;
  }
  if (!impl_->pipeline.RestoreRuntimeState(state.pipeline_state)) {
    return false;
  }
  impl_->latest_output = state.latest_output;
  impl_->latest_detection_attributions = state.latest_detection_attributions;
  impl_->last_validation_issues = state.last_validation_issues;
  impl_->has_latest_output = state.has_latest_output;
  impl_->has_validation_error = state.has_validation_error;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_abort_reason = state.last_abort_reason;
  return true;
}

}  // namespace extension
}  // namespace electro_optical_sensor
