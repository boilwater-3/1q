#include "1q/electro_optical_sensor/extension/EosController.h"

#include <cstddef>
#include <memory>

#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electro_optical_sensor/runtime/components/EosCycleOutcomeRecorder.h"
#include "electro_optical_sensor/runtime/components/EosInputValidator.h"
#include "electro_optical_sensor/runtime/components/EosSignalProcessor.h"

namespace electro_optical_sensor {
namespace extension {

namespace {

constexpr std::uint32_t kControllerRuntimeStateSchemaVersion = 1U;

bool IsCompatibleControllerRuntimeState(const extension::EosControllerRuntimeState& state,
                                        const void* owner_identity) {
  return state.owner_identity == owner_identity &&
         state.schema_version == kControllerRuntimeStateSchemaVersion;
}

}  // namespace

struct EosController::Impl {
  explicit Impl(extension::IEosPipeline& pipeline_ref) : pipeline(pipeline_ref) {}

  extension::IEosPipeline& pipeline;
  session::EosOutputFrame latest_output{};
  session::ValidationIssueList last_validation_issues{};
  bool has_latest_output{false};
  bool has_validation_error{false};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  extension::EosPipelineAbortReason last_abort_reason{extension::EosPipelineAbortReason::kNone};
};

EosController::EosController(extension::IEosPipeline& pipeline) : impl_(new Impl(pipeline)) {}

EosController::~EosController() = default;

namespace {

/** @brief 校验 EOS 执行结果是否符合契约。 */
bool IsEosExecuteResultContractValid(
    const extension::EosPipelineExecuteResult& execute_result,
    const ::electro_optical_sensor::session::EosCycleInput& input) {
  if (!execute_result.executed_this_cycle) {
    return false;
  }
  return execute_result.abort_reason == extension::EosPipelineAbortReason::kNone;
}

}  // namespace

void EosController::RunOnce(const ::electro_optical_sensor::session::EosCycleInput& input) {
  runtime::components::EosInputValidator input_validator;
  runtime::components::EosSignalProcessor signal_processor(impl_->pipeline);
  runtime::components::EosCycleOutcomeRecorder outcome_recorder(
      impl_->latest_output, impl_->has_latest_output, impl_->last_cycle_executed,
      impl_->last_cycle_reused_previous_output, impl_->last_abort_reason);

  const session::EosOutputFrame previous_output = impl_->latest_output;
  const bool had_previous_output = impl_->has_latest_output;
  const extension::EosPipelineRuntimeState previous_pipeline_state =
      signal_processor.CaptureRuntimeState();
  const oneq::internal::runtime::RuntimeCycleStamp stamp =
      oneq::internal::runtime::MakeRuntimeCycleStamp(input.cycle_index, 0U);

  outcome_recorder.ResetPerCycleFlags();

  // 校验
  const session::ValidationIssueList issues = input_validator.Validate(input);
  impl_->last_validation_issues = issues;
  impl_->has_validation_error = input_validator.HasError(issues);

  if (impl_->has_validation_error) {
    outcome_recorder.RecordValidationRejected(previous_output, had_previous_output);
    if (had_previous_output) {
      impl_->latest_output = previous_output;
    } else {
      impl_->latest_output = session::EosOutputFrame{};
    }
    return;
  }

  // FreezeEnvironment（EOS 当前为空操作）
  (void)stamp;

  // 执行信号流水线
  const extension::EosPipelineExecuteResult execute_result = signal_processor.Execute(input);

  if (!IsEosExecuteResultContractValid(execute_result, input)) {
    const bool restore_ok = signal_processor.RestoreRuntimeState(previous_pipeline_state);
    if (!restore_ok) {
      outcome_recorder.RecordExecuteContractViolationRollbackFailed();
      impl_->latest_output = session::EosOutputFrame{};
      return;
    }
    outcome_recorder.RecordExecuteContractViolationRollbackSucceeded(
        previous_output, had_previous_output, execute_result.abort_reason);
    impl_->latest_output = previous_output;
    impl_->has_latest_output = had_previous_output;
    return;
  }

  session::EosOutputFrame assembled_frame;
  assembled_frame.cycle_index = input.cycle_index;
  assembled_frame.scan_azimuth_deg = execute_result.scan_azimuth_deg;
  assembled_frame.detections = std::move(execute_result.detections);
  outcome_recorder.RecordExecuteSucceeded(assembled_frame);
  impl_->latest_output = assembled_frame;
  impl_->has_latest_output = true;

}

bool EosController::HasLatestOutputFrame() const { return impl_->has_latest_output; }

const session::EosOutputFrame& EosController::GetLatestOutputFrame() const {
  return impl_->latest_output;
}

const session::ValidationIssueList& EosController::GetLastValidationIssues() const {
  return impl_->last_validation_issues;
}

bool EosController::HasValidationError() const { return impl_->has_validation_error; }

bool EosController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

bool EosController::ReusedPreviousOutputLatestCycle() const {
  return impl_->last_cycle_reused_previous_output;
}

extension::EosPipelineAbortReason EosController::GetLastAbortReason() const {
  return impl_->last_abort_reason;
}

::electro_optical_sensor::session::EosCycleResult EosController::BuildCycleResult(
    const ::electro_optical_sensor::session::EosCycleInput& input) const {
  ::electro_optical_sensor::session::EosCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.validation_issues = impl_->last_validation_issues;
  result.has_validation_error = impl_->has_validation_error;
  result.executed_this_cycle = impl_->last_cycle_executed;
  result.reused_previous_output = impl_->last_cycle_reused_previous_output;
  result.abort_reason = impl_->last_abort_reason;
  if (impl_->has_latest_output) {
    result.output_frame = impl_->latest_output;
  } else {
    result.output_frame.cycle_index = input.cycle_index;
  }
  return result;
}

extension::IEosPipeline& EosController::GetPipeline() { return impl_->pipeline; }

extension::EosControllerRuntimeState EosController::CaptureRuntimeState() const {
  extension::EosControllerRuntimeState state;
  state.owner_identity = this;
  state.schema_version = kControllerRuntimeStateSchemaVersion;
  state.latest_output = impl_->latest_output;
  state.last_validation_issues = impl_->last_validation_issues;
  state.has_latest_output = impl_->has_latest_output;
  state.has_validation_error = impl_->has_validation_error;
  state.last_cycle_executed = impl_->last_cycle_executed;
  state.last_cycle_reused_previous_output = impl_->last_cycle_reused_previous_output;
  state.last_abort_reason = impl_->last_abort_reason;
  state.pipeline_state = impl_->pipeline.CaptureRuntimeState();
  return state;
}

void EosController::RestoreRuntimeState(const extension::EosControllerRuntimeState& state) {
  if (!IsCompatibleControllerRuntimeState(state, this)) {
    return;
  }
  if (!impl_->pipeline.RestoreRuntimeState(state.pipeline_state)) {
    return;
  }
  impl_->latest_output = state.latest_output;
  impl_->last_validation_issues = state.last_validation_issues;
  impl_->has_latest_output = state.has_latest_output;
  impl_->has_validation_error = state.has_validation_error;
  impl_->last_cycle_executed = state.last_cycle_executed;
  impl_->last_cycle_reused_previous_output = state.last_cycle_reused_previous_output;
  impl_->last_abort_reason = state.last_abort_reason;
}

}  // namespace extension
}  // namespace electro_optical_sensor
