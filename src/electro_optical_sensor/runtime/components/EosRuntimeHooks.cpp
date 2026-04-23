#include "electro_optical_sensor/runtime/components/EosRuntimeHooks.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

EosRuntimeHooks::EosRuntimeHooks(
    const EosInputValidator& input_validator, const EosSignalProcessor& signal_processor,
    const EosCycleOutcomeRecorder& outcome_recorder,
    const output::EosOutputFrame& previous_output, bool had_previous_output,
    const extension::EosPipelineRuntimeState& previous_pipeline_state,
    bool& has_validation_error)
    : input_validator_(input_validator),
      signal_processor_(signal_processor),
      outcome_recorder_(outcome_recorder),
      previous_output_(previous_output),
      had_previous_output_(had_previous_output),
      previous_pipeline_state_(previous_pipeline_state),
      has_validation_error_(has_validation_error) {}

oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList>
EosRuntimeHooks::Validate(const ::electro_optical_sensor::session::EosCycleInput& input) const {
  oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> result;
  result.issues = input_validator_.Validate(input);
  result.has_error = input_validator_.HasError(result.issues);
  has_validation_error_ = result.has_error;
  return result;
}

void EosRuntimeHooks::FreezeEnvironment(
    const ::electro_optical_sensor::session::EosCycleInput& input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  (void)input;
  (void)stamp;
}

output::EosOutputFrame EosRuntimeHooks::Execute(
    const ::electro_optical_sensor::session::EosCycleInput& input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  (void)stamp;
  const extension::EosPipelineExecuteResult execute_result = signal_processor_.Execute(input);
  if (!IsExecuteResultContractValid(execute_result, input)) {
    const bool restore_ok = signal_processor_.RestoreRuntimeState(previous_pipeline_state_);
    if (!restore_ok) {
      outcome_recorder_.RecordExecuteContractViolationRollbackFailed();
      return output::EosOutputFrame{};
    }
    outcome_recorder_.RecordExecuteContractViolationRollbackSucceeded(
        previous_output_, had_previous_output_, execute_result.abort_reason);
    if (had_previous_output_) {
      return previous_output_;
    }
    return output::EosOutputFrame{};
  }

  outcome_recorder_.RecordExecuteSucceeded(execute_result);
  return execute_result.output_frame;
}

output::EosOutputFrame EosRuntimeHooks::BuildErrorOutput(
    const ::electro_optical_sensor::session::EosCycleInput& input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  (void)input;
  (void)stamp;
  outcome_recorder_.RecordValidationRejected(previous_output_, had_previous_output_);
  if (had_previous_output_) {
    return previous_output_;
  }
  return output::EosOutputFrame{};
}

bool EosRuntimeHooks::IsExecuteResultContractValid(
    const extension::EosPipelineExecuteResult& execute_result,
    const ::electro_optical_sensor::session::EosCycleInput& input) {
  if (!execute_result.executed_this_cycle) {
    return false;
  }
  return execute_result.abort_reason == extension::EosPipelineAbortReason::kNone &&
         execute_result.output_frame.cycle_index == input.cycle_index;
}

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor
