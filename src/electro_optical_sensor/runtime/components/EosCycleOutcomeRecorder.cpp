#include "electro_optical_sensor/runtime/components/EosCycleOutcomeRecorder.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

EosCycleOutcomeRecorder::EosCycleOutcomeRecorder(
    output::EosOutputFrame& latest_output, bool& has_latest_output, bool& last_cycle_executed,
    bool& last_cycle_reused_previous_output,
    extension::EosPipelineAbortReason& last_abort_reason)
    : latest_output_(latest_output),
      has_latest_output_(has_latest_output),
      last_cycle_executed_(last_cycle_executed),
      last_cycle_reused_previous_output_(last_cycle_reused_previous_output),
      last_abort_reason_(last_abort_reason) {}

void EosCycleOutcomeRecorder::ResetPerCycleFlags() const {
  last_cycle_executed_ = false;
  last_cycle_reused_previous_output_ = false;
  last_abort_reason_ = extension::EosPipelineAbortReason::kNone;
}

void EosCycleOutcomeRecorder::RecordValidationRejected(
    const output::EosOutputFrame& previous_output, bool had_previous_output) const {
  last_abort_reason_ = extension::EosPipelineAbortReason::kValidationRejected;
  last_cycle_reused_previous_output_ = had_previous_output;
  latest_output_ = previous_output;
  has_latest_output_ = had_previous_output;
}

void EosCycleOutcomeRecorder::RecordExecuteContractViolationRollbackFailed() const {
  latest_output_ = output::EosOutputFrame{};
  has_latest_output_ = false;
  last_cycle_executed_ = false;
  last_cycle_reused_previous_output_ = false;
  last_abort_reason_ = extension::EosPipelineAbortReason::kRuntimeStateRestoreRejected;
}

void EosCycleOutcomeRecorder::RecordExecuteContractViolationRollbackSucceeded(
    const output::EosOutputFrame& previous_output, bool had_previous_output,
    extension::EosPipelineAbortReason abort_reason) const {
  latest_output_ = previous_output;
  has_latest_output_ = had_previous_output;
  last_cycle_executed_ = false;
  last_cycle_reused_previous_output_ = had_previous_output;
  last_abort_reason_ = NormalizeAbortReason(abort_reason);
}

void EosCycleOutcomeRecorder::RecordExecuteSucceeded(
    const extension::EosPipelineExecuteResult& execute_result) const {
  latest_output_ = execute_result.output_frame;
  has_latest_output_ = true;
  last_cycle_executed_ = true;
}

extension::EosPipelineAbortReason EosCycleOutcomeRecorder::NormalizeAbortReason(
    extension::EosPipelineAbortReason abort_reason) {
  if (abort_reason == extension::EosPipelineAbortReason::kNone) {
    return extension::EosPipelineAbortReason::kOutputContractViolation;
  }
  return abort_reason;
}

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor
