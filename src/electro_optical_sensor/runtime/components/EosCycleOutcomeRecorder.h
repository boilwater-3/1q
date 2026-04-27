#ifndef ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_CYCLE_OUTCOME_RECORDER_H_
#define ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_CYCLE_OUTCOME_RECORDER_H_

#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

class EosCycleOutcomeRecorder {
 public:
  EosCycleOutcomeRecorder(session::EosOutputFrame& latest_output, bool& has_latest_output,
                          bool& last_cycle_executed, bool& last_cycle_reused_previous_output,
                          extension::EosPipelineAbortReason& last_abort_reason);

  void ResetPerCycleFlags() const;

  void RecordValidationRejected(const session::EosOutputFrame& previous_output,
                                bool had_previous_output) const;

  void RecordExecuteContractViolationRollbackFailed() const;

  void RecordExecuteContractViolationRollbackSucceeded(
      const session::EosOutputFrame& previous_output, bool had_previous_output,
      extension::EosPipelineAbortReason abort_reason) const;

  void RecordExecuteSucceeded(const session::EosOutputFrame& assembled_frame) const;

 private:
  static extension::EosPipelineAbortReason NormalizeAbortReason(
      extension::EosPipelineAbortReason abort_reason);

  session::EosOutputFrame& latest_output_;
  bool& has_latest_output_;
  bool& last_cycle_executed_;
  bool& last_cycle_reused_previous_output_;
  extension::EosPipelineAbortReason& last_abort_reason_;
};

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_CYCLE_OUTCOME_RECORDER_H_
