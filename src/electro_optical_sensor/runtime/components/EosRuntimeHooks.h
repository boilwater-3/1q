#ifndef ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_RUNTIME_HOOKS_H_
#define ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_RUNTIME_HOOKS_H_

#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electro_optical_sensor/runtime/components/EosCycleOutcomeRecorder.h"
#include "electro_optical_sensor/runtime/components/EosInputValidator.h"
#include "electro_optical_sensor/runtime/components/EosSignalProcessor.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

class EosRuntimeHooks {
 public:
  EosRuntimeHooks(const EosInputValidator& input_validator,
                  const EosSignalProcessor& signal_processor,
                  const EosCycleOutcomeRecorder& outcome_recorder,
                  const output::EosOutputFrame& previous_output, bool had_previous_output,
                  const extension::EosPipelineRuntimeState& previous_pipeline_state,
                  bool& has_validation_error);

  oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> Validate(
      const ::electro_optical_sensor::session::EosCycleInput& input) const;

  void FreezeEnvironment(const ::electro_optical_sensor::session::EosCycleInput& input,
                         const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

  output::EosOutputFrame Execute(
      const ::electro_optical_sensor::session::EosCycleInput& input,
      const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

  output::EosOutputFrame BuildErrorOutput(
      const ::electro_optical_sensor::session::EosCycleInput& input,
      const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

 private:
  static bool IsExecuteResultContractValid(
      const extension::EosPipelineExecuteResult& execute_result,
      const ::electro_optical_sensor::session::EosCycleInput& input);

  const EosInputValidator& input_validator_;
  const EosSignalProcessor& signal_processor_;
  const EosCycleOutcomeRecorder& outcome_recorder_;
  const output::EosOutputFrame& previous_output_;
  bool had_previous_output_{false};
  const extension::EosPipelineRuntimeState& previous_pipeline_state_;
  bool& has_validation_error_;
};

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_RUNTIME_HOOKS_H_
