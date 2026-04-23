#include "electronic_surveillance_radar/runtime/components/EsrRuntimeHooks.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

EsrRuntimeHooks::EsrRuntimeHooks(
    EsrEnvironmentUpdater& environment_updater, EsrSignalProcessor& signal_processor,
    EsrOutputFormatter& output_formatter,
    const environment::IEsrEnvironmentService& environment_service,
    oneq::internal::runtime::RuntimeCycleState<output::EsrOutputFrame,
                                               session::ValidationIssueList>& runtime_state,
    bool& last_cycle_executed, bool& last_cycle_reused_previous_output,
    extension::EsrPipelineAbortReason& last_abort_reason)
    : environment_updater_(environment_updater),
      signal_processor_(signal_processor),
      output_formatter_(output_formatter),
      environment_service_(environment_service),
      runtime_state_(runtime_state),
      last_cycle_executed_(last_cycle_executed),
      last_cycle_reused_previous_output_(last_cycle_reused_previous_output),
      last_abort_reason_(last_abort_reason) {}

oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList>
EsrRuntimeHooks::Validate(const session::EsrCycleInput& cycle_input) const {
  oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> result;
  result.issues = session::ValidateEsrCycleInput(cycle_input);
  result.has_error = session::HasValidationError(result.issues);
  return result;
}

void EsrRuntimeHooks::FreezeEnvironment(
    const session::EsrCycleInput& cycle_input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  environment_updater_.FreezeEnvironment(cycle_input, stamp);
}

output::EsrOutputFrame EsrRuntimeHooks::Execute(
    const session::EsrCycleInput& cycle_input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  const extension::InterceptCycleResult intercept_result =
      signal_processor_.Execute(cycle_input, environment_service_);
  const output::EsrOutputFrame output_frame =
      output_formatter_.BuildOutputFrame(stamp, intercept_result);
  output_formatter_.LogCycleSummary(cycle_input, stamp, intercept_result, output_frame);

  last_cycle_executed_ = true;
  last_cycle_reused_previous_output_ = false;
  last_abort_reason_ = extension::EsrPipelineAbortReason::kNone;

  return output_frame;
}

output::EsrOutputFrame EsrRuntimeHooks::BuildErrorOutput(
    const session::EsrCycleInput& cycle_input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  (void)cycle_input;
  last_cycle_executed_ = false;
  last_abort_reason_ = extension::EsrPipelineAbortReason::kValidationRejected;

  if (runtime_state_.has_latest_output) {
    last_cycle_reused_previous_output_ = true;
    return runtime_state_.latest_output;
  }

  last_cycle_reused_previous_output_ = false;
  return output_formatter_.BuildEmptyFrame(stamp);
}

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar
