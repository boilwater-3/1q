#ifndef ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_RUNTIME_HOOKS_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_RUNTIME_HOOKS_H_

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electronic_surveillance_radar/runtime/components/EsrEnvironmentUpdater.h"
#include "electronic_surveillance_radar/runtime/components/EsrOutputFormatter.h"
#include "electronic_surveillance_radar/runtime/components/EsrSignalProcessor.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

class EsrRuntimeHooks {
 public:
  EsrRuntimeHooks(
      EsrEnvironmentUpdater& environment_updater, EsrSignalProcessor& signal_processor,
      EsrOutputFormatter& output_formatter,
      const environment::IEsrEnvironmentService& environment_service,
      oneq::internal::runtime::RuntimeCycleState<output::EsrOutputFrame,
                                                 session::ValidationIssueList>& runtime_state,
      bool& last_cycle_executed, bool& last_cycle_reused_previous_output,
      extension::EsrPipelineAbortReason& last_abort_reason);

  oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> Validate(
      const session::EsrCycleInput& cycle_input) const;

  void FreezeEnvironment(const session::EsrCycleInput& cycle_input,
                         const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

  output::EsrOutputFrame Execute(const session::EsrCycleInput& cycle_input,
                                 const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

  output::EsrOutputFrame BuildErrorOutput(
      const session::EsrCycleInput& cycle_input,
      const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

 private:
  EsrEnvironmentUpdater& environment_updater_;
  EsrSignalProcessor& signal_processor_;
  EsrOutputFormatter& output_formatter_;
  const environment::IEsrEnvironmentService& environment_service_;
  oneq::internal::runtime::RuntimeCycleState<output::EsrOutputFrame,
                                             session::ValidationIssueList>& runtime_state_;
  bool& last_cycle_executed_;
  bool& last_cycle_reused_previous_output_;
  extension::EsrPipelineAbortReason& last_abort_reason_;
};

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_RUNTIME_HOOKS_H_
