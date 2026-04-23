#include "1q/electronic_surveillance_radar/extension/EsrController.h"

#include <memory>

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/extension/IInterceptPipeline.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electronic_surveillance_radar/output/EsrOutputManager.h"
#include "electronic_surveillance_radar/runtime/components/EsrEnvironmentUpdater.h"
#include "electronic_surveillance_radar/runtime/components/EsrOutputFormatter.h"
#include "electronic_surveillance_radar/runtime/components/EsrRuntimeHooks.h"
#include "electronic_surveillance_radar/runtime/components/EsrSignalProcessor.h"

namespace electronic_surveillance_radar {
namespace extension {

struct EsrController::Impl {
  Impl(extension::IInterceptPipeline& pipeline_ref,
       environment::IEsrEnvironmentService& environment_service_ref)
      : pipeline(pipeline_ref), environment_service(environment_service_ref) {}

  extension::IInterceptPipeline& pipeline;
  environment::IEsrEnvironmentService& environment_service;
  output::EsrOutputManager output_manager;
  oneq::internal::runtime::RuntimeCycleState<output::EsrOutputFrame,
                                             session::ValidationIssueList>
      runtime_state{};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  extension::EsrPipelineAbortReason last_abort_reason{extension::EsrPipelineAbortReason::kNone};
};

EsrController::EsrController(extension::IInterceptPipeline& pipeline,
                             environment::IEsrEnvironmentService& environment_service)
    : impl_(new Impl(pipeline, environment_service)) {}

EsrController::~EsrController() = default;

void EsrController::RunOnce(const session::EsrCycleInput& input) {
  runtime::components::EsrEnvironmentUpdater environment_updater(impl_->environment_service);
  runtime::components::EsrSignalProcessor signal_processor(impl_->pipeline);
  runtime::components::EsrOutputFormatter output_formatter(impl_->output_manager);
  runtime::components::EsrRuntimeHooks hooks(
      environment_updater, signal_processor, output_formatter, impl_->environment_service,
      impl_->runtime_state, impl_->last_cycle_executed, impl_->last_cycle_reused_previous_output,
      impl_->last_abort_reason);
  oneq::internal::runtime::ExecuteRuntimeCycle(input, input.cycle_index, &impl_->runtime_state,
                                               &hooks);
}

bool EsrController::HasLatestOutputFrame() const { return impl_->runtime_state.has_latest_output; }

const output::EsrOutputFrame& EsrController::GetLatestOutputFrame() const {
  return impl_->runtime_state.latest_output;
}

const session::ValidationIssueList& EsrController::GetLastValidationIssues() const {
  return impl_->runtime_state.last_validation_issues;
}

bool EsrController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

bool EsrController::ReusedPreviousOutputLatestCycle() const {
  return impl_->last_cycle_reused_previous_output;
}

extension::EsrPipelineAbortReason EsrController::GetLastAbortReason() const {
  return impl_->last_abort_reason;
}

extension::IInterceptPipeline& EsrController::GetPipeline() { return impl_->pipeline; }

environment::IEsrEnvironmentService& EsrController::GetEnvironmentService() {
  return impl_->environment_service;
}

}  // namespace extension

}  // namespace electronic_surveillance_radar
