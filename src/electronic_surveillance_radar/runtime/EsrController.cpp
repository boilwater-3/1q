#include "1q/electronic_surveillance_radar/extension/EsrController.h"

#include <memory>

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/extension/IInterceptPipeline.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electronic_surveillance_radar/output/EsrOutputManager.h"
#include "electronic_surveillance_radar/runtime/components/EsrEnvironmentUpdater.h"
#include "electronic_surveillance_radar/runtime/components/EsrOutputFormatter.h"
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
  oneq::internal::runtime::RuntimeCycleState<session::EsrOutputFrame,
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

  const oneq::internal::runtime::RuntimeCycleStamp stamp =
      oneq::internal::runtime::MakeRuntimeCycleStamp(
          input.cycle_index, impl_->runtime_state.next_batch_id);

  // 校验
  session::ValidationIssueList issues = session::ValidateEsrCycleInput(input);
  impl_->runtime_state.last_validation_issues = issues;

  if (session::HasValidationError(issues)) {
    impl_->last_cycle_executed = false;
    impl_->last_abort_reason = extension::EsrPipelineAbortReason::kValidationRejected;
    impl_->last_cycle_reused_previous_output = impl_->runtime_state.has_latest_output;
    if (!impl_->runtime_state.has_latest_output) {
      impl_->runtime_state.latest_output = output_formatter.BuildEmptyFrame(stamp);
    }
    impl_->runtime_state.has_latest_output = true;
    ++impl_->runtime_state.next_batch_id;
    return;
  }

  // 冻结环境
  environment_updater.FreezeEnvironment(input, stamp);

  // 执行
  session::EsrOutputFrame output_frame =
      signal_processor.Execute(input, impl_->environment_service);
  output_formatter.BuildOutputFrame(stamp, output_frame);
  output_formatter.LogCycleSummary(input, stamp, output_frame);

  impl_->runtime_state.latest_output = output_frame;
  impl_->runtime_state.has_latest_output = true;
  impl_->last_cycle_executed = true;
  impl_->last_cycle_reused_previous_output = false;
  impl_->last_abort_reason = extension::EsrPipelineAbortReason::kNone;
  ++impl_->runtime_state.next_batch_id;
}

bool EsrController::HasLatestOutputFrame() const { return impl_->runtime_state.has_latest_output; }

const session::EsrOutputFrame& EsrController::GetLatestOutputFrame() const {
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
