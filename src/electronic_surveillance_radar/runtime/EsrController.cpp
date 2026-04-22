#include "1q/electronic_surveillance_radar/extension/EsrController.h"

#include <cstddef>
#include <memory>

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/extension/IInterceptPipeline.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electronic_surveillance_radar/output/EsrOutputManager.h"
#include "electronic_surveillance_radar/runtime/EsrCycleTelemetryLogger.h"

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
};

EsrController::EsrController(extension::IInterceptPipeline& pipeline,
                             environment::IEsrEnvironmentService& environment_service)
    : impl_(new Impl(pipeline, environment_service)) {}

EsrController::~EsrController() = default;

void EsrController::RunOnce(const session::EsrCycleInput& input) {
  struct EsrRuntimeHooks {
    Impl* impl{nullptr};

    oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> Validate(
        const session::EsrCycleInput& cycle_input) const {
      oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> result;
      result.issues = session::ValidateEsrCycleInput(cycle_input);
      result.has_error = session::HasValidationError(result.issues);
      return result;
    }

    void FreezeEnvironment(const session::EsrCycleInput& cycle_input,
                           const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      if (impl == nullptr) {
        return;
      }
      environment::EsrEnvironmentCycleContext environment_context;
      environment_context.cycle_index = stamp.cycle_index;
      environment_context.dt_sec = cycle_input.dt_sec;
      environment_context.observation = cycle_input.environment_observation;
      impl->environment_service.BeginCycle(environment_context);
    }

    output::EsrOutputFrame Execute(const session::EsrCycleInput& cycle_input,
                                   const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      const extension::InterceptCycleResult intercept_result =
          impl->pipeline.RunCycle(cycle_input, impl->environment_service);
      const output::EsrOutputFrame output_frame = impl->output_manager.BuildOutputFrame(
          stamp.cycle_index, stamp.batch_id, intercept_result);

      std::size_t matched_truth_count = 0U;
      for (std::size_t i = 0; i < output_frame.truth_evaluation_output.associations.size(); ++i) {
        if (output_frame.truth_evaluation_output.associations[i].matched) {
          ++matched_truth_count;
        }
      }

      const runtime::EsrCycleTelemetryPayload payload(
          stamp, cycle_input.scene_emitters.size(), intercept_result.raw_observation_count,
          output_frame.observation_output.observations.size(), intercept_result.cluster_count,
          output_frame.emitter_output.hypotheses.size(),
          output_frame.truth_evaluation_output.associations.size(), matched_truth_count, true);
      runtime::EsrCycleTelemetryLogger::LogCycleSummary(payload);

      return output_frame;
    }

    output::EsrOutputFrame BuildErrorOutput(
        const session::EsrCycleInput& cycle_input,
        const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      (void)cycle_input;
      return impl->output_manager.BuildEmptyFrame(stamp.cycle_index, stamp.batch_id);
    }
  };

  EsrRuntimeHooks hooks;
  hooks.impl = impl_.get();
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

extension::IInterceptPipeline& EsrController::GetPipeline() { return impl_->pipeline; }

environment::IEsrEnvironmentService& EsrController::GetEnvironmentService() {
  return impl_->environment_service;
}

}  // namespace extension

}  // namespace electronic_surveillance_radar
