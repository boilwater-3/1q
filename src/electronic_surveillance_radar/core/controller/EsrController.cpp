#include "1q/electronic_surveillance_radar/core/controller/EsrController.h"

#include <memory>

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/pipeline/IInterceptPipeline.h"
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electronic_surveillance_radar/core/output/EsrOutputManager.h"

namespace electronic_surveillance_radar {
namespace core {
namespace controller {

struct EsrController::Impl {
  Impl(pipeline::IInterceptPipeline& pipeline_ref,
       environment::IEsrEnvironmentService& environment_service_ref)
      : pipeline(pipeline_ref), environment_service(environment_service_ref) {}

  pipeline::IInterceptPipeline& pipeline;
  environment::IEsrEnvironmentService& environment_service;
  output::EsrOutputManager output_manager;
  oneq::internal::runtime::RuntimeCycleState<common::EsrOutputFrame,
                                             context::EsrValidationIssueList>
      runtime_state{};
};

EsrController::EsrController(pipeline::IInterceptPipeline& pipeline,
                             environment::IEsrEnvironmentService& environment_service)
    : impl_(new Impl(pipeline, environment_service)) {}

EsrController::~EsrController() = default;

void EsrController::RunOnce(const context::EsrCycleInput& input) {
  struct EsrRuntimeHooks {
    Impl* impl{nullptr};

    oneq::internal::runtime::RuntimeValidationResult<context::EsrValidationIssueList> Validate(
        const context::EsrCycleInput& cycle_input) const {
      oneq::internal::runtime::RuntimeValidationResult<context::EsrValidationIssueList> result;
      result.issues = context::ValidateEsrCycleInput(cycle_input);
      result.has_error = context::HasEsrValidationError(result.issues);
      return result;
    }

    void FreezeEnvironment(const context::EsrCycleInput& cycle_input,
                           const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      if (impl == nullptr) {
        return;
      }
      environment::EsrEnvironmentCycleContext environment_context;
      environment_context.cycle_index = stamp.cycle_index;
      environment_context.dt_sec = cycle_input.dt_sec;
      environment_context.scene_state = cycle_input.environment_scene_state;
      impl->environment_service.BeginCycle(environment_context);
    }

    common::EsrOutputFrame Execute(const context::EsrCycleInput& cycle_input,
                                   const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      const pipeline::InterceptCycleResult intercept_result =
          impl->pipeline.RunCycle(cycle_input, impl->environment_service);
      const common::EsrOutputFrame output_frame = impl->output_manager.BuildOutputFrame(
          stamp.cycle_index, stamp.batch_id, intercept_result);
      PROJECT_LOG_DEBUG(
          "[EsrController] cycle summary: cycle_index={} batch_id={} "
          "observations={} hypotheses={} truth_associations={}",
          stamp.cycle_index, stamp.batch_id, output_frame.observation_output.observations.size(),
          output_frame.emitter_output.hypotheses.size(),
          output_frame.truth_evaluation_output.associations.size());
      return output_frame;
    }

    common::EsrOutputFrame BuildErrorOutput(
        const context::EsrCycleInput& cycle_input,
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

const common::EsrOutputFrame& EsrController::GetLatestOutputFrame() const {
  return impl_->runtime_state.latest_output;
}

const context::EsrValidationIssueList& EsrController::GetLastValidationIssues() const {
  return impl_->runtime_state.last_validation_issues;
}

}  // namespace controller
}  // namespace core
}  // namespace electronic_surveillance_radar
