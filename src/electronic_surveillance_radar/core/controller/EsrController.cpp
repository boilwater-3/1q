#include "1q/electronic_surveillance_radar/core/controller/EsrController.h"

#include <memory>

#include <spdlog/spdlog.h>

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/pipeline/IInterceptPipeline.h"
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
  common::EsrOutputFrame latest_output_frame{};
  bool has_latest_output_frame{false};
  context::EsrValidationIssueList last_validation_issues{};
  std::uint64_t batch_id{1U};
};

EsrController::EsrController(pipeline::IInterceptPipeline& pipeline,
                             environment::IEsrEnvironmentService&
                                 environment_service)
    : impl_(new Impl(pipeline, environment_service)) {}

EsrController::~EsrController() = default;

void EsrController::RunOnce(const context::EsrCycleInput& input) {
  impl_->last_validation_issues = context::ValidateEsrCycleInput(input);
  if (context::HasEsrValidationError(impl_->last_validation_issues)) {
    impl_->latest_output_frame =
        impl_->output_manager.BuildEmptyFrame(input.cycle_index, impl_->batch_id);
    impl_->has_latest_output_frame = true;
    ++impl_->batch_id;
    return;
  }

  environment::EsrEnvironmentCycleContext environment_context;
  environment_context.cycle_index = input.cycle_index;
  environment_context.dt_sec = input.dt_sec;
  environment_context.scene_state = input.environment_scene_state;
  impl_->environment_service.BeginCycle(environment_context);

  const pipeline::InterceptCycleResult intercept_result = impl_->pipeline.RunCycle(
      input, impl_->environment_service);
  impl_->latest_output_frame = impl_->output_manager.BuildOutputFrame(
      input.cycle_index, impl_->batch_id, intercept_result);
  impl_->has_latest_output_frame = true;

  spdlog::debug(
      "[EsrController] cycle summary: cycle_index={} batch_id={} "
      "observations={} hypotheses={} truth_associations={}",
      input.cycle_index, impl_->batch_id,
      impl_->latest_output_frame.observation_output.observations.size(),
      impl_->latest_output_frame.emitter_output.hypotheses.size(),
      impl_->latest_output_frame.truth_evaluation_output.associations.size());

  ++impl_->batch_id;
}

bool EsrController::HasLatestOutputFrame() const {
  return impl_->has_latest_output_frame;
}

const common::EsrOutputFrame& EsrController::GetLatestOutputFrame() const {
  return impl_->latest_output_frame;
}

const context::EsrValidationIssueList& EsrController::GetLastValidationIssues()
    const {
  return impl_->last_validation_issues;
}

}  // namespace controller
}  // namespace core
}  // namespace electronic_surveillance_radar
