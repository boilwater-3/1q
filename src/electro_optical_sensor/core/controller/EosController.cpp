#include "1q/electro_optical_sensor/core/controller/EosController.h"

#include <memory>

#include "1q/electro_optical_sensor/pipeline/IEosPipeline.h"

namespace electro_optical_sensor {
namespace core {
namespace controller {

struct EosController::Impl {
  explicit Impl(pipeline::IEosPipeline& pipeline_ref) : pipeline(pipeline_ref) {}

  pipeline::IEosPipeline& pipeline;
  common::EosOutputFrame latest_output{};
  context::EosValidationIssueList last_validation_issues{};
  bool has_latest_output{false};
  bool has_validation_error{false};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
};

EosController::EosController(pipeline::IEosPipeline& pipeline) : impl_(new Impl(pipeline)) {}

EosController::~EosController() = default;

void EosController::RunOnce(const context::EosCycleInput& input) {
  impl_->last_cycle_executed = false;
  impl_->last_cycle_reused_previous_output = false;
  impl_->last_validation_issues = context::ValidateEosCycleInput(input);
  impl_->has_validation_error = context::HasEosValidationError(impl_->last_validation_issues);
  if (impl_->has_validation_error) {
    if (!impl_->has_latest_output) {
      impl_->latest_output = common::EosOutputFrame{};
      impl_->latest_output.cycle_index = input.cycle_index;
      impl_->has_latest_output = true;
      return;
    }
    impl_->last_cycle_reused_previous_output = true;
    impl_->has_latest_output = true;
    return;
  }

  impl_->latest_output = impl_->pipeline.Execute(input);
  impl_->has_latest_output = true;
  impl_->last_cycle_executed = true;
}

bool EosController::HasLatestOutputFrame() const { return impl_->has_latest_output; }

const common::EosOutputFrame& EosController::GetLatestOutputFrame() const {
  return impl_->latest_output;
}

const context::EosValidationIssueList& EosController::GetLastValidationIssues() const {
  return impl_->last_validation_issues;
}

bool EosController::HasValidationError() const { return impl_->has_validation_error; }

bool EosController::ExecutedLatestCycle() const { return impl_->last_cycle_executed; }

bool EosController::ReusedPreviousOutputLatestCycle() const {
  return impl_->last_cycle_reused_previous_output;
}

pipeline::IEosPipeline& EosController::GetPipeline() { return impl_->pipeline; }

}  // namespace controller
}  // namespace core
}  // namespace electro_optical_sensor
