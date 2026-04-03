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
};

EosController::EosController(pipeline::IEosPipeline& pipeline) : impl_(new Impl(pipeline)) {}

EosController::~EosController() = default;

void EosController::RunOnce(const context::EosCycleInput& input) {
  impl_->last_validation_issues = context::ValidateEosCycleInput(input);
  impl_->has_validation_error = context::HasEosValidationError(impl_->last_validation_issues);
  if (impl_->has_validation_error) {
    impl_->latest_output = common::EosOutputFrame{};
    impl_->latest_output.cycle_index = input.cycle_index;
    impl_->has_latest_output = true;
    return;
  }

  impl_->latest_output = impl_->pipeline.Execute(input);
  impl_->has_latest_output = true;
}

bool EosController::HasLatestOutputFrame() const { return impl_->has_latest_output; }

const common::EosOutputFrame& EosController::GetLatestOutputFrame() const {
  return impl_->latest_output;
}

const context::EosValidationIssueList& EosController::GetLastValidationIssues() const {
  return impl_->last_validation_issues;
}

bool EosController::HasValidationError() const { return impl_->has_validation_error; }

}  // namespace controller
}  // namespace core
}  // namespace electro_optical_sensor
