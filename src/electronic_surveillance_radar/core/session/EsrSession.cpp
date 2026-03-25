#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"

#include "1q/electronic_surveillance_radar/core/controller/EsrController.h"
#include "electronic_surveillance_radar/core/session/EsrSessionConfigResolver.h"
#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {

struct EsrSession::Impl {
  explicit Impl(const EsrSessionConfig& config)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        pipeline(resolved_config.pipeline_config, resolved_config.runtime_config),
        environment_service(resolved_config.environment_config),
        controller(pipeline, environment_service) {}

  // 装配当前周期会话结果。
  EsrCycleResult BuildCycleResult() const {
    EsrCycleResult result;
    if (controller.HasLatestOutputFrame()) {
      result.output_frame = controller.GetLatestOutputFrame();
    }
    result.validation_issues = controller.GetLastValidationIssues();
    result.has_validation_error = context::HasEsrValidationError(result.validation_issues);
    return result;
  }

  internal::ResolvedEsrSessionConfig resolved_config{};
  pipeline::InterceptPipeline pipeline;
  environment::EsrEnvironmentService environment_service;
  controller::EsrController controller;
};

EsrSession::EsrSession(EsrSessionConfig config) : impl_(new Impl(config)) {}

EsrSession::~EsrSession() = default;

common::EsrOutputFrame EsrSession::Step(const context::EsrCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EsrCycleResult EsrSession::StepWithResult(const context::EsrCycleInput& input) {
  impl_->controller.RunOnce(input);
  return impl_->BuildCycleResult();
}

}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar
