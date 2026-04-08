#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"

#include "1q/electronic_surveillance_radar/core/controller/EsrController.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/pipeline/IInterceptPipeline.h"
#include "electronic_surveillance_radar/core/session/EsrRuntimeConfigResolver.h"
#include "electronic_surveillance_radar/core/session/EsrSessionConfigResolver.h"
#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {

struct EsrSession::Impl {
  explicit Impl(const EsrSessionConfig& config)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        owned_pipeline(
            new pipeline::InterceptPipeline(resolved_config.pipeline_config, resolved_config.runtime_config)),
        owned_environment_service(
            new environment::EsrEnvironmentService(resolved_config.environment_model_config)),
        owned_controller(new controller::EsrController(*owned_pipeline, *owned_environment_service)),
        pipeline(*owned_pipeline),
        environment_service(*owned_environment_service),
        controller(*owned_controller) {}

  Impl(const EsrSessionConfig& config, pipeline::IInterceptPipeline& pipeline_ref)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        owned_environment_service(
            new environment::EsrEnvironmentService(resolved_config.environment_model_config)),
        owned_controller(new controller::EsrController(pipeline_ref, *owned_environment_service)),
        pipeline(pipeline_ref),
        environment_service(*owned_environment_service),
        controller(*owned_controller) {
    pipeline.UpdateConfig(resolved_config.pipeline_config);
    pipeline.UpdateRuntimeConfig(resolved_config.runtime_config);
  }

  Impl(const EsrSessionConfig& config, environment::IEsrEnvironmentService& environment_service_ref)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        owned_pipeline(
            new pipeline::InterceptPipeline(resolved_config.pipeline_config, resolved_config.runtime_config)),
        owned_controller(new controller::EsrController(*owned_pipeline, environment_service_ref)),
        pipeline(*owned_pipeline),
        environment_service(environment_service_ref),
        controller(*owned_controller) {
    environment_service.UpdateModelConfig(resolved_config.environment_model_config);
  }

  Impl(const EsrSessionConfig& config, controller::EsrController& controller_ref)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        pipeline(controller_ref.GetPipeline()),
        environment_service(controller_ref.GetEnvironmentService()),
        controller(controller_ref) {
    pipeline.UpdateConfig(resolved_config.pipeline_config);
    pipeline.UpdateRuntimeConfig(resolved_config.runtime_config);
    environment_service.UpdateModelConfig(resolved_config.environment_model_config);
  }

  Impl(const EsrSessionConfig& config, pipeline::IInterceptPipeline& pipeline_ref,
       environment::IEsrEnvironmentService& environment_service_ref,
       controller::EsrController& controller_ref)
      : resolved_config(internal::ResolveEsrSessionConfig(config)),
        pipeline(pipeline_ref),
        environment_service(environment_service_ref),
        controller(controller_ref) {
    pipeline.UpdateConfig(resolved_config.pipeline_config);
    pipeline.UpdateRuntimeConfig(resolved_config.runtime_config);
    environment_service.UpdateModelConfig(resolved_config.environment_model_config);
  }

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
  std::unique_ptr<pipeline::IInterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<controller::EsrController> owned_controller;
  pipeline::IInterceptPipeline& pipeline;
  environment::IEsrEnvironmentService& environment_service;
  controller::EsrController& controller;
};

EsrSession::EsrSession(EsrSessionConfig config) : impl_(new Impl(config)) {}
EsrSession::EsrSession(EsrSessionConfig config, pipeline::IInterceptPipeline& pipeline)
    : impl_(new Impl(config, pipeline)) {}
EsrSession::EsrSession(EsrSessionConfig config,
                       environment::IEsrEnvironmentService& environment_service)
    : impl_(new Impl(config, environment_service)) {}
EsrSession::EsrSession(EsrSessionConfig config, controller::EsrController& controller)
    : impl_(new Impl(config, controller)) {}
EsrSession::EsrSession(EsrSessionConfig config, pipeline::IInterceptPipeline& pipeline,
                       environment::IEsrEnvironmentService& environment_service,
                       controller::EsrController& controller)
    : impl_(new Impl(config, pipeline, environment_service, controller)) {}

EsrSession::~EsrSession() = default;

common::EsrOutputFrame EsrSession::Step(const context::EsrCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EsrCycleResult EsrSession::StepWithResult(const context::EsrCycleInput& input) {
  impl_->controller.RunOnce(input);
  return impl_->BuildCycleResult();
}

void EsrSession::ApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch) {
  const internal::EsrRuntimeConfigResolveResult resolved =
      internal::ResolveEsrRuntimeConfigPatch(impl_->resolved_config, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return;
  }

  impl_->resolved_config = resolved.next_config;
  if (resolved.runtime_config_changed) {
    impl_->pipeline.UpdateRuntimeConfig(impl_->resolved_config.runtime_config);
  }
  if (resolved.pipeline_config_changed) {
    impl_->pipeline.UpdateConfig(impl_->resolved_config.pipeline_config);
  }
  if (resolved.environment_model_config_changed) {
    impl_->environment_service.UpdateModelConfig(impl_->resolved_config.environment_model_config);
  }
}

}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar
