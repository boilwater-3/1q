#include "1q/electronic_surveillance_radar/session/EsrSession.h"

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/extension/EsrController.h"
#include "1q/electronic_surveillance_radar/extension/IInterceptPipeline.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"
#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"
#include "electronic_surveillance_radar/session/EsrSessionCompositionRoot.h"

namespace electronic_surveillance_radar {
namespace session {

struct EsrSession::Impl {
  explicit Impl(internal::EsrSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)),
        pipeline(*composition.pipeline),
        environment_service(*composition.environment_service),
        controller(*composition.controller) {
    resolved_config.pipeline_config = composition.runtime_pipeline_config;
    resolved_config.environment_model_config = composition.runtime_environment_model_config;
    resolved_config.runtime_config = composition.runtime_config;
  }

  /**
   * @brief 装配当前周期会话结果。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult BuildCycleResult(const session::EsrCycleInput& input) const {
    EsrCycleResult result;
    result.input_cycle_index = input.cycle_index;
    result.output_frame = BuildOutputFrame();
    result.validation_issues = controller.GetLastValidationIssues();
    result.has_validation_error = session::HasValidationError(result.validation_issues);
    result.executed_this_cycle = controller.ExecutedLatestCycle();
    result.reused_previous_output = controller.ReusedPreviousOutputLatestCycle();
    result.abort_reason = controller.GetLastAbortReason();
    return result;
  }

  /**
   * @brief 仅获取当前周期输出帧，避免构建完整 EsrCycleResult 的开销。
   * @return 当前周期输出帧。
   */
  session::EsrOutputFrame BuildOutputFrame() const {
    if (controller.HasLatestOutputFrame()) {
      return controller.GetLatestOutputFrame();
    }
    return {};
  }

  internal::ResolvedEsrSessionConfig resolved_config{};
  std::unique_ptr<extension::IInterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::EsrController> owned_controller;
  extension::IInterceptPipeline& pipeline;
  environment::IEsrEnvironmentService& environment_service;
  extension::EsrController& controller;
};

EsrSession::EsrSession(EsrSessionConfig config)
    : impl_(new Impl(internal::EsrSessionCompositionRoot::ComposeDefault(config))) {}

EsrSession::EsrSession(EsrSessionConfig config, extension::IInterceptPipeline& pipeline_ref)
    : impl_(new Impl(
          internal::EsrSessionCompositionRoot::ComposeWithPipeline(config, pipeline_ref))) {}

EsrSession::EsrSession(EsrSessionConfig config,
                       environment::IEsrEnvironmentService& environment_service_ref)
    : impl_(new Impl(internal::EsrSessionCompositionRoot::ComposeWithEnvironmentService(
          config, environment_service_ref))) {}

EsrSession::EsrSession(EsrSessionConfig config, extension::EsrController& controller_ref)
    : impl_(new Impl(
          internal::EsrSessionCompositionRoot::ComposeWithController(config, controller_ref))) {}

EsrSession::EsrSession(EsrSessionConfig config, extension::IInterceptPipeline& pipeline_ref,
                       environment::IEsrEnvironmentService& environment_service_ref,
                       extension::EsrController& controller_ref)
    : impl_(new Impl(internal::EsrSessionCompositionRoot::ComposeAllExternal(
          config, pipeline_ref, environment_service_ref, controller_ref))) {}

EsrSession::~EsrSession() = default;

EsrSession::EsrSession(EsrSession&& other) noexcept = default;

EsrSession& EsrSession::operator=(EsrSession&& other) noexcept = default;

session::EsrOutputFrame EsrSession::Step(const session::EsrCycleInput& input) {
  impl_->controller.RunOnce(input);
  return impl_->BuildOutputFrame();
}

EsrCycleResult EsrSession::StepWithResult(const session::EsrCycleInput& input) {
  impl_->controller.RunOnce(input);
  return impl_->BuildCycleResult(input);
}

void EsrSession::ApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch) {
  (void)ApplyRuntimeConfigWithResult(patch);
}

bool EsrSession::TryApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch) {
  return ApplyRuntimeConfigWithResult(patch).applied;
}

EsrRuntimeConfigApplyResult EsrSession::ApplyRuntimeConfigWithResult(
    const EsrRuntimeConfigPatch& patch) {
  EsrRuntimeConfigApplyResult apply_result;
  const internal::EsrRuntimeConfigResolveResult resolved =
      internal::ResolveEsrRuntimeConfigPatch(impl_->resolved_config, patch);
  apply_result.status = resolved.status;
  apply_result.has_requested_update = resolved.has_requested_update;
  if (!resolved.has_requested_update || !resolved.is_valid) {
    apply_result.applied = false;
    return apply_result;
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
  apply_result.applied = true;
  return apply_result;
}

// ── EsrSessionFactory ──────────────────────────────────────────────────────

EsrSession EsrSessionFactory::Create(EsrSessionConfig config) {
  return EsrSession(std::move(config));
}

EsrSession EsrSessionFactory::CreateWithPipeline(EsrSessionConfig config,
                                                 extension::IInterceptPipeline& pipeline_ref) {
  return EsrSession(std::move(config), pipeline_ref);
}

EsrSession EsrSessionFactory::CreateWithEnvironmentService(
    EsrSessionConfig config, environment::IEsrEnvironmentService& environment_service_ref) {
  return EsrSession(std::move(config), environment_service_ref);
}

EsrSession EsrSessionFactory::CreateWithController(EsrSessionConfig config,
                                                   extension::EsrController& controller_ref) {
  return EsrSession(std::move(config), controller_ref);
}

EsrSession EsrSessionFactory::CreateWithAll(
    EsrSessionConfig config, extension::IInterceptPipeline& pipeline_ref,
    environment::IEsrEnvironmentService& environment_service_ref,
    extension::EsrController& controller_ref) {
  return EsrSession(std::move(config), pipeline_ref, environment_service_ref, controller_ref);
}

}  // namespace session

}  // namespace electronic_surveillance_radar
