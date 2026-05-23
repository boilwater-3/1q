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
  explicit Impl(EsrSessionComposition composition)
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
    if (controller.HasLatestInterceptOutputFrame()) {
      result.output_frame = controller.GetLatestInterceptOutputFrame();
    }
    result.validation_issues = controller.GetLastValidationIssues();
    result.has_validation_error = session::HasValidationError(result.validation_issues);
    result.executed_this_cycle = controller.ExecutedLatestCycle();
    result.reused_previous_output = controller.ReusedPreviousInterceptOutputLatestCycle();
    result.abort_reason = controller.GetLastInterceptCycleAbortReason();
    return result;
  }

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult RunCycle(const session::EsrCycleInput& input) {
    const auto pipeline_state = pipeline.CaptureRuntimeState();
    const auto controller_state = controller.CaptureRuntimeState();

    controller.RunOnce(input);

    if (!controller.ExecutedLatestCycle() &&
        controller.GetLastInterceptCycleAbortReason() !=
            extension::EsrPipelineAbortReason::kValidationRejected) {
      pipeline.RestoreRuntimeState(pipeline_state);
      controller.RestoreRuntimeState(controller_state);
    }
    return BuildCycleResult(input);
  }

  ResolvedEsrSessionConfig resolved_config{};
  std::unique_ptr<extension::IInterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::EsrController> owned_controller;
  extension::IInterceptPipeline& pipeline;
  environment::IEsrEnvironmentService& environment_service;
  extension::EsrController& controller;
};

EsrSession::EsrSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EsrSession::EsrSession()
    : impl_(new Impl(EsrSessionCompositionRoot::ComposeDefault(EsrSessionConfig{}))) {}

EsrSession::~EsrSession() = default;

EsrSession::EsrSession(EsrSession&& other) noexcept = default;

EsrSession& EsrSession::operator=(EsrSession&& other) noexcept = default;

session::EsrOutputFrame EsrSession::Step(const session::EsrCycleInput& input) {
  return impl_->RunCycle(input).output_frame;
}

EsrCycleResult EsrSession::StepWithResult(const session::EsrCycleInput& input) {
  return impl_->RunCycle(input);
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
  const EsrRuntimeConfigResolveResult resolved =
      ResolveEsrRuntimeConfigPatch(impl_->resolved_config, patch);
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
  return EsrSession(std::unique_ptr<EsrSession::Impl>(
      new EsrSession::Impl(EsrSessionCompositionRoot::ComposeDefault(config))));
}

EsrSession EsrSessionFactory::CreateWithPipeline(EsrSessionConfig config,
                                                 extension::IInterceptPipeline& pipeline_ref) {
  return EsrSession(std::unique_ptr<EsrSession::Impl>(
      new EsrSession::Impl(EsrSessionCompositionRoot::ComposeWithPipeline(config, pipeline_ref))));
}

EsrSession EsrSessionFactory::CreateWithEnvironmentService(
    EsrSessionConfig config, environment::IEsrEnvironmentService& environment_service_ref) {
  return EsrSession(std::unique_ptr<EsrSession::Impl>(
      new EsrSession::Impl(EsrSessionCompositionRoot::ComposeWithEnvironmentService(
          config, environment_service_ref))));
}

EsrSession EsrSessionFactory::CreateWithController(EsrSessionConfig config,
                                                   extension::EsrController& controller_ref) {
  return EsrSession(std::unique_ptr<EsrSession::Impl>(
      new EsrSession::Impl(EsrSessionCompositionRoot::ComposeWithController(config, controller_ref))));
}

EsrSession EsrSessionFactory::CreateWithAll(
    EsrSessionConfig config, extension::IInterceptPipeline& pipeline_ref,
    environment::IEsrEnvironmentService& environment_service_ref,
    extension::EsrController& controller_ref) {
  return EsrSession(std::unique_ptr<EsrSession::Impl>(
      new EsrSession::Impl(EsrSessionCompositionRoot::ComposeAllExternal(
          config, pipeline_ref, environment_service_ref, controller_ref))));
}

}  // namespace session

}  // namespace electronic_surveillance_radar
