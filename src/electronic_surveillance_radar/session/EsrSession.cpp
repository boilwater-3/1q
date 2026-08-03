#include "1q/electronic_surveillance_radar/session/EsrSession.h"

#include <utility>

#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"
#include "electronic_surveillance_radar/runtime/EsrController.h"
#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"
#include "electronic_surveillance_radar/session/EsrSessionCompositionRoot.h"

namespace electronic_surveillance_radar {
namespace session {

struct EsrSession::Impl {
  explicit Impl(EsrSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_environment_service(std::move(composition.owned_environment_service)),
        owned_controller(std::move(composition.owned_controller)) {
    resolved_config = std::move(composition.execution_config);
  }

  /**
   * @brief 装配当前周期会话结果。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult BuildCycleResult(const session::EsrCycleInput& input) const {
    EsrCycleResult result;
    result.input_cycle_index = input.cycle_index;
    if (Controller().GetLatestCycleStatus() == EsrCycleExecutionStatus::kCompleted &&
        Controller().HasLatestInterceptOutputFrame()) {
      result.output_frame = Controller().GetLatestInterceptOutputFrame();
    }
    result.validation_issues = Controller().GetLastValidationIssues();
    result.has_validation_error = session::HasValidationError(result.validation_issues);
    result.status = Controller().GetLatestCycleStatus();
    result.abort_reason = Controller().GetLastInterceptCycleAbortReason();
    return result;
  }

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult RunCycle(const session::EsrCycleInput& input) {
    const auto pipeline_state = Pipeline().CaptureRuntimeState();
    const auto controller_state = Controller().CaptureRuntimeState();

    Controller().RunOnce(input);

    // 按 docs/common/contract.md「运行期配置提交策略」，ESR 属立即提交类，配置不在
    // session 层回滚。关机由 pipeline 显式上报，且没有推进任何累积状态；保留该
    // 非执行结果；结果 DTO 不回传最近有效输出。其他非 validation abort 才回滚运行态。
    if (Controller().GetLatestCycleStatus() == EsrCycleExecutionStatus::kRejected &&
        Controller().GetLastInterceptCycleAbortReason() !=
            session::EsrPipelineAbortReason::kValidationRejected &&
        Controller().GetLastInterceptCycleAbortReason() !=
            session::EsrPipelineAbortReason::kSensorPoweredOff &&
        Controller().GetLastInterceptCycleAbortReason() !=
            session::EsrPipelineAbortReason::kRfReceiverRejected) {
      const bool pipeline_restored = Pipeline().RestoreRuntimeState(pipeline_state);
      const bool controller_restored = Controller().RestoreRuntimeState(controller_state);
      if (!pipeline_restored || !controller_restored) {
        // 两份快照各自只恢复唯一 owner；任一恢复失败都必须作为结构化内部错误暴露，
        // 不能继续返回恢复前或半恢复状态。
        EsrCycleResult result;
        result.input_cycle_index = input.cycle_index;
        result.status = EsrCycleExecutionStatus::kRejected;
        result.abort_reason = session::EsrPipelineAbortReason::kRuntimeStateRestoreRejected;
        return result;
      }
    }
    return BuildCycleResult(input);
  }

  EsrInternalExecutionConfig resolved_config{};
  std::unique_ptr<pipeline::InterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::EsrController> owned_controller;

  pipeline::InterceptPipeline& Pipeline() const { return *owned_pipeline; }
  environment::IEsrEnvironmentService& EnvironmentService() const {
    return *owned_environment_service;
  }
  extension::EsrController& Controller() const { return *owned_controller; }
};

EsrSession::EsrSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EsrSession::EsrSession()
    : impl_(new Impl(EsrSessionCompositionRoot::ComposeDefault(config::EsrSessionConfig{}))) {}

EsrSession::~EsrSession() = default;

EsrSession::EsrSession(EsrSession&& other) noexcept = default;

EsrSession& EsrSession::operator=(EsrSession&& other) noexcept = default;

session::EsrOutputFrame EsrSession::Step(const session::EsrCycleInput& input) {
  return impl_->RunCycle(input).output_frame;
}

EsrCycleResult EsrSession::StepWithResult(const session::EsrCycleInput& input) {
  return impl_->RunCycle(input);
}

void EsrSession::ApplyRuntimeConfig(const config::EsrRuntimeConfigPatch& patch) {
  (void)ApplyRuntimeConfigWithResult(patch);
}

bool EsrSession::TryApplyRuntimeConfig(const config::EsrRuntimeConfigPatch& patch) {
  return ApplyRuntimeConfigWithResult(patch).applied;
}

EsrRuntimeConfigApplyResult EsrSession::ApplyRuntimeConfigWithResult(
    const config::EsrRuntimeConfigPatch& patch) {
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
  if (resolved.runtime_config_changed || resolved.pipeline_config_changed) {
    // 写路径直接吃 internal config（与 EOS/AR 一致），避免 internal→extension→internal
    // 往返。RunCycle 的 per-cycle MutableEsrContext 投影仍在 RunCycle 内进行。
    impl_->Pipeline().UpdateConfig(impl_->resolved_config);
  }
  if (resolved.environment_model_config_changed) {
    impl_->EnvironmentService().UpdateModelConfig(impl_->resolved_config.environment);
  }
  apply_result.applied = true;
  return apply_result;
}

// ── EsrSession static factory ──────────────────────────────────────────────────────

EsrSession EsrSession::Create(const config::EsrSessionConfig& config) {
  return EsrSession(std::unique_ptr<EsrSession::Impl>(
      new EsrSession::Impl(EsrSessionCompositionRoot::ComposeDefault(config))));
}

EsrSession EsrSession::CreateWithDiagnostics(const config::EsrSessionConfig& config,
                                             config::ValidationIssueList* issues) {
  const config::ValidationIssueList found = config::ValidateEsrSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

}  // namespace session

}  // namespace electronic_surveillance_radar
