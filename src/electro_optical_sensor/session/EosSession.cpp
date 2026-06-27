#include "1q/electro_optical_sensor/session/EosSession.h"

#include <cstdlib>
#include <utility>

#include "electro_optical_sensor/runtime/EosController.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"
#include "electro_optical_sensor/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {

namespace {

/**
 * @brief 校验组合根输出的依赖指针非空并返回引用。
 * @param[in] ptr 待校验指针。
 * @param[in] dependency_name 依赖名称，用于日志与故障定位。
 * @return 非空依赖引用。
 * @warning 若组合结果违反约束且传入空指针，将终止进程以避免未定义行为。
 */
template <typename T>
T& RequireCompositionDependency(T* ptr, const char* dependency_name) {
  if (ptr != nullptr) {
    return *ptr;
  }
  PROJECT_LOG_ERROR("[EosSession] Composition dependency '{}' is null.", dependency_name);
  std::abort();
}

}  // namespace

struct EosSession::Impl {
  explicit Impl(EosSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_controller(std::move(composition.owned_controller)),
        controller(RequireCompositionDependency(owned_controller.get(), "controller")),
        internal_config_(composition.internal_config) {
  }

  std::unique_ptr<signal::pipeline::EosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;
  extension::EosController& controller;
  config::execution::EosInternalExecutionConfig internal_config_;
};

EosSession::EosSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EosSession::EosSession()
    : impl_(new Impl(EosSessionCompositionRoot::ComposeDefault(config::EosSessionConfig{}))) {}

EosSession::~EosSession() noexcept = default;
EosSession::EosSession(EosSession&&) noexcept = default;
EosSession& EosSession::operator=(EosSession&&) noexcept = default;

EosSession EosSession::Create(const config::EosSessionConfig& config) {
  return EosSession(std::unique_ptr<EosSession::Impl>(
      new EosSession::Impl(EosSessionCompositionRoot::ComposeDefault(config))));
}

EosSession EosSession::TryCreate(const config::EosSessionConfig& config,
                                 config::ValidationIssueList* issues) {
  const config::ValidationIssueList found = config::ValidateEosSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

session::EosOutputFrame EosSession::Step(const EosCycleInput& input) {
  return impl_->controller.RunOnce(input), impl_->controller.BuildCycleResult(input).output_frame;
}

::electro_optical_sensor::session::EosCycleResult EosSession::StepWithResult(
    const EosCycleInput& input) {
  impl_->controller.RunOnce(input);
  return impl_->controller.BuildCycleResult(input);
}

void EosSession::ApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool EosSession::TryApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch) {
  const ::electro_optical_sensor::runtime::session::EosRuntimeConfigResolveResult
      resolved = ::electro_optical_sensor::runtime::session::ResolveEosRuntimeConfigPatch(
          impl_->internal_config_, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return false;
  }
  impl_->internal_config_ = resolved.next_config;
  if (impl_->owned_pipeline != nullptr) {
    impl_->owned_pipeline->ApplyInternalConfig(
        impl_->internal_config_, resolved.reset_scan_phase);
  }
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
