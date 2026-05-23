#include "1q/electro_optical_sensor/session/EosSession.h"

#include <cstdlib>
#include <utility>

#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"
#include "1q/electro_optical_sensor/session/EosSessionFactory.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"

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
        pipeline(RequireCompositionDependency(composition.pipeline, "pipeline")),
        controller(RequireCompositionDependency(composition.controller, "controller")),
        runtime_config_(composition.runtime_config) {
    pipeline.UpdateConfig(composition.pipeline_config,
                          composition.initial_reset_scan_phase);
  }

  std::unique_ptr<::electro_optical_sensor::extension::IEosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;
  ::electro_optical_sensor::extension::IEosPipeline& pipeline;
  extension::EosController& controller;
  ::electro_optical_sensor::config::EosSessionConfig runtime_config_;
};

EosSession::EosSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EosSession::EosSession()
    : impl_(new Impl(EosSessionCompositionRoot::ComposeDefault(config::EosSessionConfig{}))) {}

EosSession::~EosSession() noexcept = default;
EosSession::EosSession(EosSession&&) noexcept = default;
EosSession& EosSession::operator=(EosSession&&) noexcept = default;

EosSession EosSessionFactory::Create(const config::EosSessionConfig& config) {
  return EosSession(std::unique_ptr<EosSession::Impl>(
      new EosSession::Impl(EosSessionCompositionRoot::ComposeDefault(config))));
}

EosSession EosSessionFactory::CreateWithPipeline(
    const config::EosSessionConfig& config, ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  return EosSession(std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
      EosSessionCompositionRoot::ComposeWithPipeline(config, pipeline))));
}

EosSession EosSessionFactory::CreateWithEnvironmentService(
    const config::EosSessionConfig& config, environment::IEosEnvironmentService& environment_service) {
  return EosSession(std::unique_ptr<EosSession::Impl>(
      new EosSession::Impl(EosSessionCompositionRoot::ComposeWithEnvironmentService(
          config, environment_service))));
}

EosSession EosSessionFactory::CreateWithController(const config::EosSessionConfig& config,
                                                   extension::EosController& controller) {
  return EosSession(std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
      EosSessionCompositionRoot::ComposeWithController(config, controller))));
}

EosSession EosSessionFactory::CreateWithAll(const config::EosSessionConfig& config,
                                             extension::IEosPipeline& pipeline,
                                             extension::EosController& controller) {
  return EosSession(std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
      EosSessionCompositionRoot::ComposeAllExternal(config, pipeline, controller))));
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
          impl_->runtime_config_, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return false;
  }
  impl_->runtime_config_ = resolved.next_config;
  impl_->pipeline.UpdateConfig(
      ::electro_optical_sensor::runtime::session::BuildEosPipelineConfig(
          impl_->runtime_config_),
      resolved.reset_scan_phase);
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
