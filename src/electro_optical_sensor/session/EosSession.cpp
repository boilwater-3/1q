#include "1q/electro_optical_sensor/session/EosSession.h"

#include <cstdlib>
#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"
#include "electro_optical_sensor/runtime/EosCycleOrchestrator.h"

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
  explicit Impl(internal::EosSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_controller(std::move(composition.owned_controller)),
        pipeline(RequireCompositionDependency(composition.pipeline, "pipeline")),
        controller(RequireCompositionDependency(composition.controller, "controller")),
        cycle_orchestrator(composition.runtime_config, composition.pipeline_config,
                           composition.initial_reset_scan_phase, pipeline, controller) {
  }

  std::unique_ptr<::electro_optical_sensor::extension::IEosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;
  ::electro_optical_sensor::extension::IEosPipeline& pipeline;
  extension::EosController& controller;
  runtime::session::internal::EosCycleOrchestrator cycle_orchestrator;
};

EosSession::EosSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EosSession::~EosSession() noexcept = default;
EosSession::EosSession(EosSession&&) noexcept = default;
EosSession& EosSession::operator=(EosSession&&) noexcept = default;

EosSession EosSessionFactory::Create(const EosSessionConfig& config) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeDefault(config))));
}

EosSession EosSessionFactory::CreateWithPipeline(
    const EosSessionConfig& config, ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeWithPipeline(config, pipeline))));
}

EosSession EosSessionFactory::CreateWithEnvironmentService(
    const EosSessionConfig& config,
    environment::IEosEnvironmentService& environment_service) {
  return EosSession(std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
      internal::EosSessionCompositionRoot::ComposeWithEnvironmentService(config,
                                                                         environment_service))));
}

EosSession EosSessionFactory::CreateWithController(
    const EosSessionConfig& config, extension::EosController& controller) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeWithController(config, controller))));
}

output::EosOutputFrame EosSession::Step(const EosCycleInput& input) {
  return StepWithResult(input).output_frame;
}

::electro_optical_sensor::session::EosCycleResult EosSession::StepWithResult(const EosCycleInput& input) {
  return impl_->cycle_orchestrator.Step(input);
}

void EosSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  impl_->cycle_orchestrator.ApplyRuntimeConfig(patch);
}

}  // namespace session
}  // namespace electro_optical_sensor
