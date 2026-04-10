/**
 * @file EosSessionCompositionRoot.cpp
 * @brief 实现 EOS 会话组合根，统一装配 pipeline 与 controller 依赖。
 */

#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"

#include <cstdlib>
#include <memory>
#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

namespace {

/**
 * @brief 校验组合结果中的关键依赖非空。
 * @param[in] ptr 待校验指针。
 * @param[in] dependency_name 依赖名称，用于日志与故障定位。
 * @return 非空依赖引用。
 * @warning 若装配输出为空指针，将终止进程避免未定义行为。
 */
template <typename T>
T& RequireComposedDependency(T* ptr, const char* dependency_name) {
  if (ptr != nullptr) {
    return *ptr;
  }
  PROJECT_LOG_ERROR("[EosSessionCompositionRoot] Dependency '{}' is null.", dependency_name);
  std::abort();
}

/**
 * @brief 统一收尾并校验组合结果。
 * @param[in] composition 待校验组合结果。
 * @return 校验后的组合结果。
 */
EosSessionComposition FinalizeComposition(EosSessionComposition composition) {
  static_cast<void>(RequireComposedDependency(composition.pipeline, "pipeline"));
  static_cast<void>(RequireComposedDependency(composition.controller, "controller"));
  return composition;
}

std::shared_ptr<extension::IEosEnvironmentService> MakeNonOwningEnvironmentServiceHandle(
    extension::IEosEnvironmentService& environment_service) {
  return std::shared_ptr<extension::IEosEnvironmentService>(
      &environment_service, [](extension::IEosEnvironmentService*) {});
}

EosSessionComposition MakeCompositionWithOwnedPipeline(
    std::unique_ptr<extension::IEosPipeline> owned_pipeline) {
  EosSessionComposition composition;
  composition.owned_pipeline = std::move(owned_pipeline);
  composition.owned_controller.reset(new extension::EosController(*composition.owned_pipeline));
  composition.pipeline = composition.owned_pipeline.get();
  composition.controller = composition.owned_controller.get();
  return composition;
}

EosSessionComposition MakeCompositionWithExternalPipeline(
    extension::IEosPipeline& pipeline) {
  EosSessionComposition composition;
  composition.owned_controller.reset(new extension::EosController(pipeline));
  composition.pipeline = &pipeline;
  composition.controller = composition.owned_controller.get();
  return composition;
}

}  // namespace

EosSessionComposition EosSessionCompositionRoot::ComposeDefault() {
  return FinalizeComposition(MakeCompositionWithOwnedPipeline(std::unique_ptr<extension::IEosPipeline>(
  new core::pipeline::EosPipeline(::electro_optical_sensor::extension::EosPipelineConfig{}))));
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithPipeline(
    ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  return FinalizeComposition(MakeCompositionWithExternalPipeline(pipeline));
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithEnvironmentService(
    extension::IEosEnvironmentService& environment_service) {
  return FinalizeComposition(MakeCompositionWithOwnedPipeline(std::unique_ptr<extension::IEosPipeline>(
      new core::pipeline::EosPipeline(::electro_optical_sensor::extension::EosPipelineConfig{},
                                       MakeNonOwningEnvironmentServiceHandle(
                                           environment_service)))));
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithController(
    extension::EosController& controller) {
  EosSessionComposition composition;
  composition.pipeline = &controller.GetPipeline();
  composition.controller = &controller;
  return FinalizeComposition(std::move(composition));
}

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
