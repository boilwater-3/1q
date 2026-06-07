/**
 * @file EosSessionCompositionRoot.cpp
 * @brief 实现 EOS 会话组合根，统一装配 pipeline 与 controller 依赖。
 */

#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"

#include <cstdlib>
#include <memory>
#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {

namespace {

EosSessionComposition BuildInitialCompositionRuntime(const config::EosSessionConfig& config,
                                                     EosSessionComposition composition) {
  composition.internal_config = runtime::session::MapSessionToInternal(config);
  composition.initial_reset_scan_phase = true;
  return composition;
}

template <typename T>
T& RequireComposedDependency(T* ptr, const char* dependency_name) {
  if (ptr != nullptr) {
    return *ptr;
  }
  PROJECT_LOG_ERROR("[EosSessionCompositionRoot] Dependency '{}' is null.", dependency_name);
  std::abort();
}

EosSessionComposition FinalizeComposition(EosSessionComposition composition) {
  static_cast<void>(RequireComposedDependency(composition.pipeline, "pipeline"));
  static_cast<void>(RequireComposedDependency(composition.controller, "controller"));
  return composition;
}

std::shared_ptr<environment::IEosEnvironmentService> MakeNonOwningEnvironmentServiceHandle(
    environment::IEosEnvironmentService& environment_service) {
  return std::shared_ptr<environment::IEosEnvironmentService>(
      &environment_service, [](environment::IEosEnvironmentService*) {});
}

EosSessionComposition MakeCompositionWithOwnedPipeline(
    std::unique_ptr<extension::IEosPipeline> owned_pipeline,
    const config::EosSessionConfig& config) {
  EosSessionComposition composition;
  composition.owned_pipeline = std::move(owned_pipeline);
  composition.owned_controller.reset(
      new extension::EosController(*composition.owned_pipeline));
  composition.pipeline = composition.owned_pipeline.get();
  composition.controller = composition.owned_controller.get();
  return BuildInitialCompositionRuntime(config, std::move(composition));
}

EosSessionComposition MakeCompositionWithExternalPipeline(
    extension::IEosPipeline& pipeline,
    const config::EosSessionConfig& config) {
  EosSessionComposition composition;
  composition.owned_controller.reset(new extension::EosController(pipeline));
  composition.pipeline = &pipeline;
  composition.controller = composition.owned_controller.get();
  composition = BuildInitialCompositionRuntime(config, std::move(composition));
  // 通过公开接口更新（支持外部 IEosPipeline 实现）
  composition.pipeline->UpdateConfig(
      runtime::session::InternalToPipelineConfig(composition.internal_config),
      composition.initial_reset_scan_phase);
  return composition;
}

EosSessionComposition ComposeWithOwnedPipeline(
    std::unique_ptr<extension::IEosPipeline> owned_pipeline,
    const config::EosSessionConfig& config) {
  return FinalizeComposition(MakeCompositionWithOwnedPipeline(std::move(owned_pipeline), config));
}

EosSessionComposition ComposeWithExternalPipeline(extension::IEosPipeline& pipeline,
                                                  const config::EosSessionConfig& config) {
  return FinalizeComposition(MakeCompositionWithExternalPipeline(pipeline, config));
}

}  // namespace

EosSessionComposition EosSessionCompositionRoot::ComposeDefault(
    const config::EosSessionConfig& config) {
  return ComposeWithOwnedPipeline(
      std::unique_ptr<extension::IEosPipeline>(
          new signal::pipeline::EosPipeline(runtime::session::MapSessionToInternal(
              config))),
      config);
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithPipeline(
    const config::EosSessionConfig& config,
    ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  return ComposeWithExternalPipeline(pipeline, config);
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithEnvironmentService(
    const config::EosSessionConfig& config,
    environment::IEosEnvironmentService& environment_service) {
  return ComposeWithOwnedPipeline(
      std::unique_ptr<extension::IEosPipeline>(new signal::pipeline::EosPipeline(
          runtime::session::MapSessionToInternal(config),
          MakeNonOwningEnvironmentServiceHandle(environment_service))),
      config);
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithController(
    const config::EosSessionConfig& config,
    extension::EosController& controller) {
  EosSessionComposition composition;
  composition = BuildInitialCompositionRuntime(config, std::move(composition));
  composition.pipeline = &controller.GetPipeline();
  composition.controller = &controller;
  composition.pipeline->UpdateConfig(
      runtime::session::InternalToPipelineConfig(composition.internal_config),
      composition.initial_reset_scan_phase);
  return FinalizeComposition(std::move(composition));
}

EosSessionComposition EosSessionCompositionRoot::ComposeAllExternal(
    const config::EosSessionConfig& config,
    ::electro_optical_sensor::extension::IEosPipeline& pipeline,
    extension::EosController& controller) {
  EosSessionComposition composition;
  composition = BuildInitialCompositionRuntime(config, std::move(composition));
  composition.pipeline = &pipeline;
  composition.controller = &controller;
  composition.pipeline->UpdateConfig(
      runtime::session::InternalToPipelineConfig(composition.internal_config),
      composition.initial_reset_scan_phase);
  return FinalizeComposition(std::move(composition));
}

}  // namespace session
}  // namespace electro_optical_sensor
