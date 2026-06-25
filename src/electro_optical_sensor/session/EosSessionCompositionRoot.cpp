/**
 * @file EosSessionCompositionRoot.cpp
 * @brief 实现 EOS 会话组合根，统一装配 pipeline 与 controller 依赖。
 * @note 管线与环境服务已完全内部化，仅支持 ComposeDefault。
 */

#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"

#include <cstdlib>
#include <memory>
#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {

namespace {

template <typename T>
T& RequireComposedDependency(std::unique_ptr<T>& ptr, const char* dependency_name) {
  if (ptr != nullptr) {
    return *ptr;
  }
  PROJECT_LOG_ERROR("[EosSessionCompositionRoot] Dependency '{}' is null.", dependency_name);
  std::abort();
}

void FinalizeComposition(EosSessionComposition& composition) {
  static_cast<void>(RequireComposedDependency(composition.owned_pipeline, "pipeline"));
  static_cast<void>(RequireComposedDependency(composition.owned_controller, "controller"));
}

}  // namespace

EosSessionComposition EosSessionCompositionRoot::ComposeDefault(
    const config::EosSessionConfig& config) {
  auto internal_config = runtime::session::MapSessionToInternal(config);
  EosSessionComposition composition;
  composition.internal_config = internal_config;
  composition.owned_pipeline.reset(
      new signal::pipeline::EosPipeline(internal_config));
  composition.owned_controller.reset(
      new extension::EosController(*composition.owned_pipeline));
  composition.initial_reset_scan_phase = true;
  FinalizeComposition(composition);
  return composition;
}

}  // namespace session
}  // namespace electro_optical_sensor
