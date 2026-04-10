#include "electro_optical_sensor/runtime/EosCycleOrchestrator.h"

#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "common/logging/ProjectLog.h"
#include "electro_optical_sensor/session/EosPipelineConfigMapper.h"
#include "electro_optical_sensor/session/EosRuntimeConfigResolver.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

EosCycleOrchestrator::EosCycleOrchestrator(
    const EosSessionConfig& config,
    const ::electro_optical_sensor::extension::EosPipelineConfig& pipeline_config,
    bool initial_reset_scan_phase,
    ::electro_optical_sensor::extension::IEosPipeline& pipeline,
    ::electro_optical_sensor::extension::EosController& controller)
    : runtime_config_(config), pipeline_(pipeline), controller_(controller) {
  pipeline_.UpdateConfig(pipeline_config, initial_reset_scan_phase);
}

EosCycleResult EosCycleOrchestrator::Step(const EosCycleInput& input) {
  controller_.RunOnce(input);
  return controller_.BuildCycleResult(input);
}

void EosCycleOrchestrator::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  const ::electro_optical_sensor::session::internal::EosRuntimeConfigResolveResult resolved =
      ResolveEosRuntimeConfigPatch(runtime_config_, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return;
  }
  runtime_config_ = resolved.next_config;
  pipeline_.UpdateConfig(BuildEosPipelineConfig(runtime_config_), resolved.reset_scan_phase);
}

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
