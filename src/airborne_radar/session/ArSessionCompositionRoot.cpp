#include "airborne_radar/session/ArSessionCompositionRoot.h"

#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/runtime/ArController.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/session/MutableArContext.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar {
namespace session {
namespace {

config::ArSessionConfig BuildRuntimeSessionConfig(const ArSessionComposition& composition) {
  config::ArSessionConfig config;
  config.hardware = composition.runtime_hardware;
  config.mission = composition.runtime_mission;
  config.policy = composition.runtime_policy;
  config.environment.scenario_config = composition.runtime_environment_scenario_config;
  return config;
}

ArSessionComposition BuildCompositionBase(const config::ArSessionConfig& config) {
  ArSessionComposition composition;
  composition.runtime_hardware = config.hardware;
  composition.runtime_mission = config.mission;
  composition.runtime_policy = config.policy;
  composition.runtime_environment_scenario_config = config.environment.scenario_config;
  return composition;
}

}  // namespace

ArSessionComposition ArSessionCompositionRoot::ComposeDefault(
    const config::ArSessionConfig& config) {
  ArSessionComposition composition = BuildCompositionBase(config);
  composition.owned_ar_context.reset(new MutableArContext());
  const config::execution::InternalExecutionConfig runtime_execution_config =
      config::mapping::MapSessionToExecution(BuildRuntimeSessionConfig(composition));
  composition.owned_signal_pipeline.reset(
      new signal::pipeline::SignalPipeline(runtime_execution_config));
  composition.owned_environment_service.reset(new environment::EnvironmentService(
      composition.runtime_environment_scenario_config));
  composition.owned_controller.reset(new extension::ArController(
      *composition.owned_ar_context, *composition.owned_signal_pipeline,
      *composition.owned_environment_service, composition.runtime_policy.decision_control));
  composition.ar_context = composition.owned_ar_context.get();
  composition.signal_pipeline = composition.owned_signal_pipeline.get();
  composition.environment_service = composition.owned_environment_service.get();
  composition.controller = composition.owned_controller.get();
  return composition;
}

}  // namespace session
}  // namespace airborne_radar
