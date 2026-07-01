#include "airborne_radar/session/RadarSessionCompositionRoot.h"

#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"
#include "airborne_radar/runtime/RadarController.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/session/MutableRadarContext.h"
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
  config.environment.jamming_sensitivity_profile = composition.runtime_jamming_sensitivity_profile;
  return config;
}

ArSessionComposition BuildCompositionBase(const config::ArSessionConfig& config) {
  ArSessionComposition composition;
  composition.runtime_hardware = config.hardware;
  composition.runtime_mission = config.mission;
  composition.runtime_policy = config.policy;
  composition.runtime_environment_scenario_config = config.environment.scenario_config;
  composition.runtime_jamming_sensitivity_profile = config.environment.jamming_sensitivity_profile;
  return composition;
}

bool SyncPipelineConfig(ArSessionComposition* composition) {
  if (composition == nullptr || composition->signal_pipeline == nullptr) {
    return false;
  }
  const config::ArSessionConfig runtime_session_config =
      BuildRuntimeSessionConfig(*composition);
  return composition->signal_pipeline->UpdateConfig(runtime_session_config);
}

void SyncEnvironmentModelConfig(ArSessionComposition* composition) {
  if (composition == nullptr || composition->environment_service == nullptr) {
    return;
  }
  composition->environment_service->UpdateModelConfig(
      config::BuildModelConfigFromScenario(composition->runtime_environment_scenario_config));
}

void SyncEnvironmentJammingThreshold(ArSessionComposition* composition) {
  if (composition == nullptr || composition->environment_service == nullptr) {
    return;
  }
  composition->environment_service->SetJammingSensitivityProfile(
      composition->runtime_jamming_sensitivity_profile);
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
      config::BuildModelConfigFromScenario(composition.runtime_environment_scenario_config)));
  composition.owned_controller.reset(new extension::ArController(
      *composition.owned_ar_context, *composition.owned_signal_pipeline,
      *composition.owned_environment_service));
  composition.ar_context = composition.owned_ar_context.get();
  composition.signal_pipeline = composition.owned_signal_pipeline.get();
  composition.environment_service = composition.owned_environment_service.get();
  composition.controller = composition.owned_controller.get();
  SyncEnvironmentJammingThreshold(&composition);
  return composition;
}

ArSessionComposition ArSessionCompositionRoot::ComposeWithDecisionEngine(
    const config::ArSessionConfig& config,
    session::ITacticalDecisionEngine& decision_engine) {
  ArSessionComposition composition = BuildCompositionBase(config);
  composition.owned_ar_context.reset(new MutableArContext());
  const config::execution::InternalExecutionConfig runtime_execution_config =
      config::mapping::MapSessionToExecution(BuildRuntimeSessionConfig(composition));
  composition.owned_signal_pipeline.reset(
      new signal::pipeline::SignalPipeline(runtime_execution_config));
  composition.owned_environment_service.reset(new environment::EnvironmentService(
      config::BuildModelConfigFromScenario(composition.runtime_environment_scenario_config)));
  composition.owned_controller.reset(new extension::ArController(
      *composition.owned_ar_context, *composition.owned_signal_pipeline, decision_engine,
      *composition.owned_environment_service));
  composition.ar_context = composition.owned_ar_context.get();
  composition.signal_pipeline = composition.owned_signal_pipeline.get();
  composition.environment_service = composition.owned_environment_service.get();
  composition.controller = composition.owned_controller.get();
  SyncEnvironmentJammingThreshold(&composition);
  return composition;
}

}  // namespace session
}  // namespace airborne_radar
