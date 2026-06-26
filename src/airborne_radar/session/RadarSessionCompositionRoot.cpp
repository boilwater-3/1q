#include "airborne_radar/session/RadarSessionCompositionRoot.h"

#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "airborne_radar/runtime/RadarController.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/session/MutableRadarContext.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar {
namespace session {
namespace {

config::RadarSessionConfig BuildRuntimeSessionConfig(const RadarSessionComposition& composition) {
  config::RadarSessionConfig config;
  config.hardware = composition.runtime_hardware;
  config.mission = composition.runtime_mission;
  config.policy = composition.runtime_policy;
  config.environment.scenario_config = composition.runtime_environment_scenario_config;
  config.environment.jamming_sensitivity_profile = composition.runtime_jamming_sensitivity_profile;
  return config;
}

RadarSessionComposition BuildCompositionBase(const config::RadarSessionConfig& config) {
  RadarSessionComposition composition;
  composition.runtime_hardware = config.hardware;
  composition.runtime_mission = config.mission;
  composition.runtime_policy = config.policy;
  composition.runtime_environment_scenario_config = config.environment.scenario_config;
  composition.runtime_jamming_sensitivity_profile = config.environment.jamming_sensitivity_profile;
  return composition;
}

bool SyncPipelineConfig(RadarSessionComposition* composition) {
  if (composition == nullptr || composition->signal_pipeline == nullptr) {
    return false;
  }
  const config::RadarSessionConfig runtime_session_config =
      BuildRuntimeSessionConfig(*composition);
  return composition->signal_pipeline->UpdateConfig(runtime_session_config);
}

void SyncEnvironmentModelConfig(RadarSessionComposition* composition) {
  if (composition == nullptr || composition->environment_service == nullptr) {
    return;
  }
  composition->environment_service->UpdateModelConfig(
      config::BuildModelConfigFromScenario(composition->runtime_environment_scenario_config));
}

void SyncEnvironmentJammingThreshold(RadarSessionComposition* composition) {
  if (composition == nullptr || composition->environment_service == nullptr) {
    return;
  }
  composition->environment_service->SetJammingSensitivityProfile(
      composition->runtime_jamming_sensitivity_profile);
}

}  // namespace

RadarSessionComposition RadarSessionCompositionRoot::ComposeDefault(
    const config::RadarSessionConfig& config) {
  RadarSessionComposition composition = BuildCompositionBase(config);
  composition.owned_radar_context.reset(new MutableRadarContext());
  const config::execution::InternalExecutionConfig runtime_execution_config =
      config::mapping::MapSessionToExecution(BuildRuntimeSessionConfig(composition));
  composition.owned_signal_pipeline.reset(
      new signal::pipeline::SignalPipeline(runtime_execution_config));
  composition.owned_environment_service.reset(new environment::EnvironmentService(
      config::BuildModelConfigFromScenario(composition.runtime_environment_scenario_config)));
  composition.owned_controller.reset(new extension::RadarController(
      *composition.owned_radar_context, *composition.owned_signal_pipeline,
      *composition.owned_environment_service));
  composition.radar_context = composition.owned_radar_context.get();
  composition.signal_pipeline = composition.owned_signal_pipeline.get();
  composition.environment_service = composition.owned_environment_service.get();
  composition.controller = composition.owned_controller.get();
  SyncEnvironmentJammingThreshold(&composition);
  return composition;
}

RadarSessionComposition RadarSessionCompositionRoot::ComposeWithDecisionEngine(
    const config::RadarSessionConfig& config,
    extension::ITacticalDecisionEngine& decision_engine) {
  RadarSessionComposition composition = BuildCompositionBase(config);
  composition.owned_radar_context.reset(new MutableRadarContext());
  const config::execution::InternalExecutionConfig runtime_execution_config =
      config::mapping::MapSessionToExecution(BuildRuntimeSessionConfig(composition));
  composition.owned_signal_pipeline.reset(
      new signal::pipeline::SignalPipeline(runtime_execution_config));
  composition.owned_environment_service.reset(new environment::EnvironmentService(
      config::BuildModelConfigFromScenario(composition.runtime_environment_scenario_config)));
  composition.owned_controller.reset(new extension::RadarController(
      *composition.owned_radar_context, *composition.owned_signal_pipeline, decision_engine,
      *composition.owned_environment_service));
  composition.radar_context = composition.owned_radar_context.get();
  composition.signal_pipeline = composition.owned_signal_pipeline.get();
  composition.environment_service = composition.owned_environment_service.get();
  composition.controller = composition.owned_controller.get();
  SyncEnvironmentJammingThreshold(&composition);
  return composition;
}

}  // namespace session
}  // namespace airborne_radar
