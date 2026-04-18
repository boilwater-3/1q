#include "airborne_radar/session/RadarSessionCompositionRoot.h"

#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "airborne_radar/session/MutableRadarContext.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"

namespace airborne_radar {
namespace session {
namespace internal {
namespace {

config::PipelineConfig BuildPipelineConfig(
    const RadarSessionConfig& session_config) {
  return BuildLegacyPipelineConfig(session_config);
}

RadarSessionComposition BuildCompositionBase(const RadarSessionConfig& config) {
  RadarSessionComposition composition;
  composition.runtime_pipeline_config = BuildPipelineConfig(config);
  composition.runtime_environment_scenario_config = config.environment.scenario_config;
  composition.runtime_jamming_sensitivity_profile = config.environment.jamming_sensitivity_profile;
  return composition;
}

void SyncPipelineConfig(RadarSessionComposition* composition) {
  if (composition == nullptr || composition->signal_pipeline == nullptr) {
    return;
  }
  const bool accepted =
      composition->signal_pipeline->UpdateConfig(composition->runtime_pipeline_config);
  (void)accepted;
}

void SyncEnvironmentModelConfig(RadarSessionComposition* composition) {
  if (composition == nullptr || composition->environment_service == nullptr) {
    return;
  }
  composition->environment_service->UpdateModelConfig(environment::BuildModelConfigFromScenario(
      composition->runtime_environment_scenario_config));
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
    const RadarSessionConfig& config) {
  RadarSessionComposition composition = BuildCompositionBase(config);
  composition.owned_radar_context.reset(new MutableRadarContext());
  composition.owned_signal_pipeline.reset(
      new signal::pipeline::SignalPipeline(composition.runtime_pipeline_config));
  composition.owned_environment_service.reset(
      new environment::EnvironmentService(environment::BuildModelConfigFromScenario(
          composition.runtime_environment_scenario_config)));
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

RadarSessionComposition RadarSessionCompositionRoot::ComposeWithSignalPipeline(
    const RadarSessionConfig& config, extension::ISignalPipeline& signal_pipeline) {
  RadarSessionComposition composition = BuildCompositionBase(config);
  composition.owned_radar_context.reset(new MutableRadarContext());
  composition.owned_environment_service.reset(
      new environment::EnvironmentService(environment::BuildModelConfigFromScenario(
          composition.runtime_environment_scenario_config)));
  composition.owned_controller.reset(new extension::RadarController(
      *composition.owned_radar_context, signal_pipeline, *composition.owned_environment_service));
  composition.radar_context = composition.owned_radar_context.get();
  composition.signal_pipeline = &signal_pipeline;
  composition.environment_service = composition.owned_environment_service.get();
  composition.controller = composition.owned_controller.get();
  SyncPipelineConfig(&composition);
  SyncEnvironmentJammingThreshold(&composition);
  return composition;
}

RadarSessionComposition RadarSessionCompositionRoot::ComposeWithEnvironmentService(
    const RadarSessionConfig& config, environment::IEnvironmentService& environment_service) {
  RadarSessionComposition composition = BuildCompositionBase(config);
  composition.owned_radar_context.reset(new MutableRadarContext());
  composition.owned_signal_pipeline.reset(
      new signal::pipeline::SignalPipeline(composition.runtime_pipeline_config));
  composition.owned_controller.reset(new extension::RadarController(
      *composition.owned_radar_context, *composition.owned_signal_pipeline, environment_service));
  composition.radar_context = composition.owned_radar_context.get();
  composition.signal_pipeline = composition.owned_signal_pipeline.get();
  composition.environment_service = &environment_service;
  composition.controller = composition.owned_controller.get();
  SyncEnvironmentModelConfig(&composition);
  SyncEnvironmentJammingThreshold(&composition);
  return composition;
}

RadarSessionComposition RadarSessionCompositionRoot::ComposeWithController(
    const RadarSessionConfig& config, extension::RadarController& controller) {
  RadarSessionComposition composition = BuildCompositionBase(config);
  composition.radar_context = &controller.GetRadarContext();
  composition.signal_pipeline = &controller.GetSignalPipeline();
  composition.environment_service = &controller.GetEnvironmentService();
  composition.controller = &controller;
  SyncPipelineConfig(&composition);
  SyncEnvironmentModelConfig(&composition);
  SyncEnvironmentJammingThreshold(&composition);
  return composition;
}

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar
