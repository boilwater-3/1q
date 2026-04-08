#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/airborne_radar/core/session/RadarSession.h"

namespace airborne_radar {
namespace core {
namespace session {
namespace internal {

struct RadarSessionComposition {
  signal::config::SignalPipelineConfig runtime_signal_pipeline_config{};
  environment::EnvironmentModelConfig runtime_environment_model_config{};
  float runtime_jamming_detection_threshold_db{6.0f};

  std::unique_ptr<context::IRadarContext> owned_radar_context;
  std::unique_ptr<signal::pipeline::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<environment::IEnvironmentService> owned_environment_service;
  std::unique_ptr<controller::RadarController> owned_controller;

  context::IRadarContext* radar_context{nullptr};
  signal::pipeline::ISignalPipeline* signal_pipeline{nullptr};
  environment::IEnvironmentService* environment_service{nullptr};
  controller::RadarController* controller{nullptr};
};

class RadarSessionCompositionRoot {
 public:
  static RadarSessionComposition ComposeDefault(const RadarSessionConfig& config);

  static RadarSessionComposition ComposeWithSignalPipeline(
      const RadarSessionConfig& config, signal::pipeline::ISignalPipeline& signal_pipeline);

  static RadarSessionComposition ComposeWithEnvironmentService(
      const RadarSessionConfig& config, environment::IEnvironmentService& environment_service);

  static RadarSessionComposition ComposeWithController(
      const RadarSessionConfig& config, controller::RadarController& controller);

  static RadarSessionComposition ComposeWithExternalChain(
      const RadarSessionConfig& config, context::IRadarContext& radar_context,
      signal::pipeline::ISignalPipeline& signal_pipeline,
      environment::IEnvironmentService& environment_service,
      controller::RadarController& controller);
};

}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_
