#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/airborne_radar/session/RadarSession.h"

namespace airborne_radar {
namespace session {
namespace internal {

struct RadarSessionComposition {
  config::SignalPipelineConfig runtime_signal_pipeline_config{};
  environment::EnvironmentModelConfig runtime_environment_model_config{};
  float runtime_jamming_detection_threshold_db{6.0f};

  std::unique_ptr<extension::IRadarContext> owned_radar_context;
  std::unique_ptr<extension::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<extension::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::RadarController> owned_controller;

  extension::IRadarContext* radar_context{nullptr};
  extension::ISignalPipeline* signal_pipeline{nullptr};
  extension::IEnvironmentService* environment_service{nullptr};
  extension::RadarController* controller{nullptr};
};

class RadarSessionCompositionRoot {
 public:
  static RadarSessionComposition ComposeDefault(const RadarSessionConfig& config);

  static RadarSessionComposition ComposeWithSignalPipeline(
      const RadarSessionConfig& config, extension::ISignalPipeline& signal_pipeline);

  static RadarSessionComposition ComposeWithEnvironmentService(
      const RadarSessionConfig& config, extension::IEnvironmentService& environment_service);

  static RadarSessionComposition ComposeWithController(
      const RadarSessionConfig& config, extension::RadarController& controller);
};

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_
