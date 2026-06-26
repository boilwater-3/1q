#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/session/RadarSession.h"

namespace airborne_radar {
namespace extension {
class ITacticalDecisionEngine;
class IRadarContext;
class ISignalPipeline;
class RadarController;
}  // namespace extension
namespace environment {
class IEnvironmentService;
}
namespace session {

struct RadarSessionComposition {
  config::RadarHardwareConfig runtime_hardware{};
  config::RadarMissionConfig runtime_mission{};
  config::RadarPolicyConfig runtime_policy{};
  environment::EnvironmentScenarioConfig runtime_environment_scenario_config{};
  environment::JammingSensitivityProfile runtime_jamming_sensitivity_profile{
      environment::JammingSensitivityProfile::kBalanced};

  std::unique_ptr<extension::IRadarContext> owned_radar_context;
  std::unique_ptr<extension::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<environment::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::RadarController> owned_controller;

  extension::IRadarContext* radar_context{nullptr};
  extension::ISignalPipeline* signal_pipeline{nullptr};
  environment::IEnvironmentService* environment_service{nullptr};
  extension::RadarController* controller{nullptr};
  bool pipeline_config_synced{true};
};

class RadarSessionCompositionRoot {
 public:
  static RadarSessionComposition ComposeDefault(const config::RadarSessionConfig& config);

  /**
   * @brief 注入自定义决策引擎装配会话；context/pipeline/environment 由内部默认装配。
   */
  static RadarSessionComposition ComposeWithDecisionEngine(
      const config::RadarSessionConfig& config,
      extension::ITacticalDecisionEngine& decision_engine);
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_COMPOSITION_ROOT_H_
