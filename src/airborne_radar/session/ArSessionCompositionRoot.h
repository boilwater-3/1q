#ifndef AIRBORNE_RADAR_CORE_SESSION_AR_SESSION_COMPOSITION_ROOT_H_
#define AIRBORNE_RADAR_CORE_SESSION_AR_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace session {
class ITacticalDecisionEngine;
}  // namespace session
namespace signal {
class ISignalPipeline;
}  // namespace signal
namespace extension {
class ArController;
}  // namespace extension
namespace environment {
class IEnvironmentService;
}
namespace session {
class MutableArContext;

struct ArSessionComposition {
  config::ArHardwareConfig runtime_hardware{};
  config::ArMissionConfig runtime_mission{};
  config::ArPolicyConfig runtime_policy{};
  config::EnvironmentScenarioConfig runtime_environment_scenario_config{};
  config::JammingSensitivityProfile runtime_jamming_sensitivity_profile{
      config::JammingSensitivityProfile::kBalanced};

  std::unique_ptr<MutableArContext> owned_ar_context;
  std::unique_ptr<signal::ISignalPipeline> owned_signal_pipeline;
  std::unique_ptr<environment::IEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::ArController> owned_controller;

  MutableArContext* ar_context{nullptr};
  signal::ISignalPipeline* signal_pipeline{nullptr};
  environment::IEnvironmentService* environment_service{nullptr};
  extension::ArController* controller{nullptr};
  bool pipeline_config_synced{true};
};

class ArSessionCompositionRoot {
 public:
  static ArSessionComposition ComposeDefault(const config::ArSessionConfig& config);

  /**
   * @brief 注入自定义决策引擎装配会话；context/pipeline/environment 由内部默认装配。
   */
  static ArSessionComposition ComposeWithDecisionEngine(
      const config::ArSessionConfig& config, session::ITacticalDecisionEngine& decision_engine);
};


}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_AR_SESSION_COMPOSITION_ROOT_H_
