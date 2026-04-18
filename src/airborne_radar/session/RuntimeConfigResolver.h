#ifndef AIRBORNE_RADAR_SESSION_RUNTIME_CONFIG_RESOLVER_H_
#define AIRBORNE_RADAR_SESSION_RUNTIME_CONFIG_RESOLVER_H_

#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarMissionConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"

namespace airborne_radar {
namespace session {
namespace internal {

/**
 * @brief RuntimeConfigState 描述会话持有的运行期配置唯一真值。
 */
struct RuntimeConfigState {
  config::RadarHardwareConfig hardware{};
  config::RadarMissionConfig mission{};
  config::RadarPolicyConfig policy{};
  environment::EnvironmentScenarioConfig environment_scenario_config{};
  environment::JammingSensitivityProfile jamming_sensitivity_profile{
      environment::JammingSensitivityProfile::kBalanced};
  model::AzimuthElevationDeg dwell_center_deg{};
};

/**
 * @brief RuntimeConfigResolveResult 描述运行期补丁解析结果与执行计划。
 */
struct RuntimeConfigResolveResult {
  RuntimeConfigState next_state{};
  bool has_requested_update{false};
  bool is_valid{true};
  bool pipeline_config_changed{false};
  bool environment_scenario_config_changed{false};
  bool jamming_sensitivity_profile_changed{false};
};

/**
 * @brief 解析运行期补丁并生成会话运行态更新计划。
 * @param[in] current_state 当前运行态。
 * @param[in] patch 外部提交补丁。
 * @return 解析结果与更新计划。
 */
RuntimeConfigResolveResult ResolveRuntimeConfigPatch(const RuntimeConfigState& current_state,
                                                     const config::RadarRuntimeConfigPatch& patch);

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RUNTIME_CONFIG_RESOLVER_H_
