/**
 * @file RuntimePatchMapper.h
 * @brief 定义运行期补丁映射入口。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "airborne_radar/config/execution/InternalExecutionConfig.h"

namespace airborne_radar {
namespace config {
namespace mapping {

struct RuntimePatchMappingResult {
  execution::InternalExecutionConfig next_execution_config{};
  model::AzimuthElevationDeg next_dwell_center_deg{};
  environment::EnvironmentScenarioConfig next_environment_scenario_config{};
  environment::JammingSensitivityProfile next_jamming_sensitivity_profile{
      environment::JammingSensitivityProfile::kBalanced};
  bool has_requested_update{false};
  bool is_valid{true};
  bool execution_config_changed{false};
  bool dwell_center_changed{false};
  bool environment_scenario_config_changed{false};
  bool jamming_sensitivity_profile_changed{false};
};

RuntimePatchMappingResult ApplyRuntimePatch(
    const execution::InternalExecutionConfig& current_execution_config,
    const RadarRuntimeConfigPatch& patch);

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_
