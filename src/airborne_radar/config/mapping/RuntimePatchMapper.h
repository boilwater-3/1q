/**
 * @file RuntimePatchMapper.h
 * @brief 定义运行期补丁映射与执行配置反向映射入口。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "airborne_radar/config/InternalExecutionConfig.h"

namespace airborne_radar {
namespace config {
namespace mapping {

/**
 * @brief RuntimeConfigState 描述会话持有的运行期配置唯一真值。
 */
struct RuntimeConfigState {
  execution::InternalExecutionConfig execution_config{};
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
  bool execution_config_changed{false};
  bool environment_scenario_config_changed{false};
  bool jamming_sensitivity_profile_changed{false};
};

/**
 * @brief 解析运行期补丁并生成会话运行态更新计划。
 * @param[in] current_state 当前运行态。
 * @param[in] patch 外部提交补丁。
 * @return 解析结果与更新计划。
 */
RuntimeConfigResolveResult ApplyRuntimePatch(const RuntimeConfigState& current_state,
                                             const RadarRuntimeConfigPatch& patch);

/**
 * @brief 将内部执行配置反向映射为四域会话配置。
 * @param execution_config 内部执行配置。
 * @return 对应的四域会话配置。
 */
session::RadarSessionConfig MapExecutionToSession(
    const execution::InternalExecutionConfig& execution_config);

/**
 * @brief 将运行期状态映射为当前 pipeline 可消费的四域会话配置。
 * @param runtime_state 当前运行期状态。
 * @return 已叠加运行期驻留偏移的四域会话配置。
 */
session::RadarSessionConfig MapRuntimeStateToPipelineSession(
    const RuntimeConfigState& runtime_state);

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_
