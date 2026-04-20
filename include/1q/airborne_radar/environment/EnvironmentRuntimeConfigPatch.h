/**
 * @file EnvironmentRuntimeConfigPatch.h
 * @brief 定义运行期可变环境参数补丁类型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_

#include "1q/airborne_radar/environment/EnvironmentConfig.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief EnvironmentRuntimeConfigPatch 描述运行期可变环境参数补丁。
 *
 * @par 类型合约
 * - 仅包含运行期可变更的环境参数项。
 * - 每个字段配备对应的 has_* 布尔标志，未设置的项不参与更新。
 * - 支持的补丁项：scenario_config、jamming_sensitivity_profile。
 * - 解析时遵循原子语义：整个补丁要么全部生效，要么全部拒绝。
 */
struct EnvironmentRuntimeConfigPatch {
  bool has_scenario_config{false};                /**< 是否更新环境场景输入 */
  EnvironmentScenarioConfig scenario_config{};    /**< 运行期环境场景输入 */
  bool has_jamming_sensitivity_profile{false};    /**< 是否更新干扰判定灵敏度语义档位 */
  JammingSensitivityProfile jamming_sensitivity_profile{
      JammingSensitivityProfile::kBalanced};      /**< 运行期干扰判定灵敏度语义档位 */
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
