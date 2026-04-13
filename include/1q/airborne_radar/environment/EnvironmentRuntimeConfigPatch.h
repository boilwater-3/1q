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
 */
struct EnvironmentRuntimeConfigPatch {
  bool has_model_config{false};                   /**< 是否更新环境模型配置 */
  EnvironmentModelConfig model_config{};          /**< 运行期环境模型配置 */
  bool has_jamming_detection_threshold_db{false}; /**< 是否更新干扰判定阈值 */
  float jamming_detection_threshold_db{6.0f};     /**< 运行期干扰判定阈值（单位：dB） */
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
