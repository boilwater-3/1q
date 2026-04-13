/**
 * @file EsrEnvironmentRuntimeConfigPatch.h
 * @brief ESR 环境运行期补丁契约。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrEnvironmentRuntimeConfigPatch 描述运行期可变环境补丁。
 */
struct ONEQ_API EsrEnvironmentRuntimeConfigPatch {
  bool has_model_config{false};                  /**< 是否更新环境模型配置 */
  EsrEnvironmentModelConfig model_config{};      /**< 运行期环境模型配置 */
  bool has_jamming_detection_threshold_w{false}; /**< 是否更新干扰检测阈值 */
  float jamming_detection_threshold_w{1.0e-9f};  /**< 干扰检测阈值（单位：W） */
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
