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
  bool has_preset{false}; /**< [已弃用] 运行期不支持更新环境预设语义；若置 true 将被 reject */
  config::EsrEnvironmentPreset preset{config::EsrEnvironmentPreset::kStandard};

  bool has_atmospheric_physics{false}; /**< 是否更新基础气象观测参数 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{};

  bool has_atmospheric_context{false}; /**< 是否更新时间/空间天气上下文 */
  EsrAtmosphericDerivedContext atmospheric_context{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
