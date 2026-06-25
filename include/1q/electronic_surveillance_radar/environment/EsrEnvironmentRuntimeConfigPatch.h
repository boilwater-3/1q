/**
 * @file EsrEnvironmentRuntimeConfigPatch.h
 * @brief ESR 环境运行期补丁契约。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrEnvironmentRuntimeConfigPatch 描述运行期可变环境补丁。
 *
 * @note 环境预设（EsrEnvironmentPreset）是会话初始化语义，运行期不支持热更新；
 *       早期版本的 has_preset/preset 字段已移除（曾导致"置 true 即被 reject"的
 *       弃用陷阱）。运行期仅可调整基础气象观测参数与时空天气上下文。
 */
struct ONEQ_API EsrEnvironmentRuntimeConfigPatch {
  bool has_atmospheric_physics{false}; /**< 是否更新基础气象观测参数 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{};

  bool has_atmospheric_context{false}; /**< 是否更新时间/空间天气上下文 */
  EsrAtmosphericDerivedContext atmospheric_context{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
