/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境配置便利别名入口——真源定义位于 environment/EsrEnvironmentConfig.h。
 *
 * 本文件专为 config 命名空间使用者提供便利，将 environment:: 下的环境类型以
 * using 声明聚合到 config:: 命名空间，避免 config 层代码跨子目录引用。
 * 不引入新类型定义，所有 using 别名在契约测试中验证与真源类型等价。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace config {

using EsrAtmosphericPhysicsConfig = environment::EsrAtmosphericPhysicsConfig;
using EsrAtmosphericDerivedContext = environment::EsrAtmosphericDerivedContext;
using EsrEnvironmentScenarioConfig = environment::EsrEnvironmentScenarioConfig;
using EsrEnvironmentModelConfig = environment::EsrEnvironmentModelConfig;

/**
 * @brief EsrEnvironmentConfig 是 environment::EsrEnvironmentDefaultConfig 的聚合别名。
 */
using EsrEnvironmentConfig = environment::EsrEnvironmentDefaultConfig;

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
