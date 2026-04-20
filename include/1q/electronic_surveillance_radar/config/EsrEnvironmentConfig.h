/**
 * @file EsrEnvironmentConfig.h
 * @brief ESR 环境配置聚合别名（真源位于 environment 层）。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_

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

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
