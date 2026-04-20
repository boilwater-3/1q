/**
 * @file EosEnvironmentConfig.h
 * @brief EOS 环境配置别名（语义真源位于 environment/）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace config {

using EosEnvironmentPreset = environment::EosEnvironmentPreset;
using EosEnvironmentScenarioConfig = environment::EosEnvironmentScenarioConfig;
using EosEnvironmentModelConfig = environment::EosEnvironmentModelConfig;
using EosEnvironmentCustomOverrides = environment::EosEnvironmentCustomOverrides;
using EosEnvironmentConfig = environment::EosEnvironmentDefaultConfig;

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
