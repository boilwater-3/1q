/**
 * @file EosEnvironmentConfig.h
 * @brief EOS 单一场景环境配置契约。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/environment/AtmosphericTypes.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosEnvironmentPreset 描述高层环境预设。
 */
enum class ONEQ_API EosEnvironmentPreset {
  kStandard = 0,
  kHumid,
  kDusty,
  kTurbulent,
  kMaritime
};

/** @brief EosAtmosphericPhysicsConfig 复用统一环境模块基础气象观测类型。 */
using EosAtmosphericPhysicsConfig = oneq::environment::AtmosphericObservation;

/**
 * @brief EosEnvironmentScenarioConfig 描述外部场景语义输入。
 *
 * 调用方只选择一个 @ref preset。pipeline 根据 preset 派生辐射传输算法、
 * 气溶胶和湍流基线，并始终结合单周期高度、云量与风速自动动态修正。
 * @ref atmospheric_physics 与 ESR 使用同一标准大气观测类型；其
 * `enable_physical_model` 为 `true` 时，湿度和温度进一步参与物理修正。
 */
struct ONEQ_API EosEnvironmentScenarioConfig {
  EosEnvironmentPreset preset{EosEnvironmentPreset::kStandard}; /**< 环境参数基线 */
  EosAtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选标准大气物理观测 */
};

/**
 * @brief EosEnvironmentConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EosEnvironmentConfig {
  EosEnvironmentScenarioConfig scenario_config{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
