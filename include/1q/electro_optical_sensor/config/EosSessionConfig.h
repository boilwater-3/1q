/**
 * @file EosSessionConfig.h
 * @brief 定义 EOS 会话初始化配置结构。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/config/EosHardwareConfig.h"
#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/config/EosPolicyConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief config::EosSessionConfig 描述 EOS 会话初始化高层输入。
 *
 * 电源状态由顶层 `sensor_enabled` 唯一承载（COMMON-OQ-4 收敛）：
 * `mission` 域不含电源字段，运行时补丁的电源入口为
 * `EosRuntimeConfigPatch::has_sensor_enabled`。
 */
struct ONEQ_API EosSessionConfig {
  EosHardwareConfig hardware{};
  EosMissionConfig mission{};
  EosPolicyConfig policy{};
  config::EosEnvironmentConfig environment{};
  bool sensor_enabled{true}; /**< 传感器初始电源状态 */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
