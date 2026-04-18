/**
 * @file EosSessionConfig.h
 * @brief 定义 EOS 会话初始化配置结构。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/config/EosHardwareConfig.h"
#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/config/EosPolicyConfig.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosSessionConfig 描述 EOS 会话初始化高层输入。
 */
struct ONEQ_API EosSessionConfig {
  config::EosHardwareConfig hardware{};
  config::EosMissionConfig mission{};
  config::EosPolicyConfig policy{};
  config::EosEnvironmentConfig environment{};
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
