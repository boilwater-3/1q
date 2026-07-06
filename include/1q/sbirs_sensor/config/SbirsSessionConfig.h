/**
 * @file SbirsSessionConfig.h
 * @brief 定义 SBIRS-inspired 会话初始化配置。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"
#include "1q/sbirs_sensor/config/SbirsHardwareConfig.h"
#include "1q/sbirs_sensor/config/SbirsMissionConfig.h"
#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"

namespace sbirs_sensor {
namespace config {

struct ONEQ_API SbirsSessionConfig {
  SbirsHardwareConfig hardware{};
  SbirsMissionConfig mission{};
  SbirsPolicyConfig policy{};
  SbirsEnvironmentConfig environment{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_H_
