/**
 * @file ArSessionConfig.h
 * @brief 机载雷达会话初始化主配置类型。
 *
 * 会话初始化配置（四域公开模型）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArMissionConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief ArSessionConfig 会话初始化配置（四域公开模型）。
 *
 * 电源状态由顶层 `sensor_enabled` 唯一承载（COMMON-OQ-4 字段提升）：
 * `mission` 域不含电源字段，运行时补丁的电源入口为
 * `ArRuntimeConfigPatch::has_sensor_enabled`。
 */
struct ONEQ_API ArSessionConfig {
  ArHardwareConfig hardware{};
  ArMissionConfig mission{};
  ArPolicyConfig policy{};
  ArEnvironmentConfig environment{};
  bool sensor_enabled{true}; /**< 传感器初始电源状态 */
};


}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_H_
