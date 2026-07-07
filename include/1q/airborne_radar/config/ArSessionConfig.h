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
 */
struct ONEQ_API ArSessionConfig {
  ArHardwareConfig hardware{};
  ArMissionConfig mission{};
  ArPolicyConfig policy{};
  ArEnvironmentConfig environment{};
};


}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_H_
