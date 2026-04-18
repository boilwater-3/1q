/**
 * @file RadarSessionConfig.h
 * @brief 定义 RadarSession 的四域初始化配置聚合结构。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/RadarEnvironmentConfig.h"
#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarMissionConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSession 初始化配置（四域公开模型）。
 */
struct ONEQ_API RadarSessionConfig {
  config::RadarHardwareConfig hardware{};
  config::RadarMissionConfig mission{};
  config::RadarPolicyConfig policy{};
  config::RadarEnvironmentConfig environment{};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
