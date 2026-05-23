/**
 * @file RadarSessionConfig.h
 * @brief 定义 RadarSession 的四域初始化配置聚合结构。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/RadarEnvironmentConfig.h"
#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarMissionConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief RadarSession 初始化配置（四域公开模型）。
 */
struct ONEQ_API RadarSessionConfig {
  RadarHardwareConfig hardware{};
  RadarMissionConfig mission{};
  RadarPolicyConfig policy{};
  RadarEnvironmentConfig environment{};
  environment::JammingSensitivityProfile jamming_sensitivity_profile{
      environment::JammingSensitivityProfile::kBalanced}; /**< 干扰判定灵敏度语义档位 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
