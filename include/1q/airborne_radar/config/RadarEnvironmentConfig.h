/**
 * @file RadarEnvironmentConfig.h
 * @brief 定义雷达环境域公开配置。
 *
 * 环境域直接复用 environment::EnvironmentDefaultConfig。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_ENVIRONMENT_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_ENVIRONMENT_CONFIG_H_

#include "1q/airborne_radar/environment/EnvironmentConfig.h"

namespace airborne_radar {
namespace config {

using RadarEnvironmentConfig = environment::EnvironmentDefaultConfig;

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_ENVIRONMENT_CONFIG_H_
