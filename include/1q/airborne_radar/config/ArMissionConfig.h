/**
 * @file ArMissionConfig.h
 * @brief AR module primary mission configuration type.
 *
 * Primary header for mission domain configuration.
 * Include this for new code; RadarMissionConfig.h is the deprecated compat wrapper.
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_MISSION_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_MISSION_CONFIG_H_

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief ArMissionConfig 雷达任务域配置。
 *
 * 任务域承载工作子模式与波束指向运行态。
 */
struct ONEQ_API ArMissionConfig {
  bool power_on{true};                           /**< 设备开关机状态 */
  config::ArOrientationConfig orientation{};
};

// 兼容别名：旧 RadarMissionConfig 名称在 wrapper 阶段保留。
using RadarMissionConfig = ArMissionConfig;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_MISSION_CONFIG_H_
