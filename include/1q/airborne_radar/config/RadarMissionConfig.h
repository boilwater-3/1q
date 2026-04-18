/**
 * @file RadarMissionConfig.h
 * @brief 定义雷达任务域公开配置。
 *
 * 任务域承载工作子模式与波束指向运行态。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_MISSION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_MISSION_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief 雷达任务域配置。
 *
 * 当前阶段任务域承载工作子模式与波束指向运行态。
 */
struct ONEQ_API RadarMissionConfig {
  model::RadarOrientationConfig orientation{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_MISSION_CONFIG_H_
