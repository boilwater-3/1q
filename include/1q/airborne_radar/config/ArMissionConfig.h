/**
 * @file ArMissionConfig.h
 * @brief 机载雷达任务域主配置类型。
 *
 * 任务域配置（开关机、波束指向运行态等）的主头文件。
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
 * 任务域承载工作子模式与波束指向运行态。电源状态由
 * `ArSessionConfig::sensor_enabled` 顶层字段唯一承载（COMMON-OQ-4 收敛）。
 */
struct ONEQ_API ArMissionConfig {
  config::ArOrientationConfig orientation{};
};


}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_MISSION_CONFIG_H_
