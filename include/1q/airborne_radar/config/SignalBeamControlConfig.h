/**
 * @file SignalBeamControlConfig.h
 * @brief 定义波束控制域关键配置（config 主入口）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_BEAM_CONTROL_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_BEAM_CONTROL_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {

/**
 * @brief SignalBeamControlConfig 描述波束控制域关键配置。
 */
struct SignalBeamControlConfig {
  model::RadarOrientationConfig radar_orientation{}; /**< 雷达方向配置 */
  model::PlatformAttitudeDeg platform_attitude_deg{}; /**< 平台姿态角（单位：度） */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_BEAM_CONTROL_CONFIG_H_
