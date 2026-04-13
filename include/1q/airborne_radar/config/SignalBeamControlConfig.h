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
  /**
   * @brief 平台姿态角（单位：deg，参考系：局部 ENU）。
   * @note 语义为“ENU -> Body”的 yaw/pitch/roll，不使用 NED 约定。
   *       与 `radar_orientation.mount_angles_deg`（Body -> Radar）复合后，
   *       可得到“ENU -> Radar”的等效姿态。
   */
  model::PlatformAttitudeDeg platform_attitude_deg{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_BEAM_CONTROL_CONFIG_H_
