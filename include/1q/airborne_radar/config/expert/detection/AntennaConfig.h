/**
 * @file AntennaConfig.h
 * @brief 定义 expert 天线配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_ANTENNA_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_ANTENNA_CONFIG_H_

#include "1q/airborne_radar/config/expert/detection/AntennaPatternConfig.h"

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert 天线工程参数。
 */
struct AntennaConfig {
  float main_beam_gain_db{35.0f}; /**< 主瓣峰值增益。 */
  float nominal_az_beamwidth_deg{4.0f}; /**< 名义方位波束宽度。 */
  float nominal_el_beamwidth_deg{4.0f}; /**< 名义俯仰波束宽度。 */
  AntennaPatternConfig pattern{}; /**< 方向图参数。 */
  bool enable_directional_pattern{false}; /**< 是否启用离轴方向图评估。 */
};

}  // namespace detection
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_ANTENNA_CONFIG_H_
