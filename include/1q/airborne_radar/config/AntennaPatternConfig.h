/**
 * @file AntennaPatternConfig.h
 * @brief 定义机载雷达天线方向图初始化配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {

/**
 * @brief AntennaPatternProfile 表示对外方向图能力档位。
 */
enum class AntennaPatternProfile {
  kStandard = 0,    /**< 通用阵面能力 */
  kLowSidelobe = 1, /**< 低旁瓣能力 */
  kWideCoverage = 2 /**< 广覆盖能力 */
};

/**
 * @brief AntennaPatternConfig 表示对外语义化方向图输入。
 */
struct AntennaPatternConfig {
  AntennaPatternProfile profile{AntennaPatternProfile::kStandard}; /**< 方向图能力档位 */
  model::AzimuthElevationDeg boresight_offset_deg{};               /**< 外部标定零偏 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_
