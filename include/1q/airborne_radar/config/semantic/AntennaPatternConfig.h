/**
 * @file AntennaPatternConfig.h
 * @brief 定义 semantic 天线方向图配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_ANTENNA_PATTERN_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_ANTENNA_PATTERN_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief 方向图语义档位。
 */
enum class AntennaPatternProfile {
  kStandard = 0, /**< 标准主瓣与旁瓣行为。 */
  kLowSidelobe = 1, /**< 优先降低旁瓣泄漏。 */
  kWideCoverage = 2 /**< 优先扩大覆盖波束。 */
};

/**
 * @brief semantic 天线方向图配置。
 */
struct AntennaPatternConfig {
  AntennaPatternProfile profile{AntennaPatternProfile::kStandard}; /**< 方向图语义档位。 */
  model::AzimuthElevationDeg boresight_offset_deg{}; /**< 相对雷达安装轴的方向图偏置。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_ANTENNA_PATTERN_CONFIG_H_
