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

namespace engineering {

/**
 * @brief AntennaPatternModelType 表示内部方向图主瓣近似模型类型。
 */
enum class AntennaPatternModelType {
  kGaussianMainLobe = 0,
  kParabolicMainLobe = 1,
  kCosinePower = 2
};

/**
 * @brief AntennaPatternConfig 表示内部工程方向图参数。
 */
struct AntennaPatternConfig {
  AntennaPatternModelType model_type{
      AntennaPatternModelType::kGaussianMainLobe};
  float max_sidelobe_level_db{-20.0f};
  float backlobe_level_db{-35.0f};
  float scan_loss_coeff_db_per_deg2{0.0f};
  float max_scan_loss_db{6.0f};
  model::AzimuthElevationDeg boresight_offset_deg{};
};

}  // namespace engineering

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_
