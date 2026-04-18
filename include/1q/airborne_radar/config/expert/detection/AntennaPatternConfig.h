/**
 * @file AntennaPatternConfig.h
 * @brief 定义 expert 天线方向图配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_ANTENNA_PATTERN_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_ANTENNA_PATTERN_CONFIG_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert 方向图模型类型。
 */
enum class AntennaPatternModelType {
  kGaussianMainLobe = 0, /**< 高斯主瓣近似。 */
  kParabolicMainLobe = 1, /**< 抛物线主瓣近似。 */
  kCosinePower = 2 /**< 余弦幂方向图近似。 */
};

/**
 * @brief expert 天线方向图参数。
 */
struct AntennaPatternConfig {
  AntennaPatternModelType model_type{AntennaPatternModelType::kGaussianMainLobe}; /**< 主瓣模型类型。 */
  float max_sidelobe_level_db{-20.0f}; /**< 最大旁瓣电平。 */
  float backlobe_level_db{-35.0f}; /**< 后瓣电平。 */
  float scan_loss_coeff_db_per_deg2{0.0f}; /**< 扫描损失系数。 */
  float max_scan_loss_db{6.0f}; /**< 扫描损失上限。 */
  model::AzimuthElevationDeg boresight_offset_deg{}; /**< 方向图相对安装轴偏置。 */
};

}  // namespace detection
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_ANTENNA_PATTERN_CONFIG_H_
