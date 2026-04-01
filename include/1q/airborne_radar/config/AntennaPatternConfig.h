/**
 * @file AntennaPatternConfig.h
 * @brief 定义机载雷达天线方向图初始化配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_

#include "1q/airborne_radar/config/RadarOrientationConfig.h"

namespace airborne_radar {
namespace common {
namespace config {

/**
 * @brief AntennaPatternModelType 表示天线方向图主瓣近似模型类型。
 * @note 该枚举作为 `AntennaPatternConfig::model_type` 的取值域，
 *       对应的 `AntennaPatternConfig` 结构体为“初始化固定”，不支持仿真过程中进行修改。
 */
enum class AntennaPatternModelType {
  kGaussianMainLobe = 0,  /**< 主瓣采用高斯型工程近似 */
  kParabolicMainLobe = 1, /**< 主瓣采用抛物型工程近似 */
  kCosinePower = 2        /**< 主瓣采用余弦幂工程近似 */
};

/**
 * @brief AntennaPatternConfig 表示天线方向图公共配置。
 * @note 该结构体为“初始化固定”，不支持仿真过程中进行修改。
 */
struct AntennaPatternConfig {
  AntennaPatternModelType model_type{
      AntennaPatternModelType::kGaussianMainLobe}; /**< 主瓣近似模型 */
  float max_sidelobe_level_db{-20.0f};             /**< 最大旁瓣电平（相对主瓣峰值，单位：dB） */
  float backlobe_level_db{-35.0f};                 /**< 后瓣电平（相对主瓣峰值，单位：dB） */
  float scan_loss_coeff_db_per_deg2{0.0f};         /**< 扫描损失二次项系数（单位：dB/deg²） */
  float max_scan_loss_db{6.0f};                    /**< 最大扫描损失（单位：dB） */
  AzimuthElevationDeg boresight_offset_deg;        /**< 阵面法线相对安装基准轴的零偏指向 */
};

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_
