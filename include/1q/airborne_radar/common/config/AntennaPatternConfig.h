/**
 * @file AntennaPatternConfig.h
 * @brief 定义机载雷达天线方向图建模所需的公共配置结构。
 */

#ifndef AIRBORNE_RADAR_COMMON_ANTENNA_PATTERN_CONFIG_H_
#define AIRBORNE_RADAR_COMMON_ANTENNA_PATTERN_CONFIG_H_

#include "1q/airborne_radar/common/config/RadarOrientationConfig.h"

namespace airborne_radar {
namespace common {
namespace config {

/**
 * @brief AntennaPatternModelType 表示天线方向图主瓣近似模型类型。
 */
enum class AntennaPatternModelType {
  kGaussianMainLobe = 0,  /**< 主瓣采用高斯型工程近似 */
  kParabolicMainLobe = 1, /**< 主瓣采用抛物型工程近似 */
  kCosinePower = 2        /**< 主瓣采用余弦幂工程近似 */
};

/**
 * @brief AntennaPatternBeamwidthDeg 表示参与方向图评估的有效波束宽度。
 * @note 该结构不负责决定宽度来源；调用方应先解析 nominal_* /
 *       commanded_* 规则，再将结果传入方向图评估函数。
 */
struct AntennaPatternBeamwidthDeg {
  float az_beamwidth_deg{3.0f}; /**< 有效方位波束宽度（单位：度） */
  float el_beamwidth_deg{3.0f}; /**< 有效俯仰波束宽度（单位：度） */
};

/**
 * @brief AntennaLookOffsetDeg 表示目标相对当前波束中心的离轴角。
 */
struct AntennaLookOffsetDeg {
  float delta_az_deg{0.0f}; /**< 相对波束中心的方位离轴角（单位：度） */
  float delta_el_deg{0.0f}; /**< 相对波束中心的俯仰离轴角（单位：度） */
};

/**
 * @brief AntennaPatternConfig 表示天线方向图公共配置。
 */
struct AntennaPatternConfig {
  AntennaPatternModelType model_type{/**< 主瓣近似模型 */
                                     AntennaPatternModelType::kGaussianMainLobe};

  float max_sidelobe_level_db{-20.0f};      /**< 最大旁瓣电平（相对主瓣峰值，单位：dB） */
  float backlobe_level_db{-35.0f};          /**< 后瓣电平（相对主瓣峰值，单位：dB） */
  float scan_loss_coeff_db_per_deg2{0.0f};  /**< 扫描损失二次项系数（单位：dB/deg²） */
  float max_scan_loss_db{6.0f};             /**< 最大扫描损失（单位：dB） */
  AzimuthElevationDeg boresight_offset_deg; /**< 阵面法线相对安装基准轴的零偏指向 */
};

/**
 * @brief AntennaPatternSample 表示指定方向上的方向图采样结果。
 */
struct AntennaPatternSample {
  float gain_dbi{0.0f};                 /**< 该方向上的总增益（单位：dBi） */
  float main_lobe_attenuation_db{0.0f}; /**< 主瓣离轴衰减（单位：dB） */
  float scan_loss_db{0.0f};             /**< 扫描损失（单位：dB） */
  bool inside_main_lobe{false};         /**< 是否位于主瓣范围内 */
  bool inside_back_lobe{false};         /**< 是否落入后瓣区域 */
};

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_ANTENNA_PATTERN_CONFIG_H_
