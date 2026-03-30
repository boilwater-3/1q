/**
 * @file AntennaPatternConfig.h
 * @brief 定义机载雷达天线方向图配置。
 * @note “可外部调整”定义：调用方可在不重建 `RadarSession` 的前提下，通过公开 API 直接提交修改。
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
 *       默认属于初始化固定配置。
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
 * @note 两个成员均为“可外部调整”的运行期输入量。
 */
struct AntennaPatternBeamwidthDeg {
  float az_beamwidth_deg{3.0f}; /**< [可外部调整] 有效方位波束宽度（单位：度） */
  float el_beamwidth_deg{3.0f}; /**< [可外部调整] 有效俯仰波束宽度（单位：度） */
};

/**
 * @brief AntennaLookOffsetDeg 表示目标相对当前波束中心的离轴角。
 * @note 两个成员均为“可外部调整”的运行期输入量。
 */
struct AntennaLookOffsetDeg {
  float delta_az_deg{0.0f}; /**< [可外部调整] 相对波束中心的方位离轴角（单位：度） */
  float delta_el_deg{0.0f}; /**< [可外部调整] 相对波束中心的俯仰离轴角（单位：度） */
};

/**
 * @brief AntennaPatternConfig 表示天线方向图公共配置。
 * @note 下列成员默认为“初始化固定”；当前公开 `RadarSession` 运行期接口不直接提供逐项修改。
 */
struct AntennaPatternConfig {
  AntennaPatternModelType model_type{
      AntennaPatternModelType::kGaussianMainLobe}; /**< [初始化固定] 主瓣近似模型 */

  float max_sidelobe_level_db{-20.0f};     /**< [初始化固定] 最大旁瓣电平（相对主瓣峰值，单位：dB） */
  float backlobe_level_db{-35.0f};         /**< [初始化固定] 后瓣电平（相对主瓣峰值，单位：dB） */
  float scan_loss_coeff_db_per_deg2{0.0f}; /**< [初始化固定] 扫描损失二次项系数（单位：dB/deg²） */
  float max_scan_loss_db{6.0f};            /**< [初始化固定] 最大扫描损失（单位：dB） */
  AzimuthElevationDeg boresight_offset_deg; /**< [初始化固定] 阵面法线相对安装基准轴的零偏指向 */
};

/**
 * @brief AntennaPatternSample 表示指定方向上的方向图采样结果。
 * @note 该结构是运行期计算输出，不属于可配置输入；成员不参与“可外部调整/初始化固定”分类。
 */
struct AntennaPatternSample {
  float gain_dbi{0.0f};                 /**< [运行期输出] 该方向上的总增益（单位：dBi） */
  float main_lobe_attenuation_db{0.0f}; /**< [运行期输出] 主瓣离轴衰减（单位：dB） */
  float scan_loss_db{0.0f};             /**< [运行期输出] 扫描损失（单位：dB） */
  bool inside_main_lobe{false};         /**< [运行期输出] 是否位于主瓣范围内 */
  bool inside_back_lobe{false};         /**< [运行期输出] 是否落入后瓣区域 */
};

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_ANTENNA_PATTERN_CONFIG_H_
