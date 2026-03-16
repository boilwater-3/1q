// Copyright 2026. All Rights Reserved.
//
// Description: 定义机载雷达天线方向图的工程近似评估工具函数。

#ifndef AIRBORNE_RADAR_COMMON_ANTENNA_PATTERN_UTILS_H_
#define AIRBORNE_RADAR_COMMON_ANTENNA_PATTERN_UTILS_H_

#include <cmath>

#include "1q/airborne_radar/common/AntennaPatternConfig.h"

namespace airborne_radar {
namespace common {

namespace antenna_pattern_internal {

/// @brief 对浮点值执行下限保护。
/// @param value 原始值。
/// @param min_value 下限。
/// @return 不小于下限的结果。
inline float ClampLowerBound(float value, float min_value) {
  return value < min_value ? min_value : value;
}

/// @brief 计算闭区间限幅结果。
/// @param value 原始值。
/// @param min_value 下界。
/// @param max_value 上界。
/// @return 限幅后的结果。
inline float ClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

/// @brief 判断给定离轴角是否位于后瓣区域。
/// @param offset_deg 目标相对当前波束中心的离轴角。
/// @return 若任一轴绝对离轴角超过 90 度，则视为后瓣区域。
inline bool IsInsideBackLobe(const AntennaLookOffsetDeg& offset_deg) {
  return std::fabs(offset_deg.delta_az_deg) > 90.0f ||
         std::fabs(offset_deg.delta_el_deg) > 90.0f;
}

}  // namespace antenna_pattern_internal

/// @brief 判断目标是否位于主瓣范围内。
/// @param beamwidth_deg 用于评估的有效波束宽度。
/// @param offset_deg 目标相对当前波束中心的离轴角。
/// @return 若方位与俯仰离轴角均不超过对应半功率波束半宽，则返回 true。
inline bool IsInsideMainLobe(
    const AntennaPatternBeamwidthDeg& beamwidth_deg,
    const AntennaLookOffsetDeg& offset_deg) {
  const float half_az_beamwidth_deg =
      0.5f *
      antenna_pattern_internal::ClampLowerBound(beamwidth_deg.az_beamwidth_deg,
                                                1e-3f);
  const float half_el_beamwidth_deg =
      0.5f *
      antenna_pattern_internal::ClampLowerBound(beamwidth_deg.el_beamwidth_deg,
                                                1e-3f);
  return std::fabs(offset_deg.delta_az_deg) <= half_az_beamwidth_deg &&
         std::fabs(offset_deg.delta_el_deg) <= half_el_beamwidth_deg;
}

/// @brief 计算指定离轴角下的主瓣衰减。
/// @param config 天线方向图配置。
/// @param beamwidth_deg 用于评估的有效波束宽度。
/// @param offset_deg 目标相对当前波束中心的离轴角。
/// @return 主瓣离轴衰减（单位：dB）。
inline float ComputeMainLobeAttenuationDb(
    const AntennaPatternConfig& config,
    const AntennaPatternBeamwidthDeg& beamwidth_deg,
    const AntennaLookOffsetDeg& offset_deg) {
  const float half_az_beamwidth_deg =
      0.5f *
      antenna_pattern_internal::ClampLowerBound(beamwidth_deg.az_beamwidth_deg,
                                                1e-3f);
  const float half_el_beamwidth_deg =
      0.5f *
      antenna_pattern_internal::ClampLowerBound(beamwidth_deg.el_beamwidth_deg,
                                                1e-3f);
  const float normalized_az =
      std::fabs(offset_deg.delta_az_deg) / half_az_beamwidth_deg;
  const float normalized_el =
      std::fabs(offset_deg.delta_el_deg) / half_el_beamwidth_deg;

  switch (config.model_type) {
    case AntennaPatternModelType::kParabolicMainLobe:
      return 3.0f *
             (normalized_az * normalized_az +
              normalized_el * normalized_el);

    case AntennaPatternModelType::kCosinePower: {
      const float kDeg2Rad = 3.14159265358979f / 180.0f;
      const float az_offset_rad =
          antenna_pattern_internal::ClampFloat(offset_deg.delta_az_deg, -89.9f,
                                               89.9f) *
          kDeg2Rad;
      const float el_offset_rad =
          antenna_pattern_internal::ClampFloat(offset_deg.delta_el_deg, -89.9f,
                                               89.9f) *
          kDeg2Rad;
      const float az_half_bw_rad = half_az_beamwidth_deg * kDeg2Rad;
      const float el_half_bw_rad = half_el_beamwidth_deg * kDeg2Rad;
      const float az_denominator =
          std::log(antenna_pattern_internal::ClampLowerBound(
              std::cos(az_half_bw_rad), 1e-6f));
      const float el_denominator =
          std::log(antenna_pattern_internal::ClampLowerBound(
              std::cos(el_half_bw_rad), 1e-6f));
      const float az_power = -0.69314718055995f / az_denominator;
      const float el_power = -0.69314718055995f / el_denominator;
      const float az_gain =
          std::pow(antenna_pattern_internal::ClampLowerBound(
                       std::cos(az_offset_rad), 1e-6f),
                   az_power);
      const float el_gain =
          std::pow(antenna_pattern_internal::ClampLowerBound(
                       std::cos(el_offset_rad), 1e-6f),
                   el_power);
      return -10.0f *
             std::log10(antenna_pattern_internal::ClampLowerBound(
                 az_gain * el_gain, 1e-6f));
    }

    case AntennaPatternModelType::kGaussianMainLobe:
    default:
      return 3.0f *
             (normalized_az * normalized_az +
              normalized_el * normalized_el);
  }
}

/// @brief 计算当前扫描中心下的扫描损失。
/// @param config 天线方向图配置。
/// @param scan_center_deg 当前扫描中心方向。
/// @return 扫描损失（单位：dB）。
inline float ComputeScanLossDb(
    const AntennaPatternConfig& config,
    const AzimuthElevationDeg& scan_center_deg) {
  const float delta_scan_az_deg =
      scan_center_deg.az_deg - config.boresight_offset_deg.az_deg;
  const float delta_scan_el_deg =
      scan_center_deg.el_deg - config.boresight_offset_deg.el_deg;
  const float raw_scan_loss_db =
      config.scan_loss_coeff_db_per_deg2 *
      (delta_scan_az_deg * delta_scan_az_deg +
       delta_scan_el_deg * delta_scan_el_deg);
  return antenna_pattern_internal::ClampFloat(
      raw_scan_loss_db, 0.0f,
      antenna_pattern_internal::ClampLowerBound(config.max_scan_loss_db, 0.0f));
}

/// @brief 评估指定方向上的天线方向图结果。
/// @param peak_gain_dbi 波束中心峰值增益（单位：dBi）。
/// @param config 天线方向图配置。
/// @param beamwidth_deg 用于评估的有效波束宽度。
/// @param offset_deg 目标相对当前波束中心的离轴角。
/// @param scan_center_deg 当前扫描中心方向。
/// @return 方向图采样结果。
/// @note 主瓣内返回峰值增益扣除主瓣衰减与扫描损失；
///       主瓣外返回固定旁瓣电平；
///       后瓣区域返回固定后瓣电平。
inline AntennaPatternSample EvaluateAntennaPattern(
    float peak_gain_dbi,
    const AntennaPatternConfig& config,
    const AntennaPatternBeamwidthDeg& beamwidth_deg,
    const AntennaLookOffsetDeg& offset_deg,
    const AzimuthElevationDeg& scan_center_deg) {
  AntennaPatternSample sample;
  sample.scan_loss_db = ComputeScanLossDb(config, scan_center_deg);
  sample.inside_back_lobe =
      antenna_pattern_internal::IsInsideBackLobe(offset_deg);
  sample.inside_main_lobe = !sample.inside_back_lobe &&
                            IsInsideMainLobe(beamwidth_deg, offset_deg);
  if (sample.inside_back_lobe) {
    sample.gain_dbi = peak_gain_dbi + config.backlobe_level_db -
                      sample.scan_loss_db;
    return sample;
  }

  sample.main_lobe_attenuation_db =
      ComputeMainLobeAttenuationDb(config, beamwidth_deg, offset_deg);
  if (sample.inside_main_lobe) {
    sample.gain_dbi =
        peak_gain_dbi - sample.main_lobe_attenuation_db - sample.scan_loss_db;
    return sample;
  }

  sample.gain_dbi = peak_gain_dbi + config.max_sidelobe_level_db -
                    sample.scan_loss_db;
  return sample;
}

}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_ANTENNA_PATTERN_UTILS_H_
