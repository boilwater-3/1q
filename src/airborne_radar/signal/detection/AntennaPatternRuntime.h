/**
 * @file AntennaPatternRuntime.h
 * @brief 定义方向图运行期中间量与评估函数（私有实现头）。
 * @note 本文件仅供 `signal::detection` 链路内部使用，不作为公开 API。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_ANTENNA_PATTERN_RUNTIME_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_ANTENNA_PATTERN_RUNTIME_H_

#include <cmath>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/utils/MathUtils.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief AntennaPatternBeamwidthDeg 表示方向图评估使用的有效波束宽度。
 * @note 该类型用于 `BeamControlResolver` 向方向图评估函数传递运行期派生波束宽度。
 */
struct AntennaPatternBeamwidthDeg {
  float az_beamwidth_deg{3.0f}; /**< 有效方位波束宽度（单位：deg） */
  float el_beamwidth_deg{3.0f}; /**< 有效俯仰波束宽度（单位：deg） */
};

/**
 * @brief AntennaLookOffsetDeg 表示目标相对当前波束中心的离轴角。
 * @note 该类型用于方向图评估阶段的运行期中间量，不表示顶层配置项。
 */
struct AntennaLookOffsetDeg {
  float delta_az_deg{0.0f}; /**< 方位离轴角（单位：deg） */
  float delta_el_deg{0.0f}; /**< 俯仰离轴角（单位：deg） */
};

/**
 * @brief AntennaPatternSample 表示方向图采样结果。
 * @note 该类型用于返回方向图评估的中间结果，不应由外部直接构造参与配置。
 */
struct AntennaPatternSample {
  float gain_dbi{0.0f};                 /**< 方向图总增益（单位：dBi） */
  float main_lobe_attenuation_db{0.0f}; /**< 主瓣离轴衰减（单位：dB） */
  float scan_loss_db{0.0f};             /**< 扫描损失（单位：dB） */
  bool inside_main_lobe{false};         /**< 是否位于主瓣 */
  bool inside_back_lobe{false};         /**< 是否位于后瓣 */
};

namespace antenna_pattern_internal {

inline float ClampLowerBound(float value, float min_value) {
  return value < min_value ? min_value : value;
}

/**
 * @brief 判断离轴角是否进入后瓣区域。
 * @param[in] offset_deg 目标相对当前波束中心的离轴角。
 * @return 任一轴绝对离轴角超过 90 度时返回 true。
 */
inline bool IsInsideBackLobe(const AntennaLookOffsetDeg& offset_deg) {
  return std::fabs(offset_deg.delta_az_deg) > 90.0f || std::fabs(offset_deg.delta_el_deg) > 90.0f;
}

}  // namespace antenna_pattern_internal

/**
 * @brief 判断目标是否落入主瓣范围。
 * @param[in] beamwidth_deg 用于评估的有效波束宽度。
 * @param[in] offset_deg 目标相对当前波束中心的离轴角。
 * @return 方位和俯仰离轴角均不超过半功率波束半宽时返回 true。
 */
inline bool IsInsideMainLobe(const AntennaPatternBeamwidthDeg& beamwidth_deg,
                             const AntennaLookOffsetDeg& offset_deg) {
  const float half_az_beamwidth_deg =
      0.5f * antenna_pattern_internal::ClampLowerBound(beamwidth_deg.az_beamwidth_deg, 1e-3f);
  const float half_el_beamwidth_deg =
      0.5f * antenna_pattern_internal::ClampLowerBound(beamwidth_deg.el_beamwidth_deg, 1e-3f);
  return std::fabs(offset_deg.delta_az_deg) <= half_az_beamwidth_deg &&
         std::fabs(offset_deg.delta_el_deg) <= half_el_beamwidth_deg;
}

/**
 * @brief 计算主瓣离轴衰减。
 * @param[in] config 天线方向图配置。
 * @param[in] beamwidth_deg 用于评估的有效波束宽度。
 * @param[in] offset_deg 目标相对当前波束中心的离轴角。
 * @return 主瓣离轴衰减（单位：dB）。
 */
inline float ComputeMainLobeAttenuationDb(const config::engineering::AntennaPatternConfig& config,
                                          const AntennaPatternBeamwidthDeg& beamwidth_deg,
                                          const AntennaLookOffsetDeg& offset_deg) {
  const float half_az_beamwidth_deg =
      0.5f * antenna_pattern_internal::ClampLowerBound(beamwidth_deg.az_beamwidth_deg, 1e-3f);
  const float half_el_beamwidth_deg =
      0.5f * antenna_pattern_internal::ClampLowerBound(beamwidth_deg.el_beamwidth_deg, 1e-3f);
  const float normalized_az = std::fabs(offset_deg.delta_az_deg) / half_az_beamwidth_deg;
  const float normalized_el = std::fabs(offset_deg.delta_el_deg) / half_el_beamwidth_deg;

  switch (config.model_type) {
    case config::engineering::AntennaPatternModelType::kParabolicMainLobe:
      return 3.0f * (normalized_az * normalized_az + normalized_el * normalized_el);

    case config::engineering::AntennaPatternModelType::kCosinePower: {
      const float kDeg2Rad = 3.14159265358979f / 180.0f;
      const float az_offset_rad =
          utils::ClampFloat(offset_deg.delta_az_deg, -89.9f, 89.9f) * kDeg2Rad;
      const float el_offset_rad =
          utils::ClampFloat(offset_deg.delta_el_deg, -89.9f, 89.9f) * kDeg2Rad;
      const float az_half_bw_rad = half_az_beamwidth_deg * kDeg2Rad;
      const float el_half_bw_rad = half_el_beamwidth_deg * kDeg2Rad;
      const float az_denominator =
          std::log(antenna_pattern_internal::ClampLowerBound(std::cos(az_half_bw_rad), 1e-6f));
      const float el_denominator =
          std::log(antenna_pattern_internal::ClampLowerBound(std::cos(el_half_bw_rad), 1e-6f));
      const float az_power = -0.69314718055995f / az_denominator;
      const float el_power = -0.69314718055995f / el_denominator;
      const float az_gain = std::pow(
          antenna_pattern_internal::ClampLowerBound(std::cos(az_offset_rad), 1e-6f), az_power);
      const float el_gain = std::pow(
          antenna_pattern_internal::ClampLowerBound(std::cos(el_offset_rad), 1e-6f), el_power);
      return -10.0f *
             std::log10(antenna_pattern_internal::ClampLowerBound(az_gain * el_gain, 1e-6f));
    }

    case config::engineering::AntennaPatternModelType::kGaussianMainLobe:
    default:
      return 3.0f * (normalized_az * normalized_az + normalized_el * normalized_el);
  }
}

/**
 * @brief 计算扫描损失。
 * @param[in] config 天线方向图配置。
 * @param[in] beam_pointing_deg 当前波束指向方向。
 * @return 扫描损失（单位：dB）。
 */
inline float ComputeScanLossDb(const config::engineering::AntennaPatternConfig& config,
                               const model::AzimuthElevationDeg& beam_pointing_deg) {
  const float delta_scan_az_deg = beam_pointing_deg.az_deg - config.boresight_offset_deg.az_deg;
  const float delta_scan_el_deg = beam_pointing_deg.el_deg - config.boresight_offset_deg.el_deg;
  const float raw_scan_loss_db =
      config.scan_loss_coeff_db_per_deg2 *
      (delta_scan_az_deg * delta_scan_az_deg + delta_scan_el_deg * delta_scan_el_deg);
  return utils::ClampFloat(
      raw_scan_loss_db, 0.0f,
      antenna_pattern_internal::ClampLowerBound(config.max_scan_loss_db, 0.0f));
}

/**
 * @brief 评估指定方向上的天线方向图结果。
 * @param[in] peak_gain_dbi 波束中心峰值增益（单位：dBi）。
 * @param[in] config 天线方向图配置。
 * @param[in] beamwidth_deg 用于评估的有效波束宽度。
 * @param[in] offset_deg 目标相对当前波束中心的离轴角。
 * @param[in] beam_pointing_deg 当前波束指向方向。
 * @return 方向图采样结果。
 */
inline AntennaPatternSample EvaluateAntennaPattern(
    float peak_gain_dbi, const config::engineering::AntennaPatternConfig& config,
    const AntennaPatternBeamwidthDeg& beamwidth_deg, const AntennaLookOffsetDeg& offset_deg,
    const model::AzimuthElevationDeg& beam_pointing_deg) {
  AntennaPatternSample sample;
  sample.scan_loss_db = ComputeScanLossDb(config, beam_pointing_deg);
  sample.inside_back_lobe = antenna_pattern_internal::IsInsideBackLobe(offset_deg);
  sample.inside_main_lobe = !sample.inside_back_lobe && IsInsideMainLobe(beamwidth_deg, offset_deg);
  if (sample.inside_back_lobe) {
    sample.gain_dbi = peak_gain_dbi + config.backlobe_level_db - sample.scan_loss_db;
    return sample;
  }

  sample.main_lobe_attenuation_db = ComputeMainLobeAttenuationDb(config, beamwidth_deg, offset_deg);
  if (sample.inside_main_lobe) {
    sample.gain_dbi = peak_gain_dbi - sample.main_lobe_attenuation_db - sample.scan_loss_db;
    return sample;
  }

  sample.gain_dbi = peak_gain_dbi + config.max_sidelobe_level_db - sample.scan_loss_db;
  return sample;
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_ANTENNA_PATTERN_RUNTIME_H_
