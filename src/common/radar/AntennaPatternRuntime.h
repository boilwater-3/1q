/**
 * @file AntennaPatternRuntime.h
 * @brief 定义天线方向图运行期中间量与评估函数（common 单源，header-only）。
 */

#ifndef COMMON_RADAR_ANTENNA_PATTERN_RUNTIME_H_
#define COMMON_RADAR_ANTENNA_PATTERN_RUNTIME_H_

#include <algorithm>
#include <cmath>

namespace oneq {
namespace common {
namespace radar {
namespace antenna_pattern_internal {

inline float ClampFloat(float value, float min_value, float max_value) {
  return value < min_value ? min_value : (value > max_value ? max_value : value);
}

}  // namespace antenna_pattern_internal

/**
 * @brief 天线方向图模型类型（通用枚举）。
 */
enum class AntennaPatternModelType {
  kGaussianMainLobe = 0,  /**< 高斯主瓣近似。 */
  kParabolicMainLobe = 1, /**< 抛物线主瓣近似。 */
  kCosinePower = 2,       /**< 余弦幂方向图近似。 */
  kSincPattern = 3        /**< sinc² 方向图（均匀孔径理论解，需物理孔径尺寸）。 */
};

/**
 * @brief 方位/俯仰角（单位：deg）。
 */
struct AzimuthElevationDeg {
  float az_deg{0.0f};
  float el_deg{0.0f};

  AzimuthElevationDeg() = default;
  AzimuthElevationDeg(float az_deg_in, float el_deg_in)
      : az_deg(az_deg_in), el_deg(el_deg_in) {}
};

/**
 * @brief 天线方向图参数（common 简单配置，不依赖模块 config）。
 */
struct AntennaPatternConfig {
  AntennaPatternModelType model_type{AntennaPatternModelType::kGaussianMainLobe};
  float max_sidelobe_level_db{-20.0f};
  float backlobe_level_db{-35.0f};
  float scan_loss_coeff_db_per_deg2{0.0f};
  float max_scan_loss_db{6.0f};
  AzimuthElevationDeg boresight_offset_deg{};
};


/**
 * @brief AntennaPatternBeamwidthDeg 表示方向图评估使用的有效波束宽度。
 * @note 该类型用于 `BeamControlResolver` 向方向图评估函数传递运行期派生波束宽度。
 */
struct AntennaPatternBeamwidthDeg {
  float az_beamwidth_deg{3.0f}; /**< 有效方位波束宽度（单位：deg） */
  float el_beamwidth_deg{3.0f}; /**< 有效俯仰波束宽度（单位：deg） */

  AntennaPatternBeamwidthDeg() = default;
  AntennaPatternBeamwidthDeg(float az_beamwidth_deg_in, float el_beamwidth_deg_in)
      : az_beamwidth_deg(az_beamwidth_deg_in), el_beamwidth_deg(el_beamwidth_deg_in) {}
};

/**
 * @brief AntennaLookOffsetDeg 表示目标相对当前波束中心的离轴角。
 * @note 该类型用于方向图评估阶段的运行期中间量，不表示顶层配置项。
 */
struct AntennaLookOffsetDeg {
  float delta_az_deg{0.0f}; /**< 方位离轴角（单位：deg） */
  float delta_el_deg{0.0f}; /**< 俯仰离轴角（单位：deg） */

  AntennaLookOffsetDeg() = default;
  AntennaLookOffsetDeg(float delta_az_deg_in, float delta_el_deg_in)
      : delta_az_deg(delta_az_deg_in), delta_el_deg(delta_el_deg_in) {}
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

inline float NormalizeAzimuthDeltaDeg(float delta_az_deg) {
  return std::remainder(delta_az_deg, 360.0f);
}

/**
 * @brief 判断离轴角是否进入后瓣区域。
 * @param[in] offset_deg 目标相对当前波束中心的离轴角。
 * @return 任一轴绝对离轴角超过 90 度时返回 true。
 */
inline bool IsInsideBackLobe(const AntennaLookOffsetDeg& offset_deg) {
  return std::fabs(NormalizeAzimuthDeltaDeg(offset_deg.delta_az_deg)) > 90.0f ||
         std::fabs(offset_deg.delta_el_deg) > 90.0f;
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
  const float delta_az_deg =
      antenna_pattern_internal::NormalizeAzimuthDeltaDeg(offset_deg.delta_az_deg);
  return std::fabs(delta_az_deg) <= half_az_beamwidth_deg &&
         std::fabs(offset_deg.delta_el_deg) <= half_el_beamwidth_deg;
}

/**
 * @brief 计算主瓣离轴衰减。
 * @param[in] config 天线方向图配置。
 * @param[in] beamwidth_deg 用于评估的有效波束宽度。
 * @param[in] offset_deg 目标相对当前波束中心的离轴角。
 * @return 主瓣离轴衰减（单位：dB）。
 */
inline float ComputeMainLobeAttenuationDb(const AntennaPatternConfig& config,
                                          const AntennaPatternBeamwidthDeg& beamwidth_deg,
                                          const AntennaLookOffsetDeg& offset_deg,
                                          float antenna_az_length_m = 0.0f,
                                          float antenna_el_width_m = 0.0f,
                                          float wavelength_m = 0.0f) {
  const float half_az_beamwidth_deg =
      0.5f * antenna_pattern_internal::ClampLowerBound(beamwidth_deg.az_beamwidth_deg, 1e-3f);
  const float half_el_beamwidth_deg =
      0.5f * antenna_pattern_internal::ClampLowerBound(beamwidth_deg.el_beamwidth_deg, 1e-3f);
  const float normalized_az =
      std::fabs(antenna_pattern_internal::NormalizeAzimuthDeltaDeg(offset_deg.delta_az_deg)) /
      half_az_beamwidth_deg;
  const float normalized_el = std::fabs(offset_deg.delta_el_deg) / half_el_beamwidth_deg;

  switch (config.model_type) {
    case AntennaPatternModelType::kParabolicMainLobe:
      return 3.0f * (normalized_az * normalized_az + normalized_el * normalized_el);

    case AntennaPatternModelType::kCosinePower: {
      const float kDeg2Rad = 3.14159265358979f / 180.0f;
      const float az_offset_rad =
          antenna_pattern_internal::ClampFloat(
              antenna_pattern_internal::NormalizeAzimuthDeltaDeg(offset_deg.delta_az_deg), -89.9f,
              89.9f) *
          kDeg2Rad;
      const float el_offset_rad =
          antenna_pattern_internal::ClampFloat(offset_deg.delta_el_deg, -89.9f, 89.9f) * kDeg2Rad;
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

    case AntennaPatternModelType::kSincPattern: {
      float attenuation_db = 0.0f;
      if (antenna_az_length_m > 0.0f && wavelength_m > 0.0f) {
        const float kDeg2Rad = 3.14159265358979f / 180.0f;
        const float az_offset_rad =
            antenna_pattern_internal::NormalizeAzimuthDeltaDeg(offset_deg.delta_az_deg) * kDeg2Rad;
        const float arg =
            3.14159265358979f * antenna_az_length_m * std::sin(az_offset_rad) / wavelength_m;
        const float sinc_val = (std::fabs(arg) < 1e-6f) ? 1.0f : std::sin(arg) / arg;
        const float safe_val =
            antenna_pattern_internal::ClampLowerBound(std::fabs(sinc_val), 1e-6f);
        attenuation_db += -20.0f * std::log10(safe_val);
      }
      if (antenna_el_width_m > 0.0f && wavelength_m > 0.0f) {
        const float kDeg2Rad = 3.14159265358979f / 180.0f;
        const float el_offset_rad = offset_deg.delta_el_deg * kDeg2Rad;
        const float arg =
            3.14159265358979f * antenna_el_width_m * std::sin(el_offset_rad) / wavelength_m;
        const float sinc_val = (std::fabs(arg) < 1e-6f) ? 1.0f : std::sin(arg) / arg;
        const float safe_val =
            antenna_pattern_internal::ClampLowerBound(std::fabs(sinc_val), 1e-6f);
        attenuation_db += -20.0f * std::log10(safe_val);
      }
      return attenuation_db;
    }

    case AntennaPatternModelType::kGaussianMainLobe:
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
inline float ComputeScanLossDb(const AntennaPatternConfig& config,
                               const AzimuthElevationDeg& beam_pointing_deg) {
  const float delta_scan_az_deg = antenna_pattern_internal::NormalizeAzimuthDeltaDeg(
      beam_pointing_deg.az_deg - config.boresight_offset_deg.az_deg);
  const float delta_scan_el_deg = beam_pointing_deg.el_deg - config.boresight_offset_deg.el_deg;
  const float raw_scan_loss_db =
      config.scan_loss_coeff_db_per_deg2 *
      (delta_scan_az_deg * delta_scan_az_deg + delta_scan_el_deg * delta_scan_el_deg);
  return antenna_pattern_internal::ClampFloat(
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
    float peak_gain_dbi, const AntennaPatternConfig& config,
    const AntennaPatternBeamwidthDeg& beamwidth_deg, const AntennaLookOffsetDeg& offset_deg,
    const AzimuthElevationDeg& beam_pointing_deg,
    float antenna_az_length_m = 0.0f, float antenna_el_width_m = 0.0f,
    float wavelength_m = 0.0f) {
  AntennaPatternSample sample;
  sample.scan_loss_db = ComputeScanLossDb(config, beam_pointing_deg);
  sample.inside_back_lobe = antenna_pattern_internal::IsInsideBackLobe(offset_deg);
  sample.inside_main_lobe = !sample.inside_back_lobe && IsInsideMainLobe(beamwidth_deg, offset_deg);
  if (sample.inside_back_lobe) {
    sample.gain_dbi = peak_gain_dbi + config.backlobe_level_db - sample.scan_loss_db;
    return sample;
  }

  sample.main_lobe_attenuation_db =
      ComputeMainLobeAttenuationDb(config, beamwidth_deg, offset_deg, antenna_az_length_m,
                                   antenna_el_width_m, wavelength_m);
  if (sample.inside_main_lobe) {
    sample.gain_dbi = peak_gain_dbi - sample.main_lobe_attenuation_db - sample.scan_loss_db;
    return sample;
  }

  sample.gain_dbi = peak_gain_dbi + config.max_sidelobe_level_db - sample.scan_loss_db;
  return sample;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_ANTENNA_PATTERN_RUNTIME_H_
