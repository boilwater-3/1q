/**
 * @file PulseWidthErrorModel.h
 * @brief 定义电子侦察脉宽测量误差模型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_PULSE_WIDTH_ERROR_MODEL_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_PULSE_WIDTH_ERROR_MODEL_H_

#include <algorithm>
#include <cmath>
#include <random>

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief PulseWidthErrorModelConfig 描述脉宽测量误差模型参数。
 *
 * 脉宽测量误差随信号自身脉宽缩放；scale 取信号 pulse_width_s，单位 s。
 * 仅适用于脉冲串（kPulseTrain）波形——连续/噪声/扫频无脉宽可测。
 */
struct PulseWidthErrorModelConfig {
  float coefficient{0.5f};   /**< 误差模型系数 */
  float min_std_s{1.0e-12f}; /**< 最小标准差（单位：s） */
  float max_std_s{1.0e-3f};  /**< 最大标准差（单位：s） */
};

/**
 * @brief PulseWidthErrorModel 提供脉宽测量标准差计算与误差采样能力。
 */
class PulseWidthErrorModel final {
 public:
  /**
   * @brief 计算脉宽测量误差标准差。
   * @param[in] snr_db 信噪比（单位：dB）。
   * @param[in] pulse_width_s 信号脉宽（单位：s），作为误差尺度。
   * @param[in] config 模型配置。
   * @return 脉宽测量误差标准差（单位：s）。
   */
  static float ComputeStdDevS(float snr_db, float pulse_width_s,
                               const PulseWidthErrorModelConfig& config) {
    if (!std::isfinite(snr_db) || !std::isfinite(pulse_width_s) || pulse_width_s <= 0.0f) {
      return config.max_std_s;
    }
    const double clamped_snr_db = std::max(-100.0, std::min(static_cast<double>(snr_db), 100.0));
    const double snr_linear = std::pow(10.0, clamped_snr_db / 10.0);
    const double effective_snr_linear = std::max(1.0e-6, snr_linear - 1.0);
    const double base_std_s = static_cast<double>(config.coefficient) *
                               static_cast<double>(pulse_width_s) /
                               std::sqrt(effective_snr_linear);
    const double low_snr_inflation = 1.0 + 1.0 / std::sqrt(effective_snr_linear + 1.0);
    float std_dev = static_cast<float>(base_std_s * low_snr_inflation);
    std_dev = std::max(config.min_std_s, std::min(config.max_std_s, std_dev));
    return std_dev;
  }

  /**
   * @brief 采样脉宽测量误差。
   * @param[in] snr_db 信噪比（单位：dB）。
   * @param[in] pulse_width_s 信号脉宽（单位：s）。
   * @param[in,out] rng 随机数引擎。
   * @param[in] config 模型配置。
   * @return 采样脉宽误差（单位：s）。
   * @warning `rng` 为空时返回 0，调用方应保证随机数引擎有效。
   */
  static float SampleErrorS(float snr_db, float pulse_width_s, std::mt19937* rng,
                             const PulseWidthErrorModelConfig& config) {
    if (rng == nullptr) {
      return 0.0f;
    }
    const float std_dev = ComputeStdDevS(snr_db, pulse_width_s, config);
    std::normal_distribution<float> distribution(0.0f, std_dev);
    const float raw_sample = distribution(*rng);
    const float clip_bound = std::max(config.min_std_s, 3.0f * std_dev);
    if (raw_sample > clip_bound) {
      return clip_bound;
    }
    if (raw_sample < -clip_bound) {
      return -clip_bound;
    }
    return raw_sample;
  }
};

}  // namespace intercept
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_PULSE_WIDTH_ERROR_MODEL_H_
