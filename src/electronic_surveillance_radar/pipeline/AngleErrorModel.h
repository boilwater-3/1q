/**
 * @file AngleErrorModel.h
 * @brief 定义电子侦察测角误差模型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_ANGLE_ERROR_MODEL_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_ANGLE_ERROR_MODEL_H_

#include <algorithm>
#include <cmath>
#include <random>

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief AngleErrorModelConfig 描述测角误差模型参数。
 */
struct AngleErrorModelConfig {
  float coefficient{0.51f}; /**< 误差模型系数 */
  float min_std_deg{0.01f}; /**< 最小标准差（单位：deg） */
  float max_std_deg{20.0f}; /**< 最大标准差（单位：deg） */
};

/**
 * @brief AngleErrorModel 提供测角标准差计算与误差采样能力。
 */
class AngleErrorModel final {
 public:
  /**
   * @brief 计算角误差标准差。
   * @param[in] snr_db 信噪比（单位：dB）。
   * @param[in] beamwidth_deg 波束宽度（单位：deg）。
   * @param[in] config 模型配置。
   * @return 角误差标准差（单位：deg）。
   */
  static float ComputeStdDevDeg(float snr_db, float beamwidth_deg,
                                const AngleErrorModelConfig& config) {
    if (!std::isfinite(snr_db) || !std::isfinite(beamwidth_deg) || beamwidth_deg <= 0.0f) {
      return config.max_std_deg;
    }
    const double clamped_snr_db = std::max(-100.0, std::min(static_cast<double>(snr_db), 100.0));
    const double snr_linear = std::pow(10.0, clamped_snr_db / 10.0);
    const double effective_snr_linear = std::max(1.0e-6, snr_linear - 1.0);
    const double base_std_deg = static_cast<double>(config.coefficient) *
                                static_cast<double>(beamwidth_deg) /
                                std::sqrt(effective_snr_linear);
    const double low_snr_inflation = 1.0 + 1.0 / std::sqrt(effective_snr_linear + 1.0);
    float std_dev = static_cast<float>(base_std_deg * low_snr_inflation);
    std_dev = std::max(config.min_std_deg, std::min(config.max_std_deg, std_dev));
    return std_dev;
  }

  /**
   * @brief 采样角误差。
   * @param[in] snr_db 信噪比（单位：dB）。
   * @param[in] beamwidth_deg 波束宽度（单位：deg）。
   * @param[in,out] rng 随机数引擎。
   * @param[in] config 模型配置。
   * @return 采样角误差（单位：deg）。
   * @warning `rng` 为空时返回 0，调用方应保证随机数引擎有效。
   */
  static float SampleErrorDeg(float snr_db, float beamwidth_deg, std::mt19937* rng,
                              const AngleErrorModelConfig& config) {
    if (rng == nullptr) {
      return 0.0f;
    }
    const float std_dev = ComputeStdDevDeg(snr_db, beamwidth_deg, config);
    std::normal_distribution<float> distribution(0.0f, std_dev);
    const float raw_sample = distribution(*rng);
    const float clip_bound = std::max(config.min_std_deg, 3.0f * std_dev);
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

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_ANGLE_ERROR_MODEL_H_
