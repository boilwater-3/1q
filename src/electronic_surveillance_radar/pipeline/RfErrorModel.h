/**
 * @file RfErrorModel.h
 * @brief 定义电子侦察载频测量误差模型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_RF_ERROR_MODEL_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_RF_ERROR_MODEL_H_

#include <algorithm>
#include <cmath>
#include <random>

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief RfErrorModelConfig 描述载频测量误差模型参数。
 *
 * 载频测量精度由接收机当前调谐通道带宽（频率分辨率）驱动，而非载频本身；
 * scale 取调谐通道带宽，单位 Hz。
 */
struct RfErrorModelConfig {
  float coefficient{0.5f}; /**< 误差模型系数 */
  float min_std_hz{1.0f};  /**< 最小标准差（单位：Hz） */
  float max_std_hz{1.0e7f}; /**< 最大标准差（单位：Hz） */
};

/**
 * @brief RfErrorModel 提供载频测量标准差计算与误差采样能力。
 */
class RfErrorModel final {
 public:
  /**
   * @brief 计算载频测量误差标准差。
   * @param[in] snr_db 信噪比（单位：dB）。
   * @param[in] channel_bandwidth_hz 接收机当前调谐通道带宽（单位：Hz），作为误差尺度。
   * @param[in] config 模型配置。
   * @return 载频测量误差标准差（单位：Hz）。
   */
  static float ComputeStdDevHz(float snr_db, float channel_bandwidth_hz,
                                const RfErrorModelConfig& config) {
    if (!std::isfinite(snr_db) || !std::isfinite(channel_bandwidth_hz) ||
        channel_bandwidth_hz <= 0.0f) {
      return config.max_std_hz;
    }
    const double clamped_snr_db = std::max(-100.0, std::min(static_cast<double>(snr_db), 100.0));
    const double snr_linear = std::pow(10.0, clamped_snr_db / 10.0);
    const double effective_snr_linear = std::max(1.0e-6, snr_linear - 1.0);
    const double base_std_hz = static_cast<double>(config.coefficient) *
                                static_cast<double>(channel_bandwidth_hz) /
                                std::sqrt(effective_snr_linear);
    const double low_snr_inflation = 1.0 + 1.0 / std::sqrt(effective_snr_linear + 1.0);
    float std_dev = static_cast<float>(base_std_hz * low_snr_inflation);
    std_dev = std::max(config.min_std_hz, std::min(config.max_std_hz, std_dev));
    return std_dev;
  }

  /**
   * @brief 采样载频测量误差。
   * @param[in] snr_db 信噪比（单位：dB）。
   * @param[in] channel_bandwidth_hz 接收机当前调谐通道带宽（单位：Hz）。
   * @param[in,out] rng 随机数引擎。
   * @param[in] config 模型配置。
   * @return 采样载频误差（单位：Hz）。
   * @warning `rng` 为空时返回 0，调用方应保证随机数引擎有效。
   */
  static float SampleErrorHz(float snr_db, float channel_bandwidth_hz, std::mt19937* rng,
                              const RfErrorModelConfig& config) {
    if (rng == nullptr) {
      return 0.0f;
    }
    const float std_dev = ComputeStdDevHz(snr_db, channel_bandwidth_hz, config);
    std::normal_distribution<float> distribution(0.0f, std_dev);
    const float raw_sample = distribution(*rng);
    const float clip_bound = std::max(config.min_std_hz, 3.0f * std_dev);
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

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_RF_ERROR_MODEL_H_
