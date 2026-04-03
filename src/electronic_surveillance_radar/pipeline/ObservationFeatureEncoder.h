/**
 * @file ObservationFeatureEncoder.h
 * @brief 定义 ESR 观测特征编码与距离计算工具。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_FEATURE_ENCODER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_FEATURE_ENCODER_H_

#include <cmath>
#include <cstddef>

#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

/**
 * @brief ObservationFeatureEncoder 负责观测到特征向量的映射。
 */
class ObservationFeatureEncoder final {
 public:
  /**
   * @brief 编码单条观测记录。
   * @param[in] observation 输入观测。
   * @param[in] scales 特征尺度。
   * @return 特征向量。
   */
  static ObservationFeatureVector Encode(const common::EmitterObservation& observation,
                                         const ObservationFeatureScales& scales) {
    ObservationFeatureVector feature;
    const double rf_scale = ResolveScale(scales.rf_scale_hz, 1.0);
    const double pw_scale = ResolveScale(scales.pw_scale_sec, 1.0e-6);
    const double az_scale = ResolveScale(scales.az_scale_deg, 1.0);
    const double el_scale = ResolveScale(scales.el_scale_deg, 1.0);
    const double snr_scale = ResolveScale(scales.snr_scale_db, 1.0);

    feature.values[0] = static_cast<float>(observation.rf_hz / rf_scale);
    feature.values[1] = static_cast<float>(observation.pulse_width_s / pw_scale);
    feature.values[2] = static_cast<float>(static_cast<double>(observation.aoa_az_deg) / az_scale);
    feature.values[3] = static_cast<float>(static_cast<double>(observation.aoa_el_deg) / el_scale);
    feature.values[4] = static_cast<float>(static_cast<double>(observation.snr_db) / snr_scale);
    return feature;
  }

  /**
   * @brief 计算两个特征向量的欧氏距离。
   * @param[in] lhs 左侧特征向量。
   * @param[in] rhs 右侧特征向量。
   * @return 欧氏距离。
   */
  static float Distance(const ObservationFeatureVector& lhs, const ObservationFeatureVector& rhs) {
    float sum_sq = 0.0f;
    for (std::size_t i = 0; i < kObservationFeatureDimension; ++i) {
      const float diff = lhs.values[i] - rhs.values[i];
      sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq);
  }

 private:
  static double ResolveScale(float value, double fallback) {
    return (std::isfinite(value) != 0 && value > 0.0f) ? static_cast<double>(value) : fallback;
  }
};

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_FEATURE_ENCODER_H_
