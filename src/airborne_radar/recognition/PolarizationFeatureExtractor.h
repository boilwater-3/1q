/**
 * @file PolarizationFeatureExtractor.h
 * @brief 双通道极化特征提取器（识别专用）。
 */

#ifndef AIRBORNE_RADAR_RECOGNITION_POLARIZATION_FEATURE_EXTRACTOR_H_
#define AIRBORNE_RADAR_RECOGNITION_POLARIZATION_FEATURE_EXTRACTOR_H_

#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/recognition/RecognitionTypes.h"

namespace airborne_radar {
namespace recognition {

/**
 * @brief PolarizationFeatureExtractor 从双通道极化 RCS 样本构造效能化极化观测。
 *
 * 两通道 RCS 代入同一雷达方程生成接收端线性能量（比值与发射/增益/距离
 * 无关），叠加由 SNR 决定的噪声底；能量和按距离换算到统一参考条件。
 * 任一通道缺失或视线角超出覆盖时维度整体无效。
 */
class PolarizationFeatureExtractor {
 public:
  /**
   * @brief 提取极化特征观测。
   * @param[in] samples 双通道极化 RCS 样本列表。
   * @param[in] look_az_deg 视线方位角（目标参考系，deg）。
   * @param[in] look_el_deg 视线俯仰角（目标参考系，deg）。
   * @param[in] snr_db 周期信噪比（dB）。
   * @param[in] range_m 目标斜距（m）。
   * @return 极化观测；样本为空或视线角超出覆盖时 valid=false。
   */
  static PolarizationObservation Extract(
      const std::vector<session::PolarizationRcsSample>& samples, float look_az_deg,
      float look_el_deg, float snr_db, float range_m);
};

}  // namespace recognition
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RECOGNITION_POLARIZATION_FEATURE_EXTRACTOR_H_
