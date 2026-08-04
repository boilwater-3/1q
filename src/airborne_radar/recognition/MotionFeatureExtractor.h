/**
 * @file MotionFeatureExtractor.h
 * @brief 运动特征提取器（识别专用）。
 */

#ifndef AIRBORNE_RADAR_RECOGNITION_MOTION_FEATURE_EXTRACTOR_H_
#define AIRBORNE_RADAR_RECOGNITION_MOTION_FEATURE_EXTRACTOR_H_

#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/recognition/RecognitionTypes.h"

namespace airborne_radar {
namespace recognition {

/**
 * @brief MotionFeatureExtractor 从滤波航迹快照构造运动特征观测。
 *
 * 速度/加速度直接消费滤波快照字段；绝对高度由平台高度与雷达局部
 * position_z 换算；转弯半径由横向加速度分解得到，横向加速度低于阈值
 * 时记为直线飞行。质量因子由估计不确定度（预测协方差 P 的 position
 * 分块迹）归一化——不确定度越小质量越高。
 */
class MotionFeatureExtractor {
 public:
  /**
   * @brief 提取运动特征观测。
   * @param[in] snapshot 轨迹快照（须为已确认航迹，否则维度无效）。
   * @param[in] platform_altitude_m 平台绝对高度（m）。
   * @param[in] estimation_uncertainty_trace 预测协方差 P 的 position 分块迹（m²）。
   * @return 运动观测；快照非已确认时 valid=false。
   */
  static MotionObservation Extract(const session::TrackStateSnapshot& snapshot,
                                   float platform_altitude_m, float estimation_uncertainty_trace);
};

}  // namespace recognition
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RECOGNITION_MOTION_FEATURE_EXTRACTOR_H_
