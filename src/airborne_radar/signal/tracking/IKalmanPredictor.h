/**
 * @file IKalmanPredictor.h
 * @brief 定义基于 Kalman 滤波的状态预测器抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_PREDICTOR_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_PREDICTOR_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief Kalman 预测器配置。
 */
struct KalmanPredictorConfig {
/**
 * @brief 过程噪声扩散系数 q（单位：m/s²），建模加速度白噪声强度。
 * @details 对应 Stone Soup ConstantVelocity 的 noise_diff_coeff。
 *          值越大表示目标运动越不确定，协方差增长越快。
 */
  float noise_diff_coeff{1.0f};
};
/**
 * @brief Kalman 预测器抽象接口。
 */
class IKalmanPredictor {
 public:
  virtual ~IKalmanPredictor() = default;
/**
 * @brief 对先验状态执行时间外推预测。
 * @param prior 先验高斯状态。
 * @param dt 预测时间步长（秒）。
 * @return 预测后的高斯状态。
 */
  virtual GaussianTrackState Predict(const GaussianTrackState &prior,
                                     float dt) const = 0;
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_PREDICTOR_H_
