/**
 * @file KalmanPredictor.h
 * @brief 定义基于 Kalman 滤波的状态预测器接口与恒速模型实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_PREDICTOR_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_PREDICTOR_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief 3D 恒速（Constant Velocity）Kalman 预测器。
 * @details 参考 Stone Soup CombinedLinearGaussianTransitionModel(3×ConstantVelocity)。
 *          状态向量布局 [x, vx, y, vy, z, vz]，每轴独立的 2D CV 模型通过
 *          block_diag 组合。
 *
 *          单轴转移矩阵 F_1d：
 *          | 1  dt |
 *          | 0   1 |
 *
 *          单轴过程噪声 Q_1d（连续白噪声加速度离散化）：
 *          | dt³/3  dt²/2 |   × q
 *          | dt²/2  dt    |
 */
class KalmanPredictor final : public IKalmanPredictor {
 public:
/**
 * @brief 构造函数。
 * @param config 预测器配置。
 */
  explicit KalmanPredictor(KalmanPredictorConfig config = {});
/**
 * @brief 对先验状态执行恒速模型预测。
 * @param prior 先验高斯状态。
 * @param dt 预测时间步长（秒），应由调用方保证 > 0。
 * @note 当前实现不做 dt 的运行时校验。
 * @return 预测后的高斯状态（含传播后的协方差）。
 */
  GaussianTrackState Predict(const GaussianTrackState &prior,
                             float dt) const override;
/**
 * @brief 更新配置。
 * @param config 新的预测器配置。
 */
  void UpdateConfig(KalmanPredictorConfig config);
/**
 * @brief 构建 6×6 状态转移矩阵 F。
 * @param dt 时间步长（秒）。
 * @return 3D 恒速模型的块对角转移矩阵。
 */
  static TransitionMatrix BuildTransitionMatrix(float dt);
/**
 * @brief 构建 6×6 过程噪声协方差矩阵 Q。
 * @param dt 时间步长（秒）。
 * @param q 噪声扩散系数。
 * @return 3D 恒速模型的块对角过程噪声矩阵。
 */
  static ProcessNoiseCovariance BuildProcessNoise(float dt, float q);

 private:
/**
 * @brief 当前配置。
 */
  KalmanPredictorConfig config_{};
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_PREDICTOR_H_
