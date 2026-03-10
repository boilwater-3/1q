// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现基于 3D 恒速模型的 Kalman 状态预测器。

#include "airborne_radar/signal/tracking/KalmanPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

KalmanPredictor::KalmanPredictor(KalmanPredictorConfig config)
    : config_(config) {}

/// @brief 执行恒速模型的 Kalman 预测步骤。
/// @param prior 先验高斯状态 [x, vx, y, vy, z, vz]。
/// @param dt 预测时间步长（秒）。
/// @return 预测后的高斯状态。
/// @details 数学公式（参考 Stone Soup KalmanPredictor.predict）：
///          - x̂ = F · x
///          - P̂ = F · P · Fᵀ + Q
GaussianTrackState KalmanPredictor::Predict(const GaussianTrackState &prior,
                                            float dt) const {
  const TransitionMatrix F = BuildTransitionMatrix(dt);
  const ProcessNoiseCovariance Q = BuildProcessNoise(dt, config_.noise_diff_coeff);

  GaussianTrackState predicted;
  predicted.mean = F * prior.mean;
  predicted.covariance = F * prior.covariance * F.transpose() + Q;
  return predicted;
}

void KalmanPredictor::UpdateConfig(KalmanPredictorConfig config) {
  config_ = config;
}

/// @brief 构建 3D 恒速模型的块对角转移矩阵。
/// @param dt 时间步长。
/// @return 6×6 转移矩阵。
/// @details 参考 Stone Soup ConstantVelocity.matrix()：
///          单轴 F_1d = [[1, dt], [0, 1]]
///          组合 F = block_diag(F_1d, F_1d, F_1d)
TransitionMatrix KalmanPredictor::BuildTransitionMatrix(float dt) {
  // 直接构造块对角矩阵，避免运行时 block_diag 调用
  TransitionMatrix F = TransitionMatrix::Identity();

  // X 轴：F(0,1) = dt
  F(0, 1) = dt;
  // Y 轴：F(2,3) = dt
  F(2, 3) = dt;
  // Z 轴：F(4,5) = dt
  F(4, 5) = dt;

  return F;
}

/// @brief 构建 3D 恒速模型的块对角过程噪声矩阵。
/// @param dt 时间步长。
/// @param q 噪声扩散系数。
/// @return 6×6 过程噪声矩阵。
/// @details 参考 Stone Soup ConstantVelocity.covar()：
///          单轴 Q_1d = [[dt³/3, dt²/2], [dt²/2, dt]] × q
///          组合 Q = block_diag(Q_1d, Q_1d, Q_1d)
ProcessNoiseCovariance KalmanPredictor::BuildProcessNoise(float dt, float q) {
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;

  ProcessNoiseCovariance Q = ProcessNoiseCovariance::Zero();

  // 对 X、Y、Z 三个轴分别填充 2×2 子块
  for (int axis = 0; axis < 3; ++axis) {
    const int base = axis * 2;
    Q(base, base) = dt3 / 3.0f;       // 位置-位置
    Q(base, base + 1) = dt2 / 2.0f;   // 位置-速度
    Q(base + 1, base) = dt2 / 2.0f;   // 速度-位置
    Q(base + 1, base + 1) = dt;       // 速度-速度
  }

  Q *= q;
  return Q;
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
