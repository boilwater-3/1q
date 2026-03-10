// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现标准线性 Kalman 量测更新器。

#include "airborne_radar/signal/tracking/KalmanUpdater.h"

#include <Eigen/Cholesky>

namespace airborne_radar {
namespace signal {
namespace tracking {

KalmanUpdater::KalmanUpdater(KalmanUpdaterConfig config)
    : config_(config),
      H_(BuildMeasurementMatrix()),
      R_(BuildMeasurementNoise(config.measurement_noise_std)) {}

/// @brief 执行标准 Kalman 量测更新步骤。
/// @param predicted 预测后的高斯状态。
/// @param measurement 量测向量 [x, y, z]。
/// @return 包含后验状态、新息和新息协方差的更新结果。
/// @details 数学公式（参考 Stone Soup KalmanUpdater.update）：
///          1. y = z - H·x̂                     （新息 / Innovation）
///          2. S = H·P̂·Hᵀ + R                  （新息协方差）
///          3. K = P̂·Hᵀ·S⁻¹                    （Kalman 增益）
///          4. x = x̂ + K·y                      （后验均值）
///          5. P = (I-KH)·P̂·(I-KH)ᵀ + K·R·Kᵀ  （Joseph 形式后验协方差）
KalmanUpdateResult KalmanUpdater::Update(
    const GaussianTrackState &predicted,
    const MeasurementVector &measurement) const {
  KalmanUpdateResult result;

  // 1. Innovation (新息)
  result.innovation = measurement - H_ * predicted.mean;

  // 2. Innovation covariance (新息协方差)
  //    S = H · P̂ · Hᵀ + R
  result.innovation_covariance =
      H_ * predicted.covariance * H_.transpose() + R_;

  // 3. Kalman gain (Kalman 增益)
  //    K = P̂ · Hᵀ · S⁻¹
  //    使用 LLT 分解求解 S⁻¹，比直接 .inverse() 更稳定
  const KalmanGainMatrix K =
      predicted.covariance * H_.transpose() *
      result.innovation_covariance.llt().solve(
          MeasurementCovariance::Identity());

  // 4. Posterior mean (后验均值)
  //    x = x̂ + K · y
  result.posterior.mean = predicted.mean + K * result.innovation;

  // 5. Posterior covariance (Joseph 形式)
  //    P = (I - K·H) · P̂ · (I - K·H)ᵀ + K · R · Kᵀ
  //    相比简化公式 P = (I-KH)·P̂，Joseph 形式在数值上更稳定，
  //    能保证协方差矩阵的对称正定性。
  //    参考 Stone Soup KalmanUpdater._posterior_covariance 中的实现。
  const StateCovariance I_KH =
      StateCovariance::Identity() - K * H_;
  result.posterior.covariance =
      I_KH * predicted.covariance * I_KH.transpose() +
      K * R_ * K.transpose();

  return result;
}

void KalmanUpdater::UpdateConfig(KalmanUpdaterConfig config) {
  config_ = config;
  R_ = BuildMeasurementNoise(config.measurement_noise_std);
}

/// @brief 构建 3×6 线性位置提取量测矩阵。
/// @return H 矩阵。
/// @details 状态向量 [x, vx, y, vy, z, vz] 中提取位置分量 [x, y, z]：
///          | 1 0 0 0 0 0 |
///          | 0 0 1 0 0 0 |
///          | 0 0 0 0 1 0 |
MeasurementMatrix KalmanUpdater::BuildMeasurementMatrix() {
  MeasurementMatrix H = MeasurementMatrix::Zero();
  H(0, 0) = 1.0f;  // x
  H(1, 2) = 1.0f;  // y
  H(2, 4) = 1.0f;  // z
  return H;
}

/// @brief 构建 3×3 量测噪声协方差矩阵。
/// @param std_dev 噪声标准差。
/// @return 对角量测噪声矩阵 R = diag(σ², σ², σ²)。
MeasurementCovariance KalmanUpdater::BuildMeasurementNoise(float std_dev) {
  const float variance = std_dev * std_dev;
  MeasurementCovariance R = MeasurementCovariance::Zero();
  R(0, 0) = variance;
  R(1, 1) = variance;
  R(2, 2) = variance;
  return R;
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
