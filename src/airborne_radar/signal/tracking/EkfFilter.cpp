// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现扩展 Kalman 滤波器的预测器和更新器。

#include "airborne_radar/signal/tracking/EkfFilter.h"

#include <Eigen/Cholesky>

namespace airborne_radar {
namespace signal {
namespace tracking {

EkfPredictor::EkfPredictor(const ITransitionModel *model,
                           EkfPredictorConfig config)
    : model_(model), config_(config) {}

/// @brief EKF 预测步骤。
/// @details 参考 Stone Soup ExtendedKalmanPredictor：
///          1. x̂ = f(x, dt)             ← 非线性转移
///          2. F = ∂f/∂x |_{x,dt}       ← Jacobian 线性化
///          3. Q = BuildProcessNoise(dt) ← 复用 CV 模型噪声
///          4. P̂ = F·P·Fᵀ + Q           ← 协方差传播
GaussianTrackState EkfPredictor::Predict(const GaussianTrackState &prior,
                                         float dt) const {
  // 非线性均值预测
  const StateVector x_pred = model_->Function(prior.mean, dt);

  // Jacobian 线性化
  const TransitionMatrix F = model_->Jacobian(prior.mean, dt);

  // 过程噪声（复用 KalmanPredictor 的 CV 模型噪声结构）
  const ProcessNoiseCovariance Q =
      KalmanPredictor::BuildProcessNoise(dt, config_.noise_diff_coeff);

  GaussianTrackState predicted;
  predicted.mean = x_pred;
  predicted.covariance = F * prior.covariance * F.transpose() + Q;
  return predicted;
}

EkfUpdater::EkfUpdater(const IMeasurementModel *model,
                       EkfUpdaterConfig config)
    : model_(model), config_(config),
      R_(BuildMeasurementNoise(config.measurement_noise_std)) {}

/// @brief EKF 更新步骤。
/// @details 参考 Stone Soup ExtendedKalmanUpdater：
///          1. ẑ = h(x̂)                 ← 非线性量测预测
///          2. H = ∂h/∂x |_{x̂}          ← Jacobian 线性化
///          3. y = z - ẑ                ← 新息
///          4. S = H·P̂·Hᵀ + R           ← 新息协方差
///          5. K = P̂·Hᵀ·S⁻¹             ← Kalman 增益
///          6. x = x̂ + K·y              ← 后验均值
///          7. P = (I-KH)P̂(I-KH)ᵀ+KRKᵀ ← Joseph 形式
KalmanUpdateResult EkfUpdater::Update(
    const GaussianTrackState &predicted,
    const MeasurementVector &measurement) const {
  KalmanUpdateResult result;

  // 非线性量测预测
  const MeasurementVector z_pred = model_->Function(predicted.mean);

  // Jacobian
  const MeasurementMatrix H = model_->Jacobian(predicted.mean);

  // 1. Innovation
  result.innovation = measurement - z_pred;

  // 2. Innovation covariance
  result.innovation_covariance =
      H * predicted.covariance * H.transpose() + R_;

  // 3. Kalman gain (LLT 分解)
  const KalmanGainMatrix K =
      predicted.covariance * H.transpose() *
      result.innovation_covariance.llt().solve(
          MeasurementCovariance::Identity());

  // 4. Posterior mean
  result.posterior.mean = predicted.mean + K * result.innovation;

  // 5. Posterior covariance (Joseph 形式)
  const StateCovariance I_KH =
      StateCovariance::Identity() - K * H;
  result.posterior.covariance =
      I_KH * predicted.covariance * I_KH.transpose() +
      K * R_ * K.transpose();

  return result;
}

MeasurementCovariance EkfUpdater::BuildMeasurementNoise(float std_dev) {
  const float variance = std_dev * std_dev;
  return MeasurementCovariance::Identity() * variance;
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
