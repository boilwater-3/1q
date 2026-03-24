#include "airborne_radar/signal/tracking/KalmanUpdater.h"

#include <Eigen/Cholesky>

namespace airborne_radar {
namespace signal {
namespace tracking {

KalmanUpdater::KalmanUpdater(KalmanUpdaterConfig config)
    : config_(config),
      H_(BuildMeasurementMatrix()),
      R_(BuildMeasurementNoise(config.measurement_noise_std)) {}

KalmanUpdateResult KalmanUpdater::Update(
    const GaussianTrackState &predicted,
    const MeasurementVector &measurement) const {
  return Update(predicted, measurement, R_);
}

KalmanUpdateResult KalmanUpdater::Update(
    const GaussianTrackState &predicted,
    const MeasurementVector &measurement,
    const MeasurementCovariance &dynamic_R) const {
  KalmanUpdateResult result;

  /* Innovation (新息) */
  result.innovation = measurement - H_ * predicted.mean;

  /** 
   * Innovation covariance (新息协方差)
   * S = H · P̂ · Hᵀ + R
   */ 
  result.innovation_covariance =
      H_ * predicted.covariance * H_.transpose() + dynamic_R;

  /** 
   *  Kalman gain (Kalman 增益)
   *  K = P̂ · Hᵀ · S⁻¹
   *  使用 LLT 分解求解 S⁻¹，比直接 .inverse() 更稳定
   */ 
  const KalmanGainMatrix K =
      predicted.covariance * H_.transpose() *
      result.innovation_covariance.llt().solve(
          MeasurementCovariance::Identity());

  /** 
   * Posterior mean (后验均值)
   * x = x̂ + K · y
   */ 
  result.posterior.mean = predicted.mean + K * result.innovation;

  /** 
   * Posterior covariance (Joseph 形式)
   * P = (I - K·H) · P̂ · (I - K·H)ᵀ + K · R · Kᵀ
   * 相比简化公式 P = (I-KH)·P̂，Joseph 形式在数值上更稳定，
   * 能保证协方差矩阵的对称正定性。
   * 参考 Stone Soup KalmanUpdater._posterior_covariance 中的实现。
   */ 
  const StateCovariance I_KH =
      StateCovariance::Identity() - K * H_;
  result.posterior.covariance =
      I_KH * predicted.covariance * I_KH.transpose() +
      K * dynamic_R * K.transpose();

  return result;
}

void KalmanUpdater::UpdateConfig(KalmanUpdaterConfig config) {
  config_ = config;
  R_ = BuildMeasurementNoise(config.measurement_noise_std);
}

MeasurementMatrix KalmanUpdater::BuildMeasurementMatrix() {
  MeasurementMatrix H = MeasurementMatrix::Zero();
  H(0, 0) = 1.0f;  // x
  H(1, 2) = 1.0f;  // y
  H(2, 4) = 1.0f;  // z
  return H;
}

MeasurementCovariance KalmanUpdater::BuildMeasurementNoise(float std_dev) {
  const float variance = std_dev * std_dev;
  MeasurementCovariance R = MeasurementCovariance::Zero();
  R(0, 0) = variance;
  R(1, 1) = variance;
  R(2, 2) = variance;
  return R;
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
