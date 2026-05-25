#include "airborne_radar/signal/tracking/KalmanUpdater.h"

#include <Eigen/Cholesky>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

KalmanUpdater::KalmanUpdater(KalmanUpdaterConfig config)
    : config_(config),
      H_(IKalmanUpdater::BuildPositionMeasurementMatrix()),
      R_(IKalmanUpdater::BuildDefaultMeasurementNoise(config.measurement_noise_std)) {}

KalmanUpdateResult KalmanUpdater::Update(const GaussianTrackState& predicted,
                                         const MeasurementVector& measurement) const {
  return Update(predicted, measurement, R_);
}

KalmanUpdateResult KalmanUpdater::Update(const GaussianTrackState& predicted,
                                         const MeasurementVector& measurement,
                                         const MeasurementCovariance& dynamic_R) const {
  KalmanUpdateResult result;

  /* Innovation (新息) */
  result.innovation = measurement - H_ * predicted.mean;

  /**
   * Innovation covariance (新息协方差)
   * S = H · P̂ · Hᵀ + R
   */
  result.innovation_covariance = H_ * predicted.covariance * H_.transpose() + dynamic_R;

  /**
   *  Kalman gain (Kalman 增益)
   *  K = P̂ · Hᵀ · S⁻¹  等价于  K = (S⁻¹ · H · P̂)ᵀ
   *  后者避免计算显式逆矩阵，数值更稳定
   */
  const Eigen::LLT<MeasurementCovariance> llt(result.innovation_covariance);
  if (llt.info() != Eigen::Success) {
    PROJECT_LOG_ERROR(
        "[KalmanUpdater] Innovation covariance LLT decomposition failed; update is skipped.");
    result.posterior = predicted;
    return result;
  }

  const KalmanGainMatrix K = llt.solve(H_ * predicted.covariance).transpose();

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
  const StateCovariance I_KH = StateCovariance::Identity() - K * H_;
  result.posterior.covariance =
      I_KH * predicted.covariance * I_KH.transpose() + K * dynamic_R * K.transpose();

  return result;
}

void KalmanUpdater::UpdateConfig(KalmanUpdaterConfig config) {
  config_ = config;
  R_ = IKalmanUpdater::BuildDefaultMeasurementNoise(config.measurement_noise_std);
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
