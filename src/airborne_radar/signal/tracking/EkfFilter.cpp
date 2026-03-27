#include "airborne_radar/signal/tracking/EkfFilter.h"

#include <Eigen/Cholesky>

namespace airborne_radar {
namespace signal {
namespace tracking {

EkfPredictor::EkfPredictor(const ITransitionModel* model, EkfPredictorConfig config)
    : model_(model), config_(config) {}

GaussianTrackState EkfPredictor::Predict(const GaussianTrackState& prior, float dt) const {
  /* 非线性均值预测 */
  const StateVector x_pred = model_->Function(prior.mean, dt);

  /* Jacobian 线性化 */
  const TransitionMatrix F = model_->Jacobian(prior.mean, dt);

  /* 过程噪声（复用 KalmanPredictor 的 CV 模型噪声结构） */
  const ProcessNoiseCovariance Q = KalmanPredictor::BuildProcessNoise(dt, config_.noise_diff_coeff);

  GaussianTrackState predicted;
  predicted.mean = x_pred;
  predicted.covariance = F * prior.covariance * F.transpose() + Q;
  return predicted;
}

EkfUpdater::EkfUpdater(const IMeasurementModel* model, EkfUpdaterConfig config)
    : model_(model), config_(config), R_(BuildMeasurementNoise(config.measurement_noise_std)) {}

KalmanUpdateResult EkfUpdater::Update(const GaussianTrackState& predicted,
                                      const MeasurementVector& measurement) const {
  return Update(predicted, measurement, R_);
}

KalmanUpdateResult EkfUpdater::Update(const GaussianTrackState& predicted,
                                      const MeasurementVector& measurement,
                                      const MeasurementCovariance& dynamic_R) const {
  KalmanUpdateResult result;

  /* 非线性量测预测 */
  const MeasurementVector z_pred = model_->Function(predicted.mean);

  /* Jacobian */
  const MeasurementMatrix H = model_->Jacobian(predicted.mean);

  /* Innovation */
  result.innovation = measurement - z_pred;

  /* Innovation covariance */
  result.innovation_covariance = H * predicted.covariance * H.transpose() + dynamic_R;

  /* Kalman gain：K = P̂·Hᵀ·S⁻¹ 等价于 K = (S⁻¹·H·P̂)ᵀ，避免计算显式逆 */
  const KalmanGainMatrix K =
      result.innovation_covariance.llt().solve(H * predicted.covariance).transpose();

  /* Posterior mean */
  result.posterior.mean = predicted.mean + K * result.innovation;

  /* Posterior covariance (Joseph 形式) */
  const StateCovariance I_KH = StateCovariance::Identity() - K * H;
  result.posterior.covariance =
      I_KH * predicted.covariance * I_KH.transpose() + K * dynamic_R * K.transpose();

  return result;
}

MeasurementCovariance EkfUpdater::BuildMeasurementNoise(float std_dev) {
  const float variance = std_dev * std_dev;
  return MeasurementCovariance::Identity() * variance;
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
