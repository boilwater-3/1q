#include "airborne_radar/signal/tracking/SrifUpdater.h"

#include <Eigen/Cholesky>
#include <Eigen/QR>

#include <algorithm>

#include "common/logging/ProjectLog.h"
#include "common/numerics/NumericGuard.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

using oneq::common::numerics::kCovarianceFloor;

SrifUpdater::SrifUpdater(KalmanUpdaterConfig config)
    : config_(config),
      H_(IKalmanUpdater::BuildPositionMeasurementMatrix()),
      R_(IKalmanUpdater::BuildDefaultMeasurementNoise(config.measurement_noise_std)) {}

KalmanUpdateResult SrifUpdater::Update(const GaussianTrackState& predicted,
                                       const MeasurementVector& measurement) const {
  return Update(predicted, measurement, R_);
}

KalmanUpdateResult SrifUpdater::Update(const GaussianTrackState& predicted,
                                       const MeasurementVector& measurement,
                                       const MeasurementCovariance& dynamic_R) const {
  KalmanUpdateResult result;
  result.innovation = measurement - H_ * predicted.mean;
  result.innovation_covariance = H_ * predicted.covariance * H_.transpose() + dynamic_R;

  const Eigen::LLT<StateCovariance> pred_llt(predicted.covariance);
  const Eigen::LLT<MeasurementCovariance> meas_llt(dynamic_R);
  if (pred_llt.info() != Eigen::Success || meas_llt.info() != Eigen::Success) {
    PROJECT_LOG_ERROR("[SrifUpdater] LLT decomposition failed for predicted covariance or R.");
    result.posterior = predicted;
    return result;
  }

  const StateCovariance predicted_information = pred_llt.solve(StateCovariance::Identity());
  const MeasurementCovariance measurement_information =
      meas_llt.solve(MeasurementCovariance::Identity());

  const StateCovariance information =
      predicted_information + H_.transpose() * measurement_information * H_;
  const Eigen::LDLT<StateCovariance> information_ldlt(information);
  if (information_ldlt.info() != Eigen::Success) {
    PROJECT_LOG_ERROR("[SrifUpdater] Information matrix LDLT decomposition failed.");
    result.posterior = predicted;
    return result;
  }

  const StateVector information_vector =
      predicted_information * predicted.mean + H_.transpose() * measurement_information * measurement;
  result.posterior.mean = information_ldlt.solve(information_vector);
  StateCovariance posterior_covariance =
      information_ldlt.solve(StateCovariance::Identity());
  posterior_covariance = (posterior_covariance + posterior_covariance.transpose()) * 0.5f;
  for (int i = 0; i < posterior_covariance.rows(); ++i) {
    posterior_covariance(i, i) = std::max(posterior_covariance(i, i), kCovarianceFloor);
  }
  result.posterior.covariance = posterior_covariance;
  return result;
}

void SrifUpdater::UpdateConfig(KalmanUpdaterConfig config) {
  config_ = config;
  R_ = IKalmanUpdater::BuildDefaultMeasurementNoise(config.measurement_noise_std);
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
