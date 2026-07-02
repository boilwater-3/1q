#include "airborne_radar/signal/tracking/UdkfUpdater.h"

#include <Eigen/Cholesky>

#include <algorithm>

#include "common/logging/ProjectLog.h"
#include "common/numerics/NumericGuard.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

using oneq::common::numerics::kCovarianceFloor;

UdkfUpdater::UdkfUpdater(KalmanUpdaterConfig config)
    : config_(config),
      H_(IKalmanUpdater::BuildPositionMeasurementMatrix()),
      R_(IKalmanUpdater::BuildDefaultMeasurementNoise(config.measurement_noise_std)) {}

KalmanUpdateResult UdkfUpdater::Update(const GaussianTrackState& predicted,
                                       const MeasurementVector& measurement) const {
  return Update(predicted, measurement, R_);
}

KalmanUpdateResult UdkfUpdater::Update(const GaussianTrackState& predicted,
                                       const MeasurementVector& measurement,
                                       const MeasurementCovariance& dynamic_R) const {
  KalmanUpdateResult result;
  result.innovation = measurement - H_ * predicted.mean;
  result.innovation_covariance = H_ * predicted.covariance * H_.transpose() + dynamic_R;

  const Eigen::LLT<MeasurementCovariance> llt(result.innovation_covariance);
  if (llt.info() != Eigen::Success) {
    PROJECT_LOG_ERROR("[UdkfUpdater] Innovation covariance LLT decomposition failed.");
    result.posterior = predicted;
    return result;
  }

  const KalmanGainMatrix K = llt.solve(H_ * predicted.covariance).transpose();
  result.posterior.mean = predicted.mean + K * result.innovation;

  const StateCovariance I_KH = StateCovariance::Identity() - K * H_;
  const StateCovariance joseph_covariance =
      I_KH * predicted.covariance * I_KH.transpose() + K * dynamic_R * K.transpose();
  result.posterior.covariance = StabilizeCovarianceWithUd(joseph_covariance);
  return result;
}

void UdkfUpdater::UpdateConfig(KalmanUpdaterConfig config) {
  config_ = config;
  R_ = IKalmanUpdater::BuildDefaultMeasurementNoise(config.measurement_noise_std);
}

StateCovariance UdkfUpdater::StabilizeCovarianceWithUd(const StateCovariance& covariance) {
  const StateCovariance symmetric_covariance = (covariance + covariance.transpose()) * 0.5f;
  Eigen::LDLT<StateCovariance> ldlt(symmetric_covariance);
  if (ldlt.info() != Eigen::Success) {
    StateCovariance fallback = symmetric_covariance;
    for (int i = 0; i < fallback.rows(); ++i) {
      fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
    }
    return fallback;
  }

  const Eigen::Matrix<float, kStateDim, 1> diagonal = ldlt.vectorD().cwiseMax(kCovarianceFloor);
  StateCovariance lower = StateCovariance::Identity();
  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < row; ++col) {
      lower(row, col) = ldlt.matrixL()(row, col);
    }
  }
  return lower * diagonal.asDiagonal() * lower.transpose();
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
