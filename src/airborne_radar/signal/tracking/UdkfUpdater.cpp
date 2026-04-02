#include "airborne_radar/signal/tracking/UdkfUpdater.h"

#include <Eigen/Cholesky>

#include <algorithm>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {

constexpr float kCovarianceFloor = 1.0e-6f;

}  // namespace

UdkfUpdater::UdkfUpdater(KalmanUpdaterConfig config)
    : config_(config),
      H_(BuildMeasurementMatrix()),
      R_(BuildMeasurementNoise(config.measurement_noise_std)) {}

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

MeasurementMatrix UdkfUpdater::BuildMeasurementMatrix() {
  MeasurementMatrix H = MeasurementMatrix::Zero();
  H(0, 0) = 1.0f;
  H(1, 2) = 1.0f;
  H(2, 4) = 1.0f;
  return H;
}

MeasurementCovariance UdkfUpdater::BuildMeasurementNoise(float std_dev) {
  const float variance = std_dev * std_dev;
  MeasurementCovariance R = MeasurementCovariance::Zero();
  R(0, 0) = variance;
  R(1, 1) = variance;
  R(2, 2) = variance;
  return R;
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
