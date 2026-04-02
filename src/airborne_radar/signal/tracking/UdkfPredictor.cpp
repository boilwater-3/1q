#include "airborne_radar/signal/tracking/UdkfPredictor.h"

#include <Eigen/Cholesky>

#include <algorithm>

#include "airborne_radar/signal/tracking/KalmanPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {

constexpr float kCovarianceFloor = 1.0e-6f;

}  // namespace

UdkfPredictor::UdkfPredictor(KalmanPredictorConfig config) : config_(config) {}

GaussianTrackState UdkfPredictor::Predict(const GaussianTrackState& prior, float dt) const {
  const TransitionMatrix F = KalmanPredictor::BuildTransitionMatrix(dt);
  const ProcessNoiseCovariance Q = KalmanPredictor::BuildProcessNoise(dt, config_.noise_diff_coeff);

  GaussianTrackState predicted;
  predicted.mean = F * prior.mean;

  const StateCovariance propagated_covariance = F * prior.covariance * F.transpose() + Q;
  predicted.covariance = StabilizeCovariance(propagated_covariance);
  return predicted;
}

void UdkfPredictor::UpdateConfig(KalmanPredictorConfig config) { config_ = config; }

bool UdkfPredictor::Cov2Ud(const StateCovariance& covariance, StateCovariance* upper_u,
                           StateVector* diagonal_d) {
  if (upper_u == nullptr || diagonal_d == nullptr) {
    return false;
  }
  const StateCovariance symmetric_covariance = (covariance + covariance.transpose()) * 0.5f;
  const Eigen::LDLT<StateCovariance> ldlt(symmetric_covariance);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }

  StateCovariance lower_l = StateCovariance::Identity();
  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < row; ++col) {
      lower_l(row, col) = ldlt.matrixL()(row, col);
    }
  }
  *upper_u = lower_l.transpose();
  *diagonal_d = ldlt.vectorD().cwiseMax(kCovarianceFloor);
  return true;
}

StateCovariance UdkfPredictor::Ud2Cov(const StateCovariance& upper_u, const StateVector& diagonal_d) {
  return upper_u.transpose() * diagonal_d.asDiagonal() * upper_u;
}

StateCovariance UdkfPredictor::StabilizeCovariance(const StateCovariance& covariance) {
  StateCovariance upper_u = StateCovariance::Identity();
  StateVector diagonal_d = StateVector::Ones();
  if (!Cov2Ud(covariance, &upper_u, &diagonal_d)) {
    StateCovariance fallback = (covariance + covariance.transpose()) * 0.5f;
    for (int i = 0; i < kStateDim; ++i) {
      fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
    }
    return fallback;
  }
  return Ud2Cov(upper_u, diagonal_d);
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
