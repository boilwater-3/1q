#include "airborne_radar/signal/tracking/SrifPredictor.h"

#include <Eigen/Cholesky>

#include <algorithm>

#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "common/numerics/NumericGuard.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

using oneq::internal::numerics::kCovarianceFloor;

SrifPredictor::SrifPredictor(KalmanPredictorConfig config) : config_(config) {}

GaussianTrackState SrifPredictor::Predict(const GaussianTrackState& prior, float dt) const {
  const TransitionMatrix F = KalmanPredictor::BuildTransitionMatrix(dt);
  const ProcessNoiseCovariance Q = KalmanPredictor::BuildProcessNoise(dt, config_.noise_diff_coeff);

  GaussianTrackState predicted;
  predicted.mean = F * prior.mean;
  const StateCovariance propagated_covariance = F * prior.covariance * F.transpose() + Q;
  predicted.covariance = StabilizeWithInformationForm(propagated_covariance);
  return predicted;
}

void SrifPredictor::UpdateConfig(KalmanPredictorConfig config) { config_ = config; }

StateCovariance SrifPredictor::StabilizeWithInformationForm(const StateCovariance& covariance) {
  const StateCovariance symmetric_covariance = (covariance + covariance.transpose()) * 0.5f;
  const Eigen::LLT<StateCovariance> llt(symmetric_covariance);
  if (llt.info() != Eigen::Success) {
    StateCovariance fallback = symmetric_covariance;
    for (int i = 0; i < kStateDim; ++i) {
      fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
    }
    return fallback;
  }

  const StateCovariance information = llt.solve(StateCovariance::Identity());
  const Eigen::LDLT<StateCovariance> information_ldlt(information);
  if (information_ldlt.info() != Eigen::Success) {
    StateCovariance fallback = symmetric_covariance;
    for (int i = 0; i < kStateDim; ++i) {
      fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
    }
    return fallback;
  }

  StateCovariance stabilized = information_ldlt.solve(StateCovariance::Identity());
  stabilized = (stabilized + stabilized.transpose()) * 0.5f;
  for (int i = 0; i < kStateDim; ++i) {
    stabilized(i, i) = std::max(stabilized(i, i), kCovarianceFloor);
  }
  return stabilized;
}

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar
