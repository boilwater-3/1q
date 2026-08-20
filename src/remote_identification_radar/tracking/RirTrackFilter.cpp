/**
 * @file RirTrackFilter.cpp
 * @brief RIR 轻量跟踪子集的单目标 Kalman 滤波器实现（阶段 2-T T1）。
 */

#include "remote_identification_radar/tracking/RirTrackFilter.h"

namespace remote_identification_radar {
namespace tracking {

namespace {

::oneq::common::estimation::KalmanPredictorConfig MakePredictorConfig(
    const RirTrackFilterConfig& config) {
  ::oneq::common::estimation::KalmanPredictorConfig common_config;
  common_config.noise_diff_coeff = config.process_noise_diff_coeff;
  return common_config;
}

::oneq::common::estimation::KalmanUpdaterConfig MakeUpdaterConfig(
    const RirTrackFilterConfig& config) {
  ::oneq::common::estimation::KalmanUpdaterConfig common_config;
  common_config.measurement_noise_std = config.default_measurement_noise_std;
  return common_config;
}

}  // namespace

RirTrackFilter::RirTrackFilter(RirTrackFilterConfig config)
    : config_(config),
      predictor_(MakePredictorConfig(config)),
      updater_(MakeUpdaterConfig(config)) {}

RirGaussianState RirTrackFilter::Initialize(const RirTrackMeasurement& measurement) const {
  return Initialize(measurement.position, measurement.velocity);
}

RirGaussianState RirTrackFilter::Initialize(const Eigen::Vector3f& position,
                                            const Eigen::Vector3f& velocity) const {
  RirStateVector mean = RirStateVector::Zero();
  mean(0) = position.x();
  mean(1) = velocity.x();
  mean(2) = position.y();
  mean(3) = velocity.y();
  mean(4) = position.z();
  mean(5) = velocity.z();

  RirStateCovariance covariance = RirStateCovariance::Identity() * config_.initial_state_variance;
  return RirGaussianState(mean, covariance);
}

RirGaussianState RirTrackFilter::Predict(const RirGaussianState& prior, float dt_sec) const {
  return predictor_.Predict(prior, dt_sec);
}

RirKalmanUpdateResult RirTrackFilter::Update(const RirGaussianState& predicted,
                                             const RirTrackMeasurement& measurement) const {
  const RirMeasurementVector position = measurement.position;
  return updater_.Update(predicted, position, measurement.measurement_covariance);
}

RirKalmanUpdateResult RirTrackFilter::Update(
    const RirGaussianState& predicted, const Eigen::Vector3f& position,
    const RirMeasurementCovariance& measurement_covariance) const {
  return updater_.Update(predicted, position, measurement_covariance);
}

void RirTrackFilter::UpdateConfig(RirTrackFilterConfig config) {
  config_ = config;
  predictor_.UpdateConfig(MakePredictorConfig(config));
  updater_.UpdateConfig(MakeUpdaterConfig(config));
}

}  // namespace tracking
}  // namespace remote_identification_radar
