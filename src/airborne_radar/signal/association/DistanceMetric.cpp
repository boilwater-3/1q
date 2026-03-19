#include "airborne_radar/signal/association/DistanceMetric.h"

namespace airborne_radar {
namespace signal {
namespace association {

MahalanobisDistanceMetric::MahalanobisDistanceMetric(
    float speed_sigma, float rcs_sigma, float acceleration_sigma) {
  inverse_variances_ << 1.0f / (speed_sigma * speed_sigma),
      1.0f / (rcs_sigma * rcs_sigma),
      1.0f / (acceleration_sigma * acceleration_sigma);
}

float MahalanobisDistanceMetric::Compute(
    const Eigen::Vector3f &predicted,
    const Eigen::Vector3f &measurement) const {
  const Eigen::Vector3f innovation = measurement - predicted;
  return innovation.array().square().matrix().dot(inverse_variances_.matrix());
}

FullMahalanobisDistanceMetric::FullMahalanobisDistanceMetric(
    const Eigen::Matrix3f &innovation_covariance)
    : llt_(innovation_covariance) {}

FullMahalanobisDistanceMetric::FullMahalanobisDistanceMetric(
    float sigma_0, float sigma_1, float sigma_2) {
  Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
  S(0, 0) = sigma_0 * sigma_0;
  S(1, 1) = sigma_1 * sigma_1;
  S(2, 2) = sigma_2 * sigma_2;
  llt_.compute(S);
}

void FullMahalanobisDistanceMetric::SetInnovationCovariance(
    const Eigen::Matrix3f &S) {
  llt_.compute(S);
}

float FullMahalanobisDistanceMetric::Compute(
    const Eigen::Vector3f &predicted,
    const Eigen::Vector3f &measurement) const {
  const Eigen::Vector3f innovation = measurement - predicted;
  const Eigen::Vector3f s_inv_dz = llt_.solve(innovation);
  return innovation.dot(s_inv_dz);
}

} // namespace association
} // namespace signal
} // namespace airborne_radar
