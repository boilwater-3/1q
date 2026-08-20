#include "airborne_radar/signal/association/DistanceMetric.h"

#include <cmath>
#include <limits>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace association {

MahalanobisDistanceMetric::MahalanobisDistanceMetric(float speed_sigma, float rcs_sigma,
                                                     float acceleration_sigma) {
  inverse_variances_ << 1.0f / (speed_sigma * speed_sigma), 1.0f / (rcs_sigma * rcs_sigma),
      1.0f / (acceleration_sigma * acceleration_sigma);
}

float MahalanobisDistanceMetric::Compute(const Eigen::Vector3f& predicted,
                                         const Eigen::Vector3f& measurement) const {
  const Eigen::Vector3f innovation = measurement - predicted;
  return innovation.array().square().matrix().dot(inverse_variances_.matrix());
}

FullMahalanobisDistanceMetric::FullMahalanobisDistanceMetric(
    const Eigen::Matrix3f& innovation_covariance)
    : llt_(innovation_covariance), llt_valid_(llt_.info() == Eigen::Success) {
  if (!llt_valid_) {
    // 中译：协方差矩阵 LLT 分解失败（构造时），距离计算不可用。
    // 标识：数值保护——协方差非正定时马氏距离退化为无穷大，
    //       该度量下的关联不会产生假设；排查协方差输入。
    PROJECT_LOG_ERROR("[FullMahalanobisDistanceMetric] LLT decomposition failed in constructor.");
  }
}

FullMahalanobisDistanceMetric::FullMahalanobisDistanceMetric(float sigma_0, float sigma_1,
                                                             float sigma_2) {
  Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
  S(0, 0) = sigma_0 * sigma_0;
  S(1, 1) = sigma_1 * sigma_1;
  S(2, 2) = sigma_2 * sigma_2;
  llt_.compute(S);
  llt_valid_ = llt_.info() == Eigen::Success;
  if (!llt_valid_) {
    // 中译：协方差矩阵 LLT 分解失败（sigma 构造时），距离计算不可用。
    // 标识：数值保护——协方差非正定时马氏距离退化为无穷大，
    //       该度量下的关联不会产生假设；排查协方差输入。
    PROJECT_LOG_ERROR(
        "[FullMahalanobisDistanceMetric] LLT decomposition failed in sigma constructor.");
  }
}

void FullMahalanobisDistanceMetric::SetInnovationCovariance(const Eigen::Matrix3f& S) {
  llt_.compute(S);
  llt_valid_ = llt_.info() == Eigen::Success;
  if (!llt_valid_) {
    // 中译：协方差矩阵 LLT 分解失败（更新时），距离计算不可用。
    // 标识：数值保护——协方差非正定时马氏距离退化为无穷大，
    //       该度量下的关联不会产生假设；排查协方差输入。
    PROJECT_LOG_ERROR(
        "[FullMahalanobisDistanceMetric] LLT decomposition failed in SetInnovationCovariance.");
  }
}

float FullMahalanobisDistanceMetric::Compute(const Eigen::Vector3f& predicted,
                                             const Eigen::Vector3f& measurement) const {
  if (!llt_valid_) {
    return std::numeric_limits<float>::infinity();
  }

  const Eigen::Vector3f innovation = measurement - predicted;
  const Eigen::Vector3f s_inv_dz = llt_.solve(innovation);
  const float distance = innovation.dot(s_inv_dz);
  return std::isfinite(distance) ? distance : std::numeric_limits<float>::infinity();
}

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar
