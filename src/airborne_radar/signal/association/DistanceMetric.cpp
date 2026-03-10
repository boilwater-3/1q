// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现 Signal 层数据关联使用的距离度量算法。

#include "airborne_radar/signal/association/DistanceMetric.h"

namespace airborne_radar {
namespace signal {
namespace association {

/// @brief 构造马氏距离度量器。
/// @param speed_sigma 速度维度标准差。
/// @param rcs_sigma RCS 维度标准差。
/// @param acceleration_sigma 加速度维度标准差。
MahalanobisDistanceMetric::MahalanobisDistanceMetric(
    float speed_sigma, float rcs_sigma, float acceleration_sigma) {
  inverse_variances_ << 1.0f / (speed_sigma * speed_sigma),
      1.0f / (rcs_sigma * rcs_sigma),
      1.0f / (acceleration_sigma * acceleration_sigma);
}

/// @brief 计算预测特征与量测特征之间的归一化平方距离。
/// @param predicted 预测特征。
/// @param measurement 量测特征。
/// @return 归一化平方距离。
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

/// @brief 计算完整马氏距离。
/// @details d² = Δzᵀ S⁻¹ Δz = Δzᵀ · solve(S, Δz)
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