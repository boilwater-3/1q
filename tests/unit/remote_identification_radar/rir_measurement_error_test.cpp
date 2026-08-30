// Copyright 2026. All Rights Reserved.
//
// @file rir_measurement_error_test.cpp
// @brief 验证 RIR 测量误差模型（副本改写自 ar_signal_detection_test.cpp 的
//        MeasurementErrorModel 段；阶段 2-M M6）。

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>

#include "remote_identification_radar/dwell/RirMeasurementErrorModel.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using dwell::RirEffectiveBeamwidthDeg;
using dwell::RirMeasurementErrorModel;

/// @brief 高 SNR 下距离误差接近偏置项、角度误差小。
TEST(RirMeasurementErrorModelTest, HighSnrSmallErrors) {
  const auto state = RirMeasurementErrorModel::Compute(20.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                       1.0e6f);
  EXPECT_NEAR(state.range_error_std_m, 27.5f, 0.5f);  // 0.5·150/10 + 20
  EXPECT_LT(state.angle_error_std_rad, 0.01f);
}

/// @brief 俯仰波束宽度加宽 → 等效角度标准差增大。
TEST(RirMeasurementErrorModelTest, ElevationBeamwidthAffectsEquivalentAngleStdDev) {
  const auto narrow =
      RirMeasurementErrorModel::Compute(13.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f}, 1.0e6f);
  const auto wide =
      RirMeasurementErrorModel::Compute(13.0f, RirEffectiveBeamwidthDeg{3.0f, 6.0f}, 1.0e6f);
  EXPECT_GT(wide.angle_error_std_rad, narrow.angle_error_std_rad);
  // 波束内距离误差不受角度维影响。
  EXPECT_FLOAT_EQ(wide.range_error_std_m, narrow.range_error_std_m);
}

/// @brief SNR 降低 → 距离与角度误差均增大。
TEST(RirMeasurementErrorModelTest, LowerSnrInflatesErrors) {
  const auto strong = RirMeasurementErrorModel::Compute(20.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                        1.0e6f);
  const auto weak = RirMeasurementErrorModel::Compute(0.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                     1.0e6f);
  EXPECT_GT(weak.range_error_std_m, strong.range_error_std_m);
  EXPECT_GT(weak.angle_error_std_rad, strong.angle_error_std_rad);
}

/// @brief 等效角度标准差 = az/el 两轴的 RMS 合成（各向同性时等于单轴）。
TEST(RirMeasurementErrorModelTest, EquivalentAngleIsRmsOfAxes) {
  const auto iso =
      RirMeasurementErrorModel::Compute(10.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f}, 1.0e6f);
  const auto az_only =
      RirMeasurementErrorModel::Compute(10.0f, RirEffectiveBeamwidthDeg{3.0f, 0.001f}, 1.0e6f);
  // el 波束极窄 → el 误差近似仅偏置项，等效值低于各向同性。
  EXPECT_LT(az_only.angle_error_std_rad, iso.angle_error_std_rad);
}

/// @brief 笛卡尔量测协瓦差雅可比仰角列 = ∂ENU/∂el = (−xz/rh, −yz/rh, rh)（模长=斜距），
///        与测试内独立构造的 J·diag(σr²,σθ²,σθ²)·Jᵀ 全矩阵一致（审计 A4：旧实现
///        仰角列整列多除 r²，高度轴贡献按 1/r⁴ 坍缩成"完美测量"）。
TEST(RirCartesianCovarianceTest, ElevationColumnMatchesInverseMapJacobian) {
  session::RirSceneTarget target;
  target.position_x = 3.0e6f;
  target.position_y = 4.0e6f;
  target.position_z = 1.0e5f;
  const float range_std_m = 50.0f;
  const float angle_std_rad = 0.002f;

  const tracking::RirMeasurementCovariance covariance =
      runtime::RirController::MakeCartesianMeasurementCovariance(target, range_std_m,
                                                                 angle_std_rad);

  // 独立构造期望值（double 复算，忽略函数内叠加的数值地板）。
  const double x = static_cast<double>(target.position_x);
  const double y = static_cast<double>(target.position_y);
  const double z = static_cast<double>(target.position_z);
  const double r = std::sqrt(x * x + y * y + z * z);
  const double rh = std::sqrt(x * x + y * y);
  const double sr = static_cast<double>(range_std_m);
  const double stheta = static_cast<double>(angle_std_rad);
  Eigen::Matrix3d jacobian;
  jacobian << x / r, -y, -x * z / rh,
              y / r,  x, -y * z / rh,
              z / r, 0.0, rh;
  const Eigen::Matrix3d expected =
      jacobian * Eigen::Vector3d(sr * sr, stheta * stheta, stheta * stheta).asDiagonal() *
      jacobian.transpose();

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(covariance(row, col), expected(row, col),
                  1.0e-3 * std::abs(expected(row, col)) + 1.0e-2)
          << "(" << row << "," << col << ")";
    }
  }
  // 回归判别：远距离高度轴方差由仰角项主导（(σθ·rh)² ≈ 1e8 m² 量级），
  // 旧 1/r⁴ 坍缩口径下该元素坍缩到数值地板（~0.34 m²）。
  EXPECT_GT(covariance(2, 2), 1.0e6);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
