// design.md 2.10 验证入口：sbirs_error_model_test
// 覆盖 5 类误差（轨道/姿态/视场/折射/滞后）与确定性随机源可复现性。
#include <gtest/gtest.h>

#include "sbirs_sensor/foundation/SbirsErrorModel.h"

namespace {

TEST(SbirsErrorModelTest, SameSeedProducesReproducibleBearing) {
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.attitude_sigma_deg = 0.05f;
  model.range_fraction_sigma = 0.01f;
  sbirs_sensor::foundation::SbirsRandomSource a(42U);
  sbirs_sensor::foundation::SbirsRandomSource b(42U);
  const sbirs_sensor::foundation::SbirsErrorBearing first =
      sbirs_sensor::foundation::ApplyAngularErrorModel(model, &a, 10.0f, 5.0f, 1.0e6, 0.0f);
  const sbirs_sensor::foundation::SbirsErrorBearing second =
      sbirs_sensor::foundation::ApplyAngularErrorModel(model, &b, 10.0f, 5.0f, 1.0e6, 0.0f);
  EXPECT_FLOAT_EQ(first.azimuth_deg, second.azimuth_deg);
  EXPECT_FLOAT_EQ(first.elevation_deg, second.elevation_deg);
  EXPECT_DOUBLE_EQ(first.range_m, second.range_m);
}

TEST(SbirsErrorModelTest, CaptureRestoreResumesRandomSequence) {
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.attitude_sigma_deg = 0.05f;
  sbirs_sensor::foundation::SbirsRandomSource src(7U);
  const unsigned int snapshot = src.Capture();
  const sbirs_sensor::foundation::SbirsErrorBearing first =
      sbirs_sensor::foundation::ApplyAngularErrorModel(model, &src, 0.0f, 0.0f, 1.0e6, 0.0f);
  // 推进若干次后恢复快照，序列应与首次一致。
  for (int i = 0; i < 3; ++i) {
    sbirs_sensor::foundation::ApplyAngularErrorModel(model, &src, 0.0f, 0.0f, 1.0e6, 0.0f);
  }
  src.Restore(snapshot);
  const sbirs_sensor::foundation::SbirsErrorBearing replayed =
      sbirs_sensor::foundation::ApplyAngularErrorModel(model, &src, 0.0f, 0.0f, 1.0e6, 0.0f);
  EXPECT_FLOAT_EQ(first.azimuth_deg, replayed.azimuth_deg);
}

TEST(SbirsErrorModelTest, RefractionDecreasesWithRange) {
  // 折射角随距离增大而减小（design 2.10 Δθ_refr = 1.5e-6 / (d·cosβ)）。
  const double near_ref = sbirs_sensor::foundation::RefractionErrorDeg(1.0e5, 30.0f);
  const double far_ref = sbirs_sensor::foundation::RefractionErrorDeg(1.0e7, 30.0f);
  EXPECT_GT(near_ref, far_ref);
  EXPECT_GT(near_ref, 0.0);
}

TEST(SbirsErrorModelTest, DynamicLagScalesWithAngularRate) {
  // 滞后误差与目标角速度成正比（design 2.10 Δθ_lag = ω / (2π·f_det)）。
  const double slow = sbirs_sensor::foundation::DynamicLagErrorDeg(0.1f, 100.0f);
  const double fast = sbirs_sensor::foundation::DynamicLagErrorDeg(1.0f, 100.0f);
  EXPECT_GT(fast, slow);
  // 零带宽保护：返回 0，不发散。
  EXPECT_DOUBLE_EQ(sbirs_sensor::foundation::DynamicLagErrorDeg(1.0f, 0.0f), 0.0);
}

TEST(SbirsErrorModelTest, LegacyAngularSigmaAppliedWhenPhysicalSigmasZero) {
  // 当 orbit/fov sigma 为 0 时回退到合并 angular_sigma_deg（向后兼容）。
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.angular_sigma_deg = 0.1f;
  model.orbit_sigma_deg = 0.0f;
  model.attitude_sigma_deg = 0.0f;
  model.fov_sigma_deg = 0.0f;
  sbirs_sensor::foundation::SbirsRandomSource src(1U);
  const sbirs_sensor::foundation::SbirsErrorBearing bearing =
      sbirs_sensor::foundation::ApplyAngularErrorModel(model, &src, 0.0f, 0.0f, 1.0e6, 0.0f);
  // 合成误差叠加到真值 0 上，应非零。
  EXPECT_NE(bearing.azimuth_deg, 0.0f);
}

}  // namespace
