// 验证 algorithms.md 误差模型条目：sbirs_error_model_test
// 覆盖 5 类误差（轨道/姿态/视场/折射/滞后）与确定性随机源可复现性。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace {

// 统一问题列表模型（规则 14）：检查校验问题列表中是否包含指定 code（机器消费字段）。
bool ContainsCode(const sbirs_sensor::session::SbirsIssueList& issues, const std::string& code) {
  for (const sbirs_sensor::session::SbirsIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

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

// --- 高刷新率多帧融合（2026-08-31 窄场高精度数据输出建模） ---

TEST(SbirsErrorModelTest, FusedSingleFrameMatchesSingleDrawPath) {
  // frame_count<=1 融合路径与既有单帧路径逐位一致（历史行为不回归）。
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.fov_sigma_deg = 0.05f;
  sbirs_sensor::foundation::SbirsRandomSource a(7U);
  sbirs_sensor::foundation::SbirsRandomSource b(7U);
  const sbirs_sensor::foundation::SbirsFusedBearingResult fused =
      sbirs_sensor::foundation::ApplyAngularErrorModelFused(model, &a, 10.0f, 5.0f, 1.0e6, 0.0f, 1);
  const sbirs_sensor::foundation::SbirsErrorBearing single =
      sbirs_sensor::foundation::ApplyAngularErrorModel(model, &b, 10.0f, 5.0f, 1.0e6, 0.0f);
  EXPECT_FLOAT_EQ(fused.bearing.azimuth_deg, single.azimuth_deg);
  EXPECT_FLOAT_EQ(fused.bearing.elevation_deg, single.elevation_deg);
  EXPECT_NEAR(fused.frame_sigma_deg, 0.05, 1.0e-6);
  EXPECT_NEAR(fused.fused_sigma_deg, 0.05, 1.0e-6);
}

TEST(SbirsErrorModelTest, FusedSigmaScalesWithFrameCount) {
  // N 帧融合：σ_fused = σ_frame/√N；无随机分量时确定性偏差照常、σ 恒 0。
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.fov_sigma_deg = 0.1f;
  sbirs_sensor::foundation::SbirsRandomSource random(3U);
  const sbirs_sensor::foundation::SbirsFusedBearingResult fused =
      sbirs_sensor::foundation::ApplyAngularErrorModelFused(model, &random, 359.9995f, 5.0f,
                                                            1.0e6, 0.0f, 16);
  EXPECT_NEAR(fused.frame_sigma_deg, 0.1, 1.0e-6);
  EXPECT_NEAR(fused.fused_sigma_deg, 0.1 / 4.0, 1.0e-7);
  // 方位均值走最短角差：真值贴 0°/360° 环绕点，融合值不得被折叠拉偏 > 单帧 σ。
  const double az_wrapped =
      fused.bearing.azimuth_deg < 180.0 ? fused.bearing.azimuth_deg + 360.0
                                        : fused.bearing.azimuth_deg;
  EXPECT_NEAR(az_wrapped, 359.9995, 0.1);

  sbirs_sensor::config::SbirsErrorModelConfig zero_model;
  zero_model.orbit_sigma_deg = 0.0f;
  zero_model.attitude_sigma_deg = 0.0f;
  zero_model.fov_sigma_deg = 0.0f;
  sbirs_sensor::foundation::SbirsRandomSource zero_random(3U);
  const sbirs_sensor::foundation::SbirsFusedBearingResult zero_fused =
      sbirs_sensor::foundation::ApplyAngularErrorModelFused(zero_model, &zero_random, 10.0f, 5.0f,
                                                            1.0e6, 0.0f, 16);
  EXPECT_DOUBLE_EQ(zero_fused.frame_sigma_deg, 0.0);
  EXPECT_DOUBLE_EQ(zero_fused.fused_sigma_deg, 0.0);
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

TEST(SbirsErrorModelTest, ZeroPhysicalSigmasProduceZeroAngularVariance) {
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.orbit_sigma_deg = 0.0f;
  model.attitude_sigma_deg = 0.0f;
  model.fov_sigma_deg = 0.0f;
  EXPECT_DOUBLE_EQ(sbirs_sensor::foundation::ResolveEffectiveAngularSigmaDeg(model), 0.0);
}

TEST(SbirsErrorModelTest, PhysicalSigmasUseRss) {
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.orbit_sigma_deg = 0.03f;
  model.attitude_sigma_deg = 0.04f;
  model.fov_sigma_deg = 0.0f;

  EXPECT_NEAR(sbirs_sensor::foundation::ResolveEffectiveAngularSigmaDeg(model), 0.05, 1.0e-7);
  const auto covariance =
      sbirs_sensor::tracking::BuildMeasurementCovariance(model, 0.0, 0.0f, 0.0f);
  const double expected_variance_rad2 = std::pow(0.05 * 3.14159265358979323846 / 180.0, 2.0);
  EXPECT_NEAR(covariance(0, 0), expected_variance_rad2, 1.0e-10);
  EXPECT_NEAR(covariance(1, 1), expected_variance_rad2, 1.0e-10);
}

TEST(SbirsErrorModelTest, PhysicalSigmaIsZeroMeanAndAzElSamplesAreIndependent) {
  sbirs_sensor::config::SbirsErrorModelConfig model;
  model.orbit_sigma_deg = 0.0f;
  model.attitude_sigma_deg = 0.2f;
  model.fov_sigma_deg = 0.0f;
  model.range_fraction_sigma = 0.0f;
  sbirs_sensor::foundation::SbirsRandomSource source(17U);

  const int sample_count = 4096;
  double azimuth_sum = 0.0;
  double azimuth_square_sum = 0.0;
  bool observed_distinct_axes = false;
  for (int i = 0; i < sample_count; ++i) {
    const auto bearing = sbirs_sensor::foundation::ApplyAngularErrorModel(
        model, &source, 0.0f, 0.0f, 0.0, 0.0f);
    azimuth_sum += bearing.azimuth_deg;
    azimuth_square_sum += static_cast<double>(bearing.azimuth_deg) * bearing.azimuth_deg;
    observed_distinct_axes = observed_distinct_axes || bearing.azimuth_deg != bearing.elevation_deg;
  }
  const double mean = azimuth_sum / sample_count;
  const double variance = azimuth_square_sum / sample_count - mean * mean;
  EXPECT_NEAR(mean, 0.0, 0.01);
  EXPECT_NEAR(std::sqrt(variance), 0.2, 0.01);
  EXPECT_TRUE(observed_distinct_axes);
}

TEST(SbirsErrorModelTest, ValidationRejectsInvalidErrorModelParameters) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.error_model.attitude_sigma_deg = -0.01f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_error_model_sigmas"));

  config.policy.error_model.attitude_sigma_deg = 0.01f;
  config.policy.error_model.orbit_sigma_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_error_model_sigmas"));

  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = std::numeric_limits<float>::infinity();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_error_model_sigmas"));

  config.policy.error_model.range_fraction_sigma = 0.001f;
  config.policy.error_model.detector_bandwidth_hz = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_detector_bandwidth"));
}

}  // namespace
