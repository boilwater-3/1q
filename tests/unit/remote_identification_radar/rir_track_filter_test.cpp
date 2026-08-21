// Copyright 2026. All Rights Reserved.
//
// @file rir_track_filter_test.cpp
// @brief 验证 RIR 轻量跟踪子集单目标 KF（阶段 2-T T1：预测/更新/初始化布局）。

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "remote_identification_radar/tracking/RirTrackFilter.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using tracking::RirGaussianState;
using tracking::RirMeasurementCovariance;
using tracking::RirStateCovariance;
using tracking::RirStateVector;
using tracking::RirTrackFilter;
using tracking::RirTrackFilterConfig;
using tracking::RirTrackMeasurement;
using tracking::RirTrackState;

RirTrackMeasurement MakeMeasurement(float px, float py, float pz, float vx, float vy, float vz) {
  RirTrackMeasurement measurement;
  measurement.position = Eigen::Vector3f(px, py, pz);
  measurement.velocity = Eigen::Vector3f(vx, vy, vz);
  measurement.measurement_covariance = RirMeasurementCovariance::Identity();
  return measurement;
}

/// @brief 初始状态布局 [x, vx, y, vy, z, vz]，协方差对角为配置初值。
TEST(RirTrackFilterTest, InitializeUsesCvLayoutAndConfiguredVariance) {
  RirTrackFilterConfig config;
  config.initial_state_variance = 25.0f;
  const RirTrackFilter filter(config);

  const RirGaussianState state =
      filter.Initialize(MakeMeasurement(10.0f, 20.0f, 30.0f, 1.0f, 2.0f, 3.0f));
  EXPECT_FLOAT_EQ(state.mean(0), 10.0f);
  EXPECT_FLOAT_EQ(state.mean(1), 1.0f);
  EXPECT_FLOAT_EQ(state.mean(2), 20.0f);
  EXPECT_FLOAT_EQ(state.mean(3), 2.0f);
  EXPECT_FLOAT_EQ(state.mean(4), 30.0f);
  EXPECT_FLOAT_EQ(state.mean(5), 3.0f);
  EXPECT_FLOAT_EQ(state.covariance(0, 0), 25.0f);
  EXPECT_FLOAT_EQ(state.covariance(3, 3), 25.0f);
  EXPECT_FLOAT_EQ(state.covariance(0, 1), 0.0f);
}

/// @brief CV 预测：位置按 dt 外推，过程噪声按连续白噪声离散化。
TEST(RirTrackFilterTest, PredictAppliesConstantVelocityAndProcessNoise) {
  RirTrackFilterConfig config;
  config.process_noise_diff_coeff = 1.0f;
  const RirTrackFilter filter(config);

  RirStateVector mean = RirStateVector::Zero();
  mean(0) = 0.0f;
  mean(1) = 10.0f;
  RirGaussianState prior(mean, RirStateCovariance::Zero());

  const RirGaussianState predicted = filter.Predict(prior, 2.0f);
  EXPECT_FLOAT_EQ(predicted.mean(0), 20.0f);
  EXPECT_FLOAT_EQ(predicted.mean(1), 10.0f);
  // Q_xx = dt³/3 = 8/3；Q_xvx = dt²/2 = 2；Q_vxvx = dt = 2。
  EXPECT_NEAR(predicted.covariance(0, 0), 8.0f / 3.0f, 1.0e-5f);
  EXPECT_NEAR(predicted.covariance(0, 1), 2.0f, 1.0e-5f);
  EXPECT_NEAR(predicted.covariance(1, 1), 2.0f, 1.0e-5f);
}

/// @brief 标准位置更新：增益与 Joseph 协方差按 1D 闭式解可验算。
TEST(RirTrackFilterTest, UpdateProducesExpectedPosteriorForDiagonalCase) {
  const RirTrackFilter filter;
  RirGaussianState predicted;
  predicted.covariance = RirStateCovariance::Identity() * 100.0f;

  RirTrackMeasurement measurement = MakeMeasurement(5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  const auto result = filter.Update(predicted, measurement);

  EXPECT_NEAR(result.innovation(0), 5.0f, 1.0e-5f);
  EXPECT_NEAR(result.innovation_covariance(0, 0), 101.0f, 1.0e-5f);
  EXPECT_NEAR(result.posterior.mean(0), 500.0f / 101.0f, 1.0e-5f);
  EXPECT_NEAR(result.posterior.covariance(0, 0), 100.0f / 101.0f, 1.0e-5f);
}

/// @brief 量测噪声 R 越小，后验越靠近量测（动态误差参与更新）。
TEST(RirTrackFilterTest, SmallerMeasurementNoisePullsPosteriorTowardMeasurement) {
  const RirTrackFilter filter;
  RirGaussianState predicted;
  predicted.covariance = RirStateCovariance::Identity() * 100.0f;

  RirTrackMeasurement precise = MakeMeasurement(5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  precise.measurement_covariance = RirMeasurementCovariance::Identity() * 0.25f;
  const auto precise_result = filter.Update(predicted, precise);

  RirTrackMeasurement coarse = precise;
  coarse.measurement_covariance = RirMeasurementCovariance::Identity() * 400.0f;
  const auto coarse_result = filter.Update(predicted, coarse);

  EXPECT_NEAR(precise_result.posterior.mean(0), 5.0f, 0.05f);
  EXPECT_NEAR(coarse_result.posterior.mean(0), 1.0f, 0.1f);
  EXPECT_LT(precise_result.posterior.covariance(0, 0), coarse_result.posterior.covariance(0, 0));
}

/// @brief 越界 q/std 下限钳制（与 AR SignalComponentFactory 工厂口径一致）：
///        负值 q 不再产生负定过程噪声，预测协方差保持 PSD（对角非负）。
TEST(RirTrackFilterTest, ClampsOutOfRangeNoiseConfigToLowerBound) {
  RirTrackFilterConfig config;
  config.process_noise_diff_coeff = -1.0f;  // 未钳制时 Q 负定 → 协方差对角变负
  config.default_measurement_noise_std = -2.0f;
  const RirTrackFilter filter(config);

  RirStateVector mean = RirStateVector::Zero();
  mean(1) = 10.0f;
  const RirGaussianState predicted = filter.Predict(RirGaussianState(mean, RirStateCovariance::Zero()), 1.0f);
  EXPECT_TRUE(predicted.covariance.allFinite());
  EXPECT_GE(predicted.covariance(0, 0), 0.0f);
  EXPECT_GE(predicted.covariance(1, 1), 0.0f);

  const auto result =
      filter.Update(predicted, MakeMeasurement(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f));
  EXPECT_TRUE(result.posterior.covariance.allFinite());
}

/// @brief 不确定度迹 = 协方差 position 分块迹（识别运动质量本源信号）。
TEST(RirTrackStateTest, EstimationUncertaintyTraceUsesPositionBlock) {
  RirTrackState state;
  state.gaussian_state.covariance = RirStateCovariance::Zero();
  state.gaussian_state.covariance(0, 0) = 1.0f;
  state.gaussian_state.covariance(2, 2) = 2.0f;
  state.gaussian_state.covariance(4, 4) = 3.0f;
  state.gaussian_state.covariance(1, 1) = 99.0f;

  EXPECT_FLOAT_EQ(state.EstimationUncertaintyTrace(), 6.0f);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
