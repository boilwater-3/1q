// Copyright 2026. All Rights Reserved.
//
// @file kalman_filter_test.cpp
// @brief 验证 KalmanPredictor 和 KalmanUpdater 的核心算法正确性。

#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cmath>

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/SrifPredictor.h"
#include "airborne_radar/signal/tracking/SrifUpdater.h"
#include "airborne_radar/signal/tracking/UdkfPredictor.h"
#include "airborne_radar/signal/tracking/UdkfUpdater.h"

namespace airborne_radar {
namespace tests {

namespace {

using signal::tracking::GaussianTrackState;
using signal::tracking::KalmanPredictor;
using signal::tracking::KalmanPredictorConfig;
using signal::tracking::KalmanUpdater;
using signal::tracking::KalmanUpdaterConfig;
using signal::tracking::KalmanUpdateResult;
using signal::tracking::kMeasurementDim;
using signal::tracking::kStateDim;
using signal::tracking::MeasurementVector;
using signal::tracking::SrifPredictor;
using signal::tracking::SrifUpdater;
using signal::tracking::StateCovariance;
using signal::tracking::StateVector;
using signal::tracking::UdkfPredictor;
using signal::tracking::UdkfUpdater;

/// @brief 构造一个简单的先验状态用于测试。
/// @param x 初始 X 位置。
/// @param vx 初始 X 速度。
/// @param initial_pos_var 初始位置方差。
/// @param initial_vel_var 初始速度方差。
GaussianTrackState MakePrior(float x, float vx, float initial_pos_var = 100.0f,
                             float initial_vel_var = 25.0f) {
  StateVector mean = StateVector::Zero();
  mean(0) = x;   // x
  mean(1) = vx;  // vx

  StateCovariance P = StateCovariance::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Index axis_index = static_cast<Eigen::Index>(axis);
    const Eigen::Index position_index = axis_index * 2;
    const Eigen::Index velocity_index = position_index + 1;
    P(position_index, position_index) = initial_pos_var;
    P(velocity_index, velocity_index) = initial_vel_var;
  }

  return GaussianTrackState(mean, P);
}

/// @brief 浮点近似比较的容差。
constexpr float kTolerance = 1e-4f;

}  // namespace

// ============================================================================
// KalmanPredictor 测试
// ============================================================================

TEST(KalmanPredictorTest, PredictWithZeroDtReturnsIdentity) {
  KalmanPredictorConfig config;
  config.noise_diff_coeff = 1.0f;
  KalmanPredictor predictor(config);

  const GaussianTrackState prior = MakePrior(100.0f, 10.0f);
  const GaussianTrackState predicted = predictor.Predict(prior, 0.0f);

  // dt=0 时 F=I，Q=0 → 均值和协方差不变
  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_NEAR(predicted.mean(i), prior.mean(i), kTolerance) << "mean mismatch at index " << i;
  }
  for (int r = 0; r < kStateDim; ++r) {
    for (int c = 0; c < kStateDim; ++c) {
      EXPECT_NEAR(predicted.covariance(r, c), prior.covariance(r, c), kTolerance)
          << "covariance mismatch at (" << r << "," << c << ")";
    }
  }
}

TEST(KalmanPredictorTest, PredictPropagatesPositionByVelocity) {
  KalmanPredictor predictor;

  // 状态: x=100, vx=10, 其余为零
  const GaussianTrackState prior = MakePrior(100.0f, 10.0f);
  const float dt = 1.0f;
  const GaussianTrackState predicted = predictor.Predict(prior, dt);

  // x̂ = x + vx*dt = 100 + 10*1 = 110
  EXPECT_NEAR(predicted.mean(0), 110.0f, kTolerance);
  // vx̂ = vx = 10（恒速模型速度不变）
  EXPECT_NEAR(predicted.mean(1), 10.0f, kTolerance);
  // Y、Z 轴位置和速度保持为零
  EXPECT_NEAR(predicted.mean(2), 0.0f, kTolerance);
  EXPECT_NEAR(predicted.mean(3), 0.0f, kTolerance);
  EXPECT_NEAR(predicted.mean(4), 0.0f, kTolerance);
  EXPECT_NEAR(predicted.mean(5), 0.0f, kTolerance);
}

TEST(KalmanPredictorTest, PredictPropagatesAllAxes) {
  KalmanPredictor predictor;

  StateVector mean = StateVector::Zero();
  mean(0) = 10.0f;  // x
  mean(1) = 1.0f;   // vx
  mean(2) = 20.0f;  // y
  mean(3) = -2.0f;  // vy
  mean(4) = 30.0f;  // z
  mean(5) = 3.0f;   // vz

  GaussianTrackState prior(mean, StateCovariance::Identity());
  const float dt = 2.0f;
  const GaussianTrackState predicted = predictor.Predict(prior, dt);

  // 各轴: position = pos + vel * dt
  EXPECT_NEAR(predicted.mean(0), 10.0f + 1.0f * 2.0f, kTolerance);     // x=12
  EXPECT_NEAR(predicted.mean(1), 1.0f, kTolerance);                    // vx=1
  EXPECT_NEAR(predicted.mean(2), 20.0f + (-2.0f) * 2.0f, kTolerance);  // y=16
  EXPECT_NEAR(predicted.mean(3), -2.0f, kTolerance);                   // vy=-2
  EXPECT_NEAR(predicted.mean(4), 30.0f + 3.0f * 2.0f, kTolerance);     // z=36
  EXPECT_NEAR(predicted.mean(5), 3.0f, kTolerance);                    // vz=3
}

TEST(KalmanPredictorTest, CovarianceGrowsWithTime) {
  KalmanPredictorConfig config;
  config.noise_diff_coeff = 1.0f;
  KalmanPredictor predictor(config);

  StateCovariance P0 = StateCovariance::Identity();
  GaussianTrackState prior(StateVector::Zero(), P0);

  const GaussianTrackState p1 = predictor.Predict(prior, 1.0f);
  const GaussianTrackState p2 = predictor.Predict(prior, 2.0f);

  // dt 越长，过程噪声 Q 越大，协方差对角线应单调增长
  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_GT(p1.covariance(i, i), P0(i, i)) << "P(1) diagonal should grow at index " << i;
    EXPECT_GT(p2.covariance(i, i), p1.covariance(i, i))
        << "P(2) diagonal should be larger than P(1) at index " << i;
  }
}

TEST(KalmanPredictorTest, CovarianceRemainsSymmetric) {
  KalmanPredictorConfig config;
  config.noise_diff_coeff = 5.0f;
  KalmanPredictor predictor(config);

  // 使用非对角先验协方差
  StateCovariance P0 = StateCovariance::Identity() * 50.0f;
  P0(0, 1) = 5.0f;
  P0(1, 0) = 5.0f;
  GaussianTrackState prior(StateVector::Zero(), P0);

  const GaussianTrackState predicted = predictor.Predict(prior, 0.5f);

  for (int r = 0; r < kStateDim; ++r) {
    for (int c = r + 1; c < kStateDim; ++c) {
      EXPECT_NEAR(predicted.covariance(r, c), predicted.covariance(c, r), kTolerance)
          << "covariance not symmetric at (" << r << "," << c << ")";
    }
  }
}

TEST(KalmanPredictorTest, ProcessNoiseScalesWithDiffCoeff) {
  KalmanPredictorConfig config_small;
  config_small.noise_diff_coeff = 0.5f;
  KalmanPredictor predictor_small(config_small);

  KalmanPredictorConfig config_large;
  config_large.noise_diff_coeff = 5.0f;
  KalmanPredictor predictor_large(config_large);

  GaussianTrackState prior(StateVector::Zero(), StateCovariance::Identity());
  const float dt = 1.0f;

  const GaussianTrackState p_small = predictor_small.Predict(prior, dt);
  const GaussianTrackState p_large = predictor_large.Predict(prior, dt);

  // 噪声扩散系数大 → 协方差增长更快
  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_GT(p_large.covariance(i, i), p_small.covariance(i, i))
        << "larger q should produce larger covariance at index " << i;
  }
}

TEST(KalmanPredictorTest, ProcessNoiseMatchesStoneSoupCV) {
  // 验证 Q 矩阵公式与 Stone Soup ConstantVelocity.covar() 精确一致
  // 参考: Q_1d = [[dt³/3, dt²/2], [dt²/2, dt]] × q
  KalmanPredictorConfig config;
  config.noise_diff_coeff = 2.0f;
  KalmanPredictor predictor(config);

  GaussianTrackState prior(StateVector::Zero(), StateCovariance::Zero());
  const float dt = 0.5f;
  const float q = 2.0f;
  const GaussianTrackState predicted = predictor.Predict(prior, dt);

  // P̂ = F·0·Fᵀ + Q = Q，因此 predicted.covariance 就是纯 Q
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;

  // X 轴 2×2 块
  EXPECT_NEAR(predicted.covariance(0, 0), q * dt3 / 3.0f, kTolerance);
  EXPECT_NEAR(predicted.covariance(0, 1), q * dt2 / 2.0f, kTolerance);
  EXPECT_NEAR(predicted.covariance(1, 0), q * dt2 / 2.0f, kTolerance);
  EXPECT_NEAR(predicted.covariance(1, 1), q * dt, kTolerance);

  // Y 轴块
  EXPECT_NEAR(predicted.covariance(2, 2), q * dt3 / 3.0f, kTolerance);
  EXPECT_NEAR(predicted.covariance(2, 3), q * dt2 / 2.0f, kTolerance);

  // Z 轴块
  EXPECT_NEAR(predicted.covariance(4, 4), q * dt3 / 3.0f, kTolerance);
  EXPECT_NEAR(predicted.covariance(4, 5), q * dt2 / 2.0f, kTolerance);

  // 跨轴为零
  EXPECT_NEAR(predicted.covariance(0, 2), 0.0f, kTolerance);
  EXPECT_NEAR(predicted.covariance(0, 4), 0.0f, kTolerance);
  EXPECT_NEAR(predicted.covariance(2, 4), 0.0f, kTolerance);
}

TEST(KalmanPredictorBackendTest, ThreeBackendsPredictFiniteSymmetricCovariance) {
  const GaussianTrackState prior = MakePrior(100.0f, 12.0f, 300.0f, 50.0f);
  const float dt = 0.75f;

  KalmanPredictorConfig config;
  config.noise_diff_coeff = 2.0f;
  KalmanPredictor standard_predictor(config);
  UdkfPredictor udkf_predictor(config);
  SrifPredictor srif_predictor(config);

  const GaussianTrackState standard_predicted = standard_predictor.Predict(prior, dt);
  const GaussianTrackState udkf_predicted = udkf_predictor.Predict(prior, dt);
  const GaussianTrackState srif_predicted = srif_predictor.Predict(prior, dt);

  const GaussianTrackState* predicted_states[] = {&standard_predicted, &udkf_predicted,
                                                  &srif_predicted};
  for (std::size_t i = 0; i < sizeof(predicted_states) / sizeof(predicted_states[0]); ++i) {
    EXPECT_TRUE(predicted_states[i]->mean.allFinite());
    EXPECT_TRUE(predicted_states[i]->covariance.allFinite());
    for (int r = 0; r < kStateDim; ++r) {
      EXPECT_GT(predicted_states[i]->covariance(r, r), 0.0f);
      for (int c = r + 1; c < kStateDim; ++c) {
        EXPECT_NEAR(predicted_states[i]->covariance(r, c), predicted_states[i]->covariance(c, r),
                    1.0e-4f);
      }
    }
  }
}

TEST(KalmanPredictorBackendTest, UdkfPreservesPermutationSensitiveCovarianceAtZeroDt) {
  KalmanPredictorConfig config;
  config.noise_diff_coeff = 0.0f;
  UdkfPredictor udkf_predictor(config);

  StateCovariance covariance = StateCovariance::Zero();
  covariance(0, 0) = 1.0e-4f;
  covariance(1, 1) = 50.0f;
  covariance(0, 1) = 1.0e-2f;
  covariance(1, 0) = covariance(0, 1);
  covariance(2, 2) = 2.0e-4f;
  covariance(3, 3) = 60.0f;
  covariance(2, 3) = 2.0e-2f;
  covariance(3, 2) = covariance(2, 3);
  covariance(4, 4) = 3.0e-4f;
  covariance(5, 5) = 70.0f;
  covariance(4, 5) = 3.0e-2f;
  covariance(5, 4) = covariance(4, 5);
  covariance = (covariance + covariance.transpose()) * 0.5f;

  const Eigen::LDLT<StateCovariance> ldlt(covariance);
  ASSERT_EQ(ldlt.info(), Eigen::Success);
  const Eigen::Transpositions<kStateDim, kStateDim>& transpositions = ldlt.transpositionsP();
  bool has_non_identity_permutation = false;
  for (int i = 0; i < kStateDim; ++i) {
    if (transpositions.indices()(i) != i) {
      has_non_identity_permutation = true;
      break;
    }
  }
  ASSERT_TRUE(has_non_identity_permutation);

  GaussianTrackState prior(StateVector::Zero(), covariance);
  const GaussianTrackState predicted = udkf_predictor.Predict(prior, 0.0f);

  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < kStateDim; ++col) {
      EXPECT_NEAR(predicted.covariance(row, col), covariance(row, col), 1.0e-3f);
    }
  }
}

// ============================================================================
// KalmanUpdater 测试
// ============================================================================

TEST(KalmanUpdaterTest, UpdateReducesCovariance) {
  KalmanUpdaterConfig config;
  config.measurement_noise_std = 10.0f;
  KalmanUpdater updater(config);

  const GaussianTrackState predicted = MakePrior(100.0f, 10.0f, 200.0f, 50.0f);
  const MeasurementVector measurement(105.0f, 0.0f, 0.0f);

  const KalmanUpdateResult result = updater.Update(predicted, measurement);

  // 量测更新后，位置协方差应减小（信息增加）
  EXPECT_LT(result.posterior.covariance(0, 0), predicted.covariance(0, 0));
  EXPECT_LT(result.posterior.covariance(2, 2), predicted.covariance(2, 2));
  EXPECT_LT(result.posterior.covariance(4, 4), predicted.covariance(4, 4));
}

TEST(KalmanUpdaterTest, PosteriorMeanShiftsTowardMeasurement) {
  KalmanUpdaterConfig config;
  config.measurement_noise_std = 10.0f;
  KalmanUpdater updater(config);

  const GaussianTrackState predicted = MakePrior(100.0f, 10.0f, 200.0f, 50.0f);
  const MeasurementVector measurement(120.0f, 0.0f, 0.0f);

  const KalmanUpdateResult result = updater.Update(predicted, measurement);

  // 后验均值应在先验均值和量测之间
  EXPECT_GT(result.posterior.mean(0), 100.0f);
  EXPECT_LT(result.posterior.mean(0), 120.0f);
}

TEST(KalmanUpdaterTest, InnovationIsCorrect) {
  KalmanUpdater updater;

  StateVector mean = StateVector::Zero();
  mean(0) = 50.0f;  // x
  mean(2) = 30.0f;  // y
  mean(4) = 10.0f;  // z
  GaussianTrackState predicted(mean, StateCovariance::Identity() * 100.0f);

  const MeasurementVector measurement(55.0f, 32.0f, 8.0f);
  const KalmanUpdateResult result = updater.Update(predicted, measurement);

  // y = z - H·x̂ = [55-50, 32-30, 8-10] = [5, 2, -2]
  EXPECT_NEAR(result.innovation(0), 5.0f, kTolerance);
  EXPECT_NEAR(result.innovation(1), 2.0f, kTolerance);
  EXPECT_NEAR(result.innovation(2), -2.0f, kTolerance);
}

TEST(KalmanUpdaterTest, PosteriorCovarianceRemainsSymmetric) {
  KalmanUpdaterConfig config;
  config.measurement_noise_std = 5.0f;
  KalmanUpdater updater(config);

  // 非对角先验
  StateCovariance P = StateCovariance::Identity() * 50.0f;
  P(0, 1) = 10.0f;
  P(1, 0) = 10.0f;
  P(2, 3) = -5.0f;
  P(3, 2) = -5.0f;
  GaussianTrackState predicted(StateVector::Zero(), P);

  const MeasurementVector measurement(10.0f, 20.0f, 30.0f);
  const KalmanUpdateResult result = updater.Update(predicted, measurement);

  for (int r = 0; r < kStateDim; ++r) {
    for (int c = r + 1; c < kStateDim; ++c) {
      EXPECT_NEAR(result.posterior.covariance(r, c), result.posterior.covariance(c, r), kTolerance)
          << "posterior covariance not symmetric at (" << r << "," << c << ")";
    }
  }
}

TEST(KalmanUpdaterTest, PosteriorCovarianceRemainsPositiveDefinite) {
  KalmanUpdaterConfig config;
  config.measurement_noise_std = 5.0f;
  KalmanUpdater updater(config);

  GaussianTrackState predicted = MakePrior(100.0f, 10.0f, 500.0f, 100.0f);
  const MeasurementVector measurement(110.0f, 5.0f, -3.0f);

  const KalmanUpdateResult result = updater.Update(predicted, measurement);

  // 所有对角线元素应为正（正定的必要条件）
  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_GT(result.posterior.covariance(i, i), 0.0f)
        << "posterior covariance diagonal negative at index " << i;
  }

  // 用 LLT 分解验证正定性
  Eigen::LLT<StateCovariance> llt(result.posterior.covariance);
  EXPECT_EQ(llt.info(), Eigen::Success) << "posterior covariance is not positive definite";
}

TEST(KalmanUpdaterTest, PerfectMeasurementCollapsesCovariance) {
  // 量测噪声极小时，后验应几乎等于量测值
  KalmanUpdaterConfig config;
  config.measurement_noise_std = 0.001f;
  KalmanUpdater updater(config);

  GaussianTrackState predicted = MakePrior(100.0f, 10.0f, 1000.0f, 100.0f);
  const MeasurementVector measurement(200.0f, 50.0f, -30.0f);

  const KalmanUpdateResult result = updater.Update(predicted, measurement);

  // 位置应几乎等于量测
  EXPECT_NEAR(result.posterior.mean(0), 200.0f, 0.1f);  // x
  EXPECT_NEAR(result.posterior.mean(2), 50.0f, 0.1f);   // y
  EXPECT_NEAR(result.posterior.mean(4), -30.0f, 0.1f);  // z
}

TEST(KalmanUpdaterTest, NoisyMeasurementHasLessInfluence) {
  // 量测噪声很大时，后验应更接近先验
  KalmanUpdaterConfig config;
  config.measurement_noise_std = 1000.0f;
  KalmanUpdater updater(config);

  GaussianTrackState predicted = MakePrior(100.0f, 10.0f, 1.0f, 1.0f);
  const MeasurementVector measurement(200.0f, 50.0f, -30.0f);

  const KalmanUpdateResult result = updater.Update(predicted, measurement);

  // 后验应非常接近先验
  EXPECT_NEAR(result.posterior.mean(0), 100.0f, 1.0f);  // x ≈ 100
  EXPECT_NEAR(result.posterior.mean(2), 0.0f, 1.0f);    // y ≈ 0
  EXPECT_NEAR(result.posterior.mean(4), 0.0f, 1.0f);    // z ≈ 0
}

TEST(KalmanUpdaterTest, DynamicCovarianceAltersUpdateWeight) {
  KalmanUpdater updater;

  // 预测状态均值为 100
  GaussianTrackState predicted = MakePrior(100.0f, 10.0f, 100.0f, 25.0f);
  const MeasurementVector measurement(150.0f, 0.0f, 0.0f);

  // 1. 使用极小的动态 R (极其信任量测)
  signal::tracking::MeasurementCovariance small_R =
      signal::tracking::MeasurementCovariance::Identity() * 1.0f;
  const KalmanUpdateResult result_small_R = updater.Update(predicted, measurement, small_R);

  // 2. 使用极大的动态 R (极度不信任量测，信任先验)
  signal::tracking::MeasurementCovariance large_R =
      signal::tracking::MeasurementCovariance::Identity() * 10000.0f;
  const KalmanUpdateResult result_large_R = updater.Update(predicted, measurement, large_R);

  // 结果对比：小 R 下均值应被拉向 150，大 R 下均值应保持在 100 附近
  EXPECT_GT(result_small_R.posterior.mean(0), result_large_R.posterior.mean(0));
  EXPECT_NEAR(result_small_R.posterior.mean(0), 150.0f, 5.0f);
  EXPECT_NEAR(result_large_R.posterior.mean(0), 100.0f, 5.0f);
}

TEST(KalmanUpdaterBackendTest, ThreeBackendsProduceFiniteStateAndCovariance) {
  const GaussianTrackState predicted = MakePrior(100.0f, 5.0f, 250.0f, 100.0f);
  const MeasurementVector measurement(101.5f, 4.0f, -2.5f);

  KalmanUpdaterConfig config;
  config.measurement_noise_std = 4.0f;
  KalmanUpdater standard_updater(config);
  UdkfUpdater udkf_updater(config);
  SrifUpdater srif_updater(config);

  const KalmanUpdateResult standard_result = standard_updater.Update(predicted, measurement);
  const KalmanUpdateResult udkf_result = udkf_updater.Update(predicted, measurement);
  const KalmanUpdateResult srif_result = srif_updater.Update(predicted, measurement);

  const KalmanUpdateResult* results[] = {&standard_result, &udkf_result, &srif_result};
  for (std::size_t i = 0; i < sizeof(results) / sizeof(results[0]); ++i) {
    EXPECT_TRUE(results[i]->posterior.mean.allFinite());
    EXPECT_TRUE(results[i]->posterior.covariance.allFinite());
    for (int diag = 0; diag < kStateDim; ++diag) {
      EXPECT_GT(results[i]->posterior.covariance(diag, diag), 0.0f);
    }
  }
}

TEST(KalmanPredictUpdateBackendTest, ThreeBackendPairsRemainFiniteAcrossSequence) {
  KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = 1.5f;
  KalmanUpdaterConfig updater_config;
  updater_config.measurement_noise_std = 3.0f;

  KalmanPredictor standard_predictor(predictor_config);
  KalmanUpdater standard_updater(updater_config);
  UdkfPredictor udkf_predictor(predictor_config);
  UdkfUpdater udkf_updater(updater_config);
  SrifPredictor srif_predictor(predictor_config);
  SrifUpdater srif_updater(updater_config);

  GaussianTrackState standard_state = MakePrior(0.0f, 10.0f, 100.0f, 25.0f);
  GaussianTrackState udkf_state = standard_state;
  GaussianTrackState srif_state = standard_state;

  for (int cycle = 1; cycle <= 12; ++cycle) {
    const float true_x = 10.0f * static_cast<float>(cycle);
    const MeasurementVector measurement(true_x + 0.3f, -0.2f, 0.1f);
    standard_state = standard_updater.Update(standard_predictor.Predict(standard_state, 1.0f),
                                             measurement)
                         .posterior;
    udkf_state = udkf_updater.Update(udkf_predictor.Predict(udkf_state, 1.0f), measurement).posterior;
    srif_state = srif_updater.Update(srif_predictor.Predict(srif_state, 1.0f), measurement).posterior;
  }

  const GaussianTrackState* final_states[] = {&standard_state, &udkf_state, &srif_state};
  for (std::size_t i = 0; i < sizeof(final_states) / sizeof(final_states[0]); ++i) {
    EXPECT_TRUE(final_states[i]->mean.allFinite());
    EXPECT_TRUE(final_states[i]->covariance.allFinite());
    for (int diag = 0; diag < kStateDim; ++diag) {
      EXPECT_GT(final_states[i]->covariance(diag, diag), 0.0f);
    }
  }
}

// ============================================================================
// Predict-Update 联合测试
// ============================================================================

TEST(KalmanPredictUpdateTest, PredictThenUpdateIntegration) {
  KalmanPredictorConfig pred_config;
  pred_config.noise_diff_coeff = 1.0f;
  KalmanPredictor predictor(pred_config);

  KalmanUpdaterConfig upd_config;
  upd_config.measurement_noise_std = 5.0f;
  KalmanUpdater updater(upd_config);

  // 初始状态：X=0, VX=10
  GaussianTrackState state = MakePrior(0.0f, 10.0f, 1.0f, 1.0f);
  const float dt = 1.0f;

  // 模拟 5 个周期的 predict-update 循环
  for (int cycle = 1; cycle <= 5; ++cycle) {
    // 预测
    GaussianTrackState predicted = predictor.Predict(state, dt);

    // 真值：匀速运动 x = 10 * cycle
    const float true_x = 10.0f * static_cast<float>(cycle);
    // 量测 = 真值 + 小噪声
    MeasurementVector z(true_x + 0.5f, 0.0f, 0.0f);

    // 更新
    KalmanUpdateResult result = updater.Update(predicted, z);
    state = result.posterior;
  }

  // 5 个周期后，估计应接近真值 x=50
  EXPECT_NEAR(state.mean(0), 50.0f, 2.0f);
  // 速度估计应稳定在约 10
  EXPECT_NEAR(state.mean(1), 10.0f, 2.0f);
  // 协方差应收敛（测量噪声 std=5 → 方差≤25；稳态值应明显低于初始值 1.0 的 10 倍）
  EXPECT_LT(state.covariance(0, 0), 30.0f);
}

TEST(KalmanPredictUpdateTest, VelocityConvergesFromWrongInitialGuess) {
  KalmanPredictorConfig pred_config;
  pred_config.noise_diff_coeff = 1.0f;
  KalmanPredictor predictor(pred_config);

  KalmanUpdaterConfig upd_config;
  upd_config.measurement_noise_std = 1.0f;
  KalmanUpdater updater(upd_config);

  // 初始速度猜测完全错误: vx=0（真值是 20）
  GaussianTrackState state = MakePrior(0.0f, 0.0f, 100.0f, 100.0f);
  const float dt = 1.0f;

  // 模拟 20 个周期
  for (int cycle = 1; cycle <= 20; ++cycle) {
    GaussianTrackState predicted = predictor.Predict(state, dt);
    const float true_x = 20.0f * static_cast<float>(cycle);
    MeasurementVector z(true_x, 0.0f, 0.0f);
    KalmanUpdateResult result = updater.Update(predicted, z);
    state = result.posterior;
  }

  // 位置和速度都应收敛
  EXPECT_NEAR(state.mean(0), 400.0f, 5.0f);  // x = 20*20 = 400
  EXPECT_NEAR(state.mean(1), 20.0f, 3.0f);   // vx → 20
}

}  // namespace tests
}  // namespace airborne_radar
