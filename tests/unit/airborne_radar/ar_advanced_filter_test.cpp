// Copyright 2026. All Rights Reserved.
//
// @file advanced_filter_test.cpp
// @brief 验证 FullMahalanobisDistanceMetric 和 ImmFilter 的正确性。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/ImmFilter.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace tests {

namespace {

constexpr float kTolerance = 1e-3f;

using signal::tracking::GaussianTrackState;
using signal::tracking::kStateDim;
using signal::tracking::MeasurementCovariance;
using signal::tracking::MeasurementVector;
using signal::tracking::StateCovariance;
using signal::tracking::StateVector;

}  // namespace

// ============================================================================
// FullMahalanobisDistanceMetric 测试
// ============================================================================

TEST(FullMahalanobisTest, DiagonalMatchesSimplifiedVersion) {
  // 对角协方差时，Full 版本应与简化版结果一致
  const float sigma_speed = 40.0f;
  const float sigma_rcs = 8.0f;
  const float sigma_accel = 10.0f;

  signal::association::MahalanobisDistanceMetric simple(sigma_speed, sigma_rcs, sigma_accel);
  signal::association::FullMahalanobisDistanceMetric full(sigma_speed, sigma_rcs, sigma_accel);

  const Eigen::Vector3f predicted(100.0f, 2.0f, 1.0f);
  const Eigen::Vector3f measured(110.0f, 3.0f, 2.0f);

  const float d_simple = simple.Compute(predicted, measured);
  const float d_full = full.Compute(predicted, measured);

  EXPECT_NEAR(d_full, d_simple, kTolerance);
}

TEST(FullMahalanobisTest, CorrelatedCovarianceWorksCorrectly) {
  // 非对角协方差的手动验证
  Eigen::Matrix3f S;
  S << 4.0f, 1.0f, 0.0f, 1.0f, 4.0f, 0.0f, 0.0f, 0.0f, 1.0f;

  signal::association::FullMahalanobisDistanceMetric metric(S);

  const Eigen::Vector3f predicted(0.0f, 0.0f, 0.0f);
  const Eigen::Vector3f measured(1.0f, 0.0f, 0.0f);

  const float distance = metric.Compute(predicted, measured);

  // S⁻¹ for above S: [[4/15, -1/15, 0], [-1/15, 4/15, 0], [0, 0, 1]]
  // d² = [1,0,0] · S⁻¹ · [1,0,0]ᵀ = 4/15 ≈ 0.2667
  EXPECT_NEAR(distance, 4.0f / 15.0f, kTolerance);
}

TEST(FullMahalanobisTest, IdentityCovarianceGivesEuclidean) {
  signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());

  const Eigen::Vector3f a(1.0f, 2.0f, 3.0f);
  const Eigen::Vector3f b(4.0f, 6.0f, 3.0f);

  const float distance = metric.Compute(a, b);
  const float expected = 9.0f + 16.0f + 0.0f;  // 3² + 4² + 0²

  EXPECT_NEAR(distance, expected, kTolerance);
}

TEST(FullMahalanobisTest, SetInnovationCovarianceUpdates) {
  signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());

  const Eigen::Vector3f a(0.0f, 0.0f, 0.0f);
  const Eigen::Vector3f b(1.0f, 0.0f, 0.0f);

  const float d1 = metric.Compute(a, b);
  EXPECT_NEAR(d1, 1.0f, kTolerance);

  // 放大方差 → 距离减小
  metric.SetInnovationCovariance(Eigen::Matrix3f::Identity() * 4.0f);
  const float d2 = metric.Compute(a, b);
  EXPECT_NEAR(d2, 0.25f, kTolerance);
}

TEST(FullMahalanobisTest, InvalidCovarianceReturnsInfiniteDistance) {
  signal::association::FullMahalanobisDistanceMetric metric(Eigen::Matrix3f::Identity());
  metric.SetInnovationCovariance(Eigen::Matrix3f::Zero());

  const Eigen::Vector3f predicted(0.0f, 0.0f, 0.0f);
  const Eigen::Vector3f measured(1.0f, 0.0f, 0.0f);
  const float distance = metric.Compute(predicted, measured);
  EXPECT_TRUE(std::isinf(distance));
}

// ============================================================================
// IMM 测试
// ============================================================================

TEST(ImmFilterTest, SingleModelEquivalentToKF) {
  // 单模型 IMM 应等价于纯 KF
  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);

  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 5.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  // IMM with 1 model
  signal::tracking::ImmConfig imm_config;
  imm_config.transition_probability = Eigen::MatrixXf::Ones(1, 1);
  imm_config.initial_weights = Eigen::VectorXf::Ones(1);

  signal::tracking::ImmFilter imm(imm_config, {&predictor}, {&updater});

  // 初始化
  StateVector mean = StateVector::Zero();
  mean(0) = 100.0f;
  mean(1) = 10.0f;
  GaussianTrackState init(mean, StateCovariance::Identity() * 50.0f);

  signal::tracking::ImmModelState model_state;
  model_state.state = init;
  model_state.weight = 1.0f;
  imm.SetModelStates({model_state});

  // 同时用纯 KF 作参照
  GaussianTrackState kf_state = init;

  const float dt = 1.0f;
  MeasurementVector z(110.5f, 0.0f, 0.0f);

  kf_state = predictor.Predict(kf_state, dt);
  auto kf_result = updater.Update(kf_state, z);
  kf_state = kf_result.posterior;

  imm.Process(z, dt);
  auto imm_state = imm.GetCombinedState();

  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_NEAR(imm_state.mean(i), kf_state.mean(i), kTolerance) << "mean mismatch at " << i;
  }
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < kStateDim; ++j) {
      EXPECT_NEAR(imm_state.covariance(i, j), kf_state.covariance(i, j), kTolerance)
          << "covariance mismatch at (" << i << "," << j << ")";
    }
  }
}

TEST(ImmFilterTest, SingleModelImmWithKfBackendRunsFinite) {
  // 单模型 IMM(KF) 应等价于纯 KF
  signal::tracking::KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 2.0f;
  signal::tracking::KalmanPredictor predictor(pred_cfg);
  signal::tracking::KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = 2.0f;
  signal::tracking::KalmanUpdater updater(upd_cfg);

  signal::tracking::ImmConfig config;
  config.transition_probability = Eigen::MatrixXf::Ones(1, 1);
  config.initial_weights = Eigen::VectorXf::Ones(1);

  signal::tracking::ImmFilter imm(config, {&predictor}, {&updater});

  StateVector mean = StateVector::Zero();
  mean(0) = 50.0f;
  mean(1) = 7.0f;
  GaussianTrackState init(mean, StateCovariance::Identity() * 20.0f);

  imm.SetModelStates({signal::tracking::ImmModelState(init, 1.0f)});

  for (int cycle = 1; cycle <= 8; ++cycle) {
    const float x = 50.0f + 7.0f * static_cast<float>(cycle);
    MeasurementVector measurement(x, 0.0f, 0.0f);
    imm.Process(measurement, 1.0f);
  }

  const GaussianTrackState state = imm.GetCombinedState();
  EXPECT_TRUE(state.mean.allFinite());
  EXPECT_TRUE(state.covariance.allFinite());
  EXPECT_NEAR(imm.GetModelWeights()(0), 1.0f, kTolerance);
}

TEST(ImmFilterTest, TwoModelWeightsConvergeToCorrectModel) {
  // 模型1: CV q=1 (低机动), 模型2: CV q=50 (高机动)
  signal::tracking::KalmanPredictorConfig cfg1;
  cfg1.noise_diff_coeff = 1.0f;
  signal::tracking::KalmanPredictor pred1(cfg1);
  signal::tracking::KalmanUpdaterConfig ucfg;
  ucfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater upd1(ucfg);

  signal::tracking::KalmanPredictorConfig cfg2;
  cfg2.noise_diff_coeff = 50.0f;
  signal::tracking::KalmanPredictor pred2(cfg2);
  signal::tracking::KalmanUpdater upd2(ucfg);

  signal::tracking::ImmConfig imm_config;
  imm_config.transition_probability.resize(2, 2);
  imm_config.transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  imm_config.initial_weights.resize(2);
  imm_config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter imm(imm_config, {&pred1, &pred2}, {&upd1, &upd2});

  StateVector mean = StateVector::Zero();
  mean(0) = 0.0f;
  mean(1) = 10.0f;
  GaussianTrackState init(mean, StateCovariance::Identity() * 100.0f);

  signal::tracking::ImmModelState m1{init, 0.5f};
  signal::tracking::ImmModelState m2{init, 0.5f};
  imm.SetModelStates({m1, m2});

  // 匀速运动 10 个周期 → 模型 1（低机动）应胜出
  for (int cycle = 1; cycle <= 10; ++cycle) {
    const float true_x = 10.0f * static_cast<float>(cycle);
    MeasurementVector z(true_x, 0.0f, 0.0f);
    imm.Process(z, 1.0f);
  }

  Eigen::VectorXf weights = imm.GetModelWeights();
  EXPECT_GT(weights(0), weights(1)) << "低机动模型应在匀速运动中占优";
  EXPECT_GT(weights(0), 0.7f) << "低机动模型权重应显著大于 0.5";
}

TEST(ImmFilterTest, ModelWeightsShiftOnManeuver) {
  // 先匀速 → 然后急转弯 → 高机动模型权重应增加
  signal::tracking::KalmanPredictorConfig cfg1;
  cfg1.noise_diff_coeff = 0.5f;
  signal::tracking::KalmanPredictor pred1(cfg1);

  signal::tracking::KalmanPredictorConfig cfg2;
  cfg2.noise_diff_coeff = 100.0f;
  signal::tracking::KalmanPredictor pred2(cfg2);

  signal::tracking::KalmanUpdaterConfig ucfg;
  ucfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater upd1(ucfg);
  signal::tracking::KalmanUpdater upd2(ucfg);

  signal::tracking::ImmConfig imm_config;
  imm_config.transition_probability.resize(2, 2);
  imm_config.transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  imm_config.initial_weights.resize(2);
  imm_config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter imm(imm_config, {&pred1, &pred2}, {&upd1, &upd2});

  StateVector mean = StateVector::Zero();
  GaussianTrackState init(mean, StateCovariance::Identity() * 100.0f);
  signal::tracking::ImmModelState m1{init, 0.5f};
  signal::tracking::ImmModelState m2{init, 0.5f};
  imm.SetModelStates({m1, m2});

  // 匀速阶段
  for (int cycle = 1; cycle <= 10; ++cycle) {
    MeasurementVector z(10.0f * static_cast<float>(cycle), 0.0f, 0.0f);
    imm.Process(z, 1.0f);
  }

  Eigen::VectorXf weights_before = imm.GetModelWeights();

  // 急转弯阶段：位置突然偏离匀速预测
  for (int cycle = 11; cycle <= 15; ++cycle) {
    // 加速度 50 m/s² 的位置 = v0*t + 0.5*a*t²
    float t = static_cast<float>(cycle - 10);
    float x = 100.0f + 10.0f * t + 0.5f * 50.0f * t * t;
    MeasurementVector z(x, 0.0f, 0.0f);
    imm.Process(z, 1.0f);
  }

  Eigen::VectorXf weights_after = imm.GetModelWeights();

  // 高机动模型（模型 2）权重应在机动后增加
  EXPECT_GT(weights_after(1), weights_before(1)) << "高机动模型权重应在急转弯后增加";
}

TEST(ImmFilterTest, ModelWeightsSumToOne) {
  signal::tracking::KalmanPredictor pred1;
  signal::tracking::KalmanPredictor pred2;
  signal::tracking::KalmanUpdater upd1;
  signal::tracking::KalmanUpdater upd2;

  signal::tracking::ImmConfig config;
  config.transition_probability.resize(2, 2);
  config.transition_probability << 0.9f, 0.1f, 0.1f, 0.9f;
  config.initial_weights.resize(2);
  config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter imm(config, {&pred1, &pred2}, {&upd1, &upd2});

  GaussianTrackState init(StateVector::Zero(), StateCovariance::Identity() * 100.0f);
  signal::tracking::ImmModelState m{init, 0.5f};
  imm.SetModelStates({m, m});

  for (int i = 1; i <= 5; ++i) {
    MeasurementVector z(static_cast<float>(i) * 10.0f, 0.0f, 0.0f);
    imm.Process(z, 1.0f);

    Eigen::VectorXf w = imm.GetModelWeights();
    EXPECT_NEAR(w.sum(), 1.0f, kTolerance) << "weights should sum to 1 at cycle " << i;
  }
}

TEST(ImmFilterTest, ExtremeInnovationKeepsFiniteNormalizedWeights) {
  signal::tracking::KalmanPredictor pred1;
  signal::tracking::KalmanPredictor pred2;
  signal::tracking::KalmanUpdater upd1;
  signal::tracking::KalmanUpdater upd2;

  signal::tracking::ImmConfig config;
  config.transition_probability.resize(2, 2);
  config.transition_probability << 0.9f, 0.1f, 0.1f, 0.9f;
  config.initial_weights.resize(2);
  config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter imm(config, {&pred1, &pred2}, {&upd1, &upd2});
  GaussianTrackState init(StateVector::Zero(), StateCovariance::Identity() * 1.0f);
  signal::tracking::ImmModelState m{init, 0.5f};
  imm.SetModelStates({m, m});

  const MeasurementVector extreme_measurement(1.0e8f, -1.0e8f, 1.0e8f);
  imm.Process(extreme_measurement, 1.0f);

  const Eigen::VectorXf w = imm.GetModelWeights();
  ASSERT_EQ(w.size(), 2);
  EXPECT_TRUE(std::isfinite(w(0)));
  EXPECT_TRUE(std::isfinite(w(1)));
  EXPECT_GE(w(0), 0.0f);
  EXPECT_GE(w(1), 0.0f);
  EXPECT_NEAR(w.sum(), 1.0f, kTolerance);
}

TEST(ImmFilterTest, RepeatedProcessSequenceRemainsDeterministic) {
  signal::tracking::KalmanPredictorConfig cfg1;
  cfg1.noise_diff_coeff = 0.5f;
  signal::tracking::KalmanPredictor pred1(cfg1);

  signal::tracking::KalmanPredictorConfig cfg2;
  cfg2.noise_diff_coeff = 25.0f;
  signal::tracking::KalmanPredictor pred2(cfg2);

  signal::tracking::KalmanUpdaterConfig ucfg;
  ucfg.measurement_noise_std = 1.0f;
  signal::tracking::KalmanUpdater upd1(ucfg);
  signal::tracking::KalmanUpdater upd2(ucfg);

  signal::tracking::ImmConfig config;
  config.transition_probability.resize(2, 2);
  config.transition_probability << 0.9f, 0.1f, 0.1f, 0.9f;
  config.initial_weights.resize(2);
  config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter first(config, {&pred1, &pred2}, {&upd1, &upd2});
  signal::tracking::ImmFilter second(config, {&pred1, &pred2}, {&upd1, &upd2});

  GaussianTrackState init(StateVector::Zero(), StateCovariance::Identity() * 100.0f);
  signal::tracking::ImmModelState m1{init, 0.5f};
  signal::tracking::ImmModelState m2{init, 0.5f};
  first.SetModelStates({m1, m2});
  second.SetModelStates({m1, m2});

  const std::vector<MeasurementVector> measurements = {
      MeasurementVector(10.0f, 0.0f, 0.0f),
      MeasurementVector(21.0f, 0.0f, 0.0f),
      MeasurementVector(35.0f, 0.0f, 0.0f),
      MeasurementVector(54.0f, 0.0f, 0.0f),
  };

  for (std::size_t i = 0; i < measurements.size(); ++i) {
    first.Process(measurements[i], 1.0f);
    second.Process(measurements[i], 1.0f);

    const GaussianTrackState first_state = first.GetCombinedState();
    const GaussianTrackState second_state = second.GetCombinedState();
    const Eigen::VectorXf first_weights = first.GetModelWeights();
    const Eigen::VectorXf second_weights = second.GetModelWeights();

    for (int j = 0; j < kStateDim; ++j) {
      EXPECT_NEAR(first_state.mean(j), second_state.mean(j), kTolerance)
          << "mean mismatch at cycle " << i << " state index " << j;
    }
    for (int j = 0; j < first_weights.size(); ++j) {
      EXPECT_NEAR(first_weights(j), second_weights(j), kTolerance)
          << "weight mismatch at cycle " << i << " model index " << j;
    }
  }
}

}  // namespace tests
}  // namespace airborne_radar
