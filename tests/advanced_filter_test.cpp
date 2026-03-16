// Copyright 2026. All Rights Reserved.
//
// Description: 验证 FullMahalanobisDistanceMetric、EkfFilter 和 ImmFilter 的正确性。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "1q/airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/tracking/EkfFilter.h"
#include "airborne_radar/signal/tracking/ImmFilter.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar { namespace tests {

namespace {

constexpr float kTolerance = 1e-3f;

using signal::tracking::GaussianTrackState;
using signal::tracking::StateVector;
using signal::tracking::StateCovariance;
using signal::tracking::MeasurementVector;
using signal::tracking::kStateDim;

}  // namespace

// ============================================================================
// FullMahalanobisDistanceMetric 测试
// ============================================================================

TEST(FullMahalanobisTest, DiagonalMatchesSimplifiedVersion) {
  // 对角协方差时，Full 版本应与简化版结果一致
  const float sigma_speed = 40.0f;
  const float sigma_rcs = 8.0f;
  const float sigma_accel = 10.0f;

  signal::association::MahalanobisDistanceMetric simple(
      sigma_speed, sigma_rcs, sigma_accel);
  signal::association::FullMahalanobisDistanceMetric full(
      sigma_speed, sigma_rcs, sigma_accel);

  const Eigen::Vector3f predicted(100.0f, 2.0f, 1.0f);
  const Eigen::Vector3f measured(110.0f, 3.0f, 2.0f);

  const float d_simple = simple.Compute(predicted, measured);
  const float d_full = full.Compute(predicted, measured);

  EXPECT_NEAR(d_full, d_simple, kTolerance);
}

TEST(FullMahalanobisTest, CorrelatedCovarianceWorksCorrectly) {
  // 非对角协方差的手动验证
  Eigen::Matrix3f S;
  S << 4.0f, 1.0f, 0.0f,
       1.0f, 4.0f, 0.0f,
       0.0f, 0.0f, 1.0f;

  signal::association::FullMahalanobisDistanceMetric metric(S);

  const Eigen::Vector3f predicted(0.0f, 0.0f, 0.0f);
  const Eigen::Vector3f measured(1.0f, 0.0f, 0.0f);

  const float distance = metric.Compute(predicted, measured);

  // S⁻¹ for above S: [[4/15, -1/15, 0], [-1/15, 4/15, 0], [0, 0, 1]]
  // d² = [1,0,0] · S⁻¹ · [1,0,0]ᵀ = 4/15 ≈ 0.2667
  EXPECT_NEAR(distance, 4.0f / 15.0f, kTolerance);
}

TEST(FullMahalanobisTest, IdentityCovarianceGivesEuclidean) {
  signal::association::FullMahalanobisDistanceMetric metric(
      Eigen::Matrix3f::Identity());

  const Eigen::Vector3f a(1.0f, 2.0f, 3.0f);
  const Eigen::Vector3f b(4.0f, 6.0f, 3.0f);

  const float distance = metric.Compute(a, b);
  const float expected = 9.0f + 16.0f + 0.0f;  // 3² + 4² + 0²

  EXPECT_NEAR(distance, expected, kTolerance);
}

TEST(FullMahalanobisTest, SetInnovationCovarianceUpdates) {
  signal::association::FullMahalanobisDistanceMetric metric(
      Eigen::Matrix3f::Identity());

  const Eigen::Vector3f a(0.0f, 0.0f, 0.0f);
  const Eigen::Vector3f b(1.0f, 0.0f, 0.0f);

  const float d1 = metric.Compute(a, b);
  EXPECT_NEAR(d1, 1.0f, kTolerance);

  // 放大方差 → 距离减小
  metric.SetInnovationCovariance(Eigen::Matrix3f::Identity() * 4.0f);
  const float d2 = metric.Compute(a, b);
  EXPECT_NEAR(d2, 0.25f, kTolerance);
}

// ============================================================================
// EKF 测试
// ============================================================================

TEST(EkfPredictorTest, LinearModelMatchesStandardKalman) {
  // 当 f 和 F 是线性的（CV 模型），EKF 应等价于标准 KF
  signal::tracking::KalmanPredictorConfig kf_config;
  kf_config.noise_diff_coeff = 2.0f;
  signal::tracking::KalmanPredictor kf(kf_config);

  // EKF 使用线性 CV 模型（与 KF 数学等价）
  signal::tracking::LinearCvTransitionModel cv_model;
  signal::tracking::EkfPredictorConfig ekf_config;
  ekf_config.noise_diff_coeff = 2.0f;
  signal::tracking::EkfPredictor ekf(&cv_model, ekf_config);

  StateVector mean;
  mean << 100.0f, 10.0f, 50.0f, -5.0f, 200.0f, 3.0f;
  GaussianTrackState prior(mean, StateCovariance::Identity() * 50.0f);

  const GaussianTrackState kf_pred = kf.Predict(prior, 0.5f);
  const GaussianTrackState ekf_pred = ekf.Predict(prior, 0.5f);

  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_NEAR(ekf_pred.mean(i), kf_pred.mean(i), kTolerance)
        << "mean mismatch at " << i;
  }
  for (int r = 0; r < kStateDim; ++r) {
    for (int c = 0; c < kStateDim; ++c) {
      EXPECT_NEAR(ekf_pred.covariance(r, c), kf_pred.covariance(r, c),
                  kTolerance)
          << "cov mismatch at (" << r << "," << c << ")";
    }
  }
}

TEST(EkfUpdaterTest, LinearModelMatchesStandardKalman) {
  signal::tracking::KalmanUpdaterConfig kf_config;
  kf_config.measurement_noise_std = 5.0f;
  signal::tracking::KalmanUpdater kf(kf_config);

  signal::tracking::LinearPositionMeasurementModel meas_model;
  signal::tracking::EkfUpdaterConfig ekf_config;
  ekf_config.measurement_noise_std = 5.0f;
  signal::tracking::EkfUpdater ekf(&meas_model, ekf_config);

  StateVector mean;
  mean << 100.0f, 10.0f, 50.0f, -5.0f, 200.0f, 3.0f;
  GaussianTrackState predicted(mean, StateCovariance::Identity() * 100.0f);
  MeasurementVector z(105.0f, 48.0f, 202.0f);

  const auto kf_result = kf.Update(predicted, z);
  const auto ekf_result = ekf.Update(predicted, z);

  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_NEAR(ekf_result.posterior.mean(i), kf_result.posterior.mean(i),
                kTolerance)
        << "posterior mean mismatch at " << i;
  }
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

  signal::tracking::ImmFilter imm(
      imm_config,
      {&predictor},
      {&updater});

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
    EXPECT_NEAR(imm_state.mean(i), kf_state.mean(i), kTolerance)
        << "mean mismatch at " << i;
  }
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
  imm_config.transition_probability << 0.95f, 0.05f,
                                        0.05f, 0.95f;
  imm_config.initial_weights.resize(2);
  imm_config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter imm(
      imm_config,
      {&pred1, &pred2},
      {&upd1, &upd2});

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
  EXPECT_GT(weights(0), weights(1))
      << "低机动模型应在匀速运动中占优";
  EXPECT_GT(weights(0), 0.7f)
      << "低机动模型权重应显著大于 0.5";
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
  signal::tracking::KalmanUpdater upd1(ucfg), upd2(ucfg);

  signal::tracking::ImmConfig imm_config;
  imm_config.transition_probability.resize(2, 2);
  imm_config.transition_probability << 0.95f, 0.05f,
                                        0.05f, 0.95f;
  imm_config.initial_weights.resize(2);
  imm_config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter imm(
      imm_config, {&pred1, &pred2}, {&upd1, &upd2});

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
  EXPECT_GT(weights_after(1), weights_before(1))
      << "高机动模型权重应在急转弯后增加";
}

TEST(ImmFilterTest, ModelWeightsSumToOne) {
  signal::tracking::KalmanPredictor pred1, pred2;
  signal::tracking::KalmanUpdater upd1, upd2;

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
    EXPECT_NEAR(w.sum(), 1.0f, kTolerance)
        << "weights should sum to 1 at cycle " << i;
  }
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
  signal::tracking::KalmanUpdater upd1(ucfg), upd2(ucfg);

  signal::tracking::ImmConfig config;
  config.transition_probability.resize(2, 2);
  config.transition_probability << 0.9f, 0.1f,
                                   0.1f, 0.9f;
  config.initial_weights.resize(2);
  config.initial_weights << 0.5f, 0.5f;

  signal::tracking::ImmFilter first(config, {&pred1, &pred2}, {&upd1, &upd2});
  signal::tracking::ImmFilter second(config, {&pred1, &pred2}, {&upd1, &upd2});

  GaussianTrackState init(StateVector::Zero(),
                          StateCovariance::Identity() * 100.0f);
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

} } // namespace airborne_radar::tests
