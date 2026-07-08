// Copyright 2026. All Rights Reserved.
//
// @file ar_backend_evaluation_test.cpp
// @brief IMM 机动跟踪收益的实质证据评估。
//
// 评估目标：
//   IMM — 同 CV 不同 q 的多模型融合对机动跟踪的 RMSE 改善是否显著
//
// 注：UDKF 评估已完成 —— 500 周期 + 病态初始化下 KF(Joseph) 与 UDKF 无差异，
// UDKF 已从 AR 模块移除（见 commit b65079ce 后续）。

#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/ImmFilter.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace tests {

using signal::tracking::GaussianTrackState;
using signal::tracking::ImmConfig;
using signal::tracking::ImmFilter;
using signal::tracking::ImmModelState;
using signal::tracking::KalmanPredictor;
using signal::tracking::KalmanPredictorConfig;
using signal::tracking::KalmanUpdater;
using signal::tracking::KalmanUpdaterConfig;
using signal::tracking::kStateDim;
using signal::tracking::MeasurementVector;
using signal::tracking::StateCovariance;
using signal::tracking::StateVector;

namespace {

constexpr float kTolerance = 1e-3f;

// ============================================================================
// 辅助工具
// ============================================================================

/// @brief 构造初始状态：x 轴位置/速度非零，其他轴零。协方差对角。
GaussianTrackState MakeState(float x, float vx, float pos_var = 100.0f,
                              float vel_var = 25.0f) {
  StateVector mean = StateVector::Zero();
  mean(0) = x;
  mean(1) = vx;
  StateCovariance P = StateCovariance::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    P(axis * 2, axis * 2) = pos_var;
    P(axis * 2 + 1, axis * 2 + 1) = vel_var;
  }
  return GaussianTrackState(mean, P);
}

/// @brief 位置 RMSE：sqrt(mean((x_est - x_true)²))
float PositionRmse(const std::vector<float>& errors) {
  if (errors.empty()) return 0.0f;
  float sum_sq = 0.0f;
  for (float e : errors) sum_sq += e * e;
  return std::sqrt(sum_sq / static_cast<float>(errors.size()));
}

// ============================================================================
// IMM 机动跟踪收益评估
// ============================================================================

struct TrackingResult {
  std::vector<float> position_errors;  // |x_est - x_true|
  std::vector<float> velocity_errors;  // |vx_est - vx_true|
};

/// @brief 单 CV KF 跟踪器，返回每周期位置误差。
TrackingResult RunSingleCvTracker(
    const std::vector<float>& true_positions,
    const std::vector<float>& measurements,
    float process_q, float meas_std, float dt = 1.0f) {
  KalmanPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = process_q;
  KalmanPredictor predictor(pred_cfg);
  KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = meas_std;
  KalmanUpdater updater(upd_cfg);

  GaussianTrackState state = MakeState(true_positions.front(), 0.0f, 100.0f, 10.0f);
  TrackingResult result;

  for (std::size_t i = 0; i < measurements.size(); ++i) {
    MeasurementVector z(measurements[i], 0.0f, 0.0f);
    state = updater.Update(predictor.Predict(state, dt), z).posterior;
    result.position_errors.push_back(std::abs(state.mean(0) - true_positions[i]));
    // 速度参考：相邻真值差分
    float true_vx = (i > 0) ? (true_positions[i] - true_positions[i - 1]) / dt : 0.0f;
    result.velocity_errors.push_back(std::abs(state.mean(1) - true_vx));
  }
  return result;
}

/// @brief IMM(CV×2, q 不同) 跟踪器。
TrackingResult RunImmCv2Tracker(
    const std::vector<float>& true_positions,
    const std::vector<float>& measurements,
    float q_low, float q_high, float meas_std, float dt = 1.0f) {
  KalmanPredictorConfig cfg_low;
  cfg_low.noise_diff_coeff = q_low;
  KalmanPredictor pred_low(cfg_low);
  KalmanPredictorConfig cfg_high;
  cfg_high.noise_diff_coeff = q_high;
  KalmanPredictor pred_high(cfg_high);

  KalmanUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = meas_std;
  KalmanUpdater upd_low(upd_cfg);
  KalmanUpdater upd_high(upd_cfg);

  ImmConfig imm_cfg;
  imm_cfg.transition_probability.resize(2, 2);
  imm_cfg.transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  imm_cfg.initial_weights.resize(2);
  imm_cfg.initial_weights << 0.5f, 0.5f;

  ImmFilter imm(imm_cfg, {&pred_low, &pred_high}, {&upd_low, &upd_high});

  GaussianTrackState init = MakeState(true_positions.front(), 0.0f, 100.0f, 10.0f);
  ImmModelState m1{init, 0.5f};
  ImmModelState m2{init, 0.5f};
  imm.SetModelStates({m1, m2});

  TrackingResult result;
  for (std::size_t i = 0; i < measurements.size(); ++i) {
    MeasurementVector z(measurements[i], 0.0f, 0.0f);
    imm.Process(z, dt);
    const GaussianTrackState combined = imm.GetCombinedState();
    result.position_errors.push_back(std::abs(combined.mean(0) - true_positions[i]));
    float true_vx = (i > 0) ? (true_positions[i] - true_positions[i - 1]) / dt : 0.0f;
    result.velocity_errors.push_back(std::abs(combined.mean(1) - true_vx));
  }
  return result;
}

TEST(BackendEvaluationTest, ImmCvVsSingleCvOnConstantVelocity) {
  // 匀速目标：IMM 不应比单 CV 差太多。两者 RMSE 应在同一量级。
  constexpr int kCycles = 100;
  std::vector<float> truth(kCycles);
  std::vector<float> meas(kCycles);
  for (int i = 0; i < kCycles; ++i) {
    truth[i] = 10.0f * static_cast<float>(i + 1);
    meas[i] = truth[i] + 2.0f;  // 小噪声
  }

  const TrackingResult single = RunSingleCvTracker(truth, meas, 0.5f, 5.0f);
  const TrackingResult imm = RunImmCv2Tracker(truth, meas, 0.5f, 50.0f, 5.0f);

  const float single_rmse = PositionRmse(single.position_errors);
  const float imm_rmse = PositionRmse(imm.position_errors);

  // 匀速场景下两者应接近
  EXPECT_NEAR(single_rmse, imm_rmse, 2.0f)
      << "CV: single_rmse=" << single_rmse << " imm_rmse=" << imm_rmse;
}

TEST(BackendEvaluationTest, ImmCvVsSingleCvOnSuddenManeuver) {
  // 前 20 周期匀速，第 21 周期起恒定加速度 20 m/s² → 考验机动响应。
  constexpr int kCycles = 60;
  constexpr float kDt = 1.0f;
  std::vector<float> truth(kCycles);
  std::vector<float> meas(kCycles);

  for (int i = 0; i < kCycles; ++i) {
    const float t = static_cast<float>(i) * kDt;
    if (i < 20) {
      truth[i] = 10.0f * t;
    } else {
      const float t_mnv = t - 20.0f * kDt;
      truth[i] = 200.0f + 10.0f * t_mnv + 0.5f * 20.0f * t_mnv * t_mnv;
    }
    meas[i] = truth[i] + 3.0f;
  }

  const TrackingResult single = RunSingleCvTracker(truth, meas, 1.0f, 5.0f);
  const TrackingResult imm = RunImmCv2Tracker(truth, meas, 1.0f, 100.0f, 5.0f);

  // 机动发生后（第 21-60 周期）的 RMSE：IMM 应优于单 CV
  std::vector<float> single_mnv(single.position_errors.begin() + 20, single.position_errors.end());
  std::vector<float> imm_mnv(imm.position_errors.begin() + 20, imm.position_errors.end());
  const float single_mnv_rmse = PositionRmse(single_mnv);
  const float imm_mnv_rmse = PositionRmse(imm_mnv);

  // 机动阶段 IMM 的位置 RMSE 应低于单 CV
  EXPECT_LT(imm_mnv_rmse, single_mnv_rmse)
      << "IMM should outperform single CV during maneuver: "
      << "IMM=" << imm_mnv_rmse << " vs CV=" << single_mnv_rmse;

  // 量化改善幅度
  const float improvement_pct = (single_mnv_rmse - imm_mnv_rmse) / single_mnv_rmse * 100.0f;
  (void)improvement_pct;  // 供分析用：>10% 则 IMM 有实质收益
}

TEST(BackendEvaluationTest, ImmCvVsSingleCvOnSustainedTurn) {
  // 持续转弯：x 匀速，y 方向余弦摆动 → CV 模型持续失配。
  constexpr int kCycles = 80;
  constexpr float kDt = 1.0f;
  std::vector<float> truth_x(kCycles);
  std::vector<float> meas_x(kCycles);

  for (int i = 0; i < kCycles; ++i) {
    const float t = static_cast<float>(i) * kDt;
    // 匀速直线 + 横向正弦摆动（模拟转弯的 x 投影）
    truth_x[i] = 50.0f * t + 30.0f * std::sin(0.15f * t);
    meas_x[i] = truth_x[i] + 2.0f;
  }

  const TrackingResult single = RunSingleCvTracker(truth_x, meas_x, 0.5f, 5.0f);
  const TrackingResult imm = RunImmCv2Tracker(truth_x, meas_x, 0.5f, 80.0f, 5.0f);

  const float single_rmse = PositionRmse(single.position_errors);
  const float imm_rmse = PositionRmse(imm.position_errors);

  // 持续失配场景下 IMM 应优于单 CV
  EXPECT_LT(imm_rmse, single_rmse)
      << "IMM should outperform single CV in sustained maneuver: "
      << "IMM=" << imm_rmse << " vs CV=" << single_rmse;

  const float improvement_pct = (single_rmse - imm_rmse) / single_rmse * 100.0f;
  (void)improvement_pct;
}

TEST(BackendEvaluationTest, ImmImprovementMagnitudeAcrossScenarios) {
  // 汇总三种场景下 IMM 相对于单 CV 的改善幅度，给出保留/移除决策的量化依据。
  //
  // 阈值约定：
  //   <5%   → 噪声级改善，不足以论证 IMM 复杂性
  //   5-15% → 边际改善，取决于场景频率
  //   >15%  → 实质改善，IMM 有价值

  struct Scenario {
    const char* name;
    float single_rmse;
    float imm_rmse;
    float improvement_pct;
  };

  std::vector<Scenario> scenarios;

  // 场景 1：匀速 100 周期
  {
    constexpr int N = 100;
    std::vector<float> truth(N), meas(N);
    for (int i = 0; i < N; ++i) {
      truth[i] = 10.0f * static_cast<float>(i + 1);
      meas[i] = truth[i] + 3.0f;
    }
    auto s = RunSingleCvTracker(truth, meas, 0.5f, 5.0f);
    auto i = RunImmCv2Tracker(truth, meas, 0.5f, 50.0f, 5.0f);
    float sr = PositionRmse(s.position_errors), ir = PositionRmse(i.position_errors);
    scenarios.push_back({"CV", sr, ir, (sr - ir) / sr * 100.0f});
  }

  // 场景 2：急转弯
  {
    constexpr int N = 60;
    std::vector<float> truth(N), meas(N);
    for (int t = 0; t < N; ++t) {
      if (t < 20) truth[t] = 10.0f * static_cast<float>(t);
      else truth[t] = 200.0f + 10.0f * (t - 20) + 10.0f * (t - 20) * (t - 20);
      meas[t] = truth[t] + 3.0f;
    }
    auto s = RunSingleCvTracker(truth, meas, 1.0f, 5.0f);
    auto i = RunImmCv2Tracker(truth, meas, 1.0f, 100.0f, 5.0f);
    float sr = PositionRmse(s.position_errors), ir = PositionRmse(i.position_errors);
    scenarios.push_back({"Maneuver", sr, ir, (sr - ir) / sr * 100.0f});
  }

  // 场景 3：持续正弦摆动
  {
    constexpr int N = 80;
    std::vector<float> truth(N), meas(N);
    for (int t = 0; t < N; ++t) {
      truth[t] = 50.0f * static_cast<float>(t) + 30.0f * std::sin(0.15f * static_cast<float>(t));
      meas[t] = truth[t] + 2.0f;
    }
    auto s = RunSingleCvTracker(truth, meas, 0.5f, 5.0f);
    auto i = RunImmCv2Tracker(truth, meas, 0.5f, 80.0f, 5.0f);
    float sr = PositionRmse(s.position_errors), ir = PositionRmse(i.position_errors);
    scenarios.push_back({"Sinusoidal", sr, ir, (sr - ir) / sr * 100.0f});
  }

  // 输出汇总表
  std::cout << "\n=== IMM vs Single CV Improvement Summary ===\n";
  for (const auto& sc : scenarios) {
    std::cout << "[" << sc.name << "] single_rmse=" << sc.single_rmse
              << " imm_rmse=" << sc.imm_rmse
              << " improvement=" << sc.improvement_pct << "%\n";
  }
  std::cout << "Threshold: <5%=noise, 5-15%=marginal, >15%=meaningful\n";

  // 验证所有场景数据有效
  for (const auto& sc : scenarios) {
    EXPECT_GT(sc.improvement_pct, -100.0f) << sc.name << " data invalid";
  }
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
