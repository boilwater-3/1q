// Copyright 2026. All Rights Reserved.
//
// @file sbirs_imm_evaluation_test.cpp
// @brief SBIRS IMM 机动跟踪收益评估：比较 EKF(CV) vs IMM(EKF×2) 在红外角度量测场景下的表现。
//
// SBIRS 与 AR 的关键差异：
//   - 量测：2D 球坐标角度 [az, el]（非线性），无距离信息
//   - 滤波：EKF（Jacobian 一阶线性化），非标准线性 KF
//   - 场景：助推段加速度、中段匀速、末端机动
//   - 卫星在轨 (~7000km)，目标在地球表面附近

#include <gtest/gtest.h>

#include <Eigen/Cholesky>

#include <cmath>
#include <iostream>
#include <vector>

#include "common/estimation/EkfFilter.h"
#include "common/estimation/GaussianState.h"
#include "common/estimation/ImmFilter.h"
#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace {

using namespace sbirs_sensor;
using namespace sbirs_sensor::tracking;

constexpr int kDim = kSbirsStateDim;
constexpr int kMeas = kSbirsMeasurementDim;

session::SbirsVector3M Vector(double x, double y, double z) {
  session::SbirsVector3M v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

// IMM facade: 6 维状态 / 2 维角度量测
using SbirsImmConfig = ::oneq::common::estimation::ImmConfig;
using SbirsImmFilter = ::oneq::common::estimation::ImmFilter<kDim, kMeas>;
using SbirsImmModelState = ::oneq::common::estimation::ImmModelState<kDim, kMeas>;

/// @brief 位置 RMSE（仅 x 分量，对应沿轨方向）。
float PositionRmse(const std::vector<float>& errors) {
  if (errors.empty()) return 0.0f;
  float sum_sq = 0.0f;
  for (float e : errors) sum_sq += e * e;
  return std::sqrt(sum_sq / static_cast<float>(errors.size()));
}

/// @brief 构造 SBIRS 典型初始状态：目标在卫星前方 ~1000km，沿 Y 轴运动。
SbirsGaussianState MakeSbirsState(double y, double vy, float pos_var = 100.0f,
                                   float vel_var = 25.0f) {
  SbirsGaussianState s;
  s.mean.setZero();
  s.mean(0) = 8000000.0f;  // x ~地球半径+大气层高度
  s.mean(1) = 0.0f;         // vx
  s.mean(2) = static_cast<float>(y);
  s.mean(3) = static_cast<float>(vy);
  s.mean(4) = 0.0f;         // z
  s.mean(5) = 0.0f;         // vz
  s.covariance = SbirsStateCovariance::Zero();
  for (int i = 0; i < 6; i += 2) {
    s.covariance(i, i) = pos_var;
    s.covariance(i + 1, i + 1) = vel_var;
  }
  return s;
}

/// @brief 将角度 sigma（deg）转为弧度 sigma，供 EkfUpdaterConfig 使用。
/// @note EkfUpdater 的 BuildDefaultMeasurementNoise 返回 I * sigma_rad²，
///       sigma_rad 即量测噪声标准差（弧度）。
float AngularSigmaDegToRad(float sigma_deg) {
  return sigma_deg * 3.14159265358979f / 180.0f;
}

// ============================================================================
// 单 EKF(CV) 跟踪器
// ============================================================================

struct EvalResult {
  std::vector<float> position_errors;   // |y_est - y_true| 沿轨方向
  std::vector<float> nis_values;        // 每周期 NIS
};

EvalResult RunSingleEkf(
    const std::vector<double>& truth_y,   // 真值 Y 位置序列
    const std::vector<double>& truth_vy,  // 真值 Y 速度序列
    float process_q, float angular_sigma_deg = 0.02f) {
  SbirsCvTransitionModel cv;
  SbirsEkfPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = process_q;
  SbirsEkfPredictor predictor(&cv, pred_cfg);

  SbirsAngleMeasurementModel meas_model;
  meas_model.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));

  SbirsEkfUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = AngularSigmaDegToRad(angular_sigma_deg);
  SbirsEkfUpdater updater(&meas_model, upd_cfg);

  SbirsGaussianState state = MakeSbirsState(truth_y[0], truth_vy[0]);
  EvalResult result;

  for (std::size_t i = 0; i < truth_y.size(); ++i) {
    meas_model.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));
    SbirsStateVector truth_state = SbirsStateVector::Zero();
    truth_state(0) = 8000000.0f;
    truth_state(2) = static_cast<float>(truth_y[i]);
    truth_state(3) = static_cast<float>(truth_vy[i]);

    const SbirsMeasurementVector z = meas_model.Function(truth_state);

    const SbirsGaussianState predicted = predictor.Predict(state, 1.0f);
    const SbirsKalmanUpdateResult update_result = updater.Update(predicted, z);
    state = update_result.posterior;

    result.position_errors.push_back(std::abs(state.mean(2) - static_cast<float>(truth_y[i])));

    // NIS
    const Eigen::LLT<SbirsMeasurementCovariance> llt(update_result.innovation_covariance);
    if (llt.info() == Eigen::Success) {
      const SbirsMeasurementVector solved = llt.solve(update_result.innovation);
      result.nis_values.push_back(update_result.innovation.dot(solved));
    } else {
      result.nis_values.push_back(std::numeric_limits<float>::infinity());
    }
  }
  return result;
}

// ============================================================================
// IMM(EKF×2) 跟踪器
// ============================================================================

EvalResult RunImmEkf2(
    const std::vector<double>& truth_y,
    const std::vector<double>& truth_vy,
    float q_low, float q_high, float angular_sigma_deg = 0.02f) {
  SbirsCvTransitionModel cv;
  SbirsEkfPredictorConfig cfg_low;
  cfg_low.noise_diff_coeff = q_low;
  auto pred_low = std::make_unique<SbirsEkfPredictor>(&cv, cfg_low);
  SbirsEkfPredictorConfig cfg_high;
  cfg_high.noise_diff_coeff = q_high;
  auto pred_high = std::make_unique<SbirsEkfPredictor>(&cv, cfg_high);

  SbirsAngleMeasurementModel meas_low;
  meas_low.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));
  SbirsAngleMeasurementModel meas_high;
  meas_high.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));

  SbirsEkfUpdaterConfig upd_cfg;
  upd_cfg.measurement_noise_std = AngularSigmaDegToRad(angular_sigma_deg);
  auto upd_low = std::make_unique<SbirsEkfUpdater>(&meas_low, upd_cfg);
  auto upd_high = std::make_unique<SbirsEkfUpdater>(&meas_high, upd_cfg);

  std::vector<::oneq::common::estimation::IKalmanPredictor<kDim, kMeas>*> predictors = {
      pred_low.get(), pred_high.get()};
  std::vector<::oneq::common::estimation::IKalmanUpdater<kDim, kMeas>*> updaters = {
      upd_low.get(), upd_high.get()};

  SbirsImmConfig imm_cfg;
  imm_cfg.transition_probability.resize(2, 2);
  imm_cfg.transition_probability << 0.95f, 0.05f, 0.05f, 0.95f;
  imm_cfg.initial_weights.resize(2);
  imm_cfg.initial_weights << 0.5f, 0.5f;

  SbirsImmFilter imm(imm_cfg, predictors, updaters);

  SbirsGaussianState init = MakeSbirsState(truth_y[0], truth_vy[0]);
  SbirsImmModelState m1{init, 0.5f};
  SbirsImmModelState m2{init, 0.5f};
  imm.SetModelStates({m1, m2});

  EvalResult result;

  for (std::size_t i = 0; i < truth_y.size(); ++i) {
    // 更新量测模型中的卫星位置（两个模型共享几何）
    meas_low.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));
    meas_high.SetSatellitePosition(Vector(7000000.0, 0.0, 0.0));

    SbirsStateVector truth_state = SbirsStateVector::Zero();
    truth_state(0) = 8000000.0f;
    truth_state(2) = static_cast<float>(truth_y[i]);
    truth_state(3) = static_cast<float>(truth_vy[i]);

    const SbirsMeasurementVector z = meas_low.Function(truth_state);

    imm.Process(z, 1.0f);
    const SbirsGaussianState combined = imm.GetCombinedState();
    result.position_errors.push_back(std::abs(combined.mean(2) - static_cast<float>(truth_y[i])));

    // IMM 不直接暴露单模型 innovation，NIS 在此不计算
    result.nis_values.push_back(0.0f);
  }
  return result;
}

// ============================================================================
// 场景定义
// ============================================================================

/// @brief 中段匀速：目标以恒定速度沿 Y 轴运动（CV 模型完全匹配）。
std::pair<std::vector<double>, std::vector<double>> MakeCvScenario(int cycles, double y0 = 0.0,
                                                                     double vy = 3000.0) {
  std::vector<double> y(cycles), vy_vec(cycles);
  for (int i = 0; i < cycles; ++i) {
    const double t = static_cast<double>(i);
    y[i] = y0 + vy * t;
    vy_vec[i] = vy;
  }
  return {y, vy_vec};
}

/// @brief 助推段：恒定加速度（CV 模型持续失配）。
std::pair<std::vector<double>, std::vector<double>> MakeBoostScenario(int cycles, double y0 = 0.0,
                                                                        double vy0 = 3000.0,
                                                                        double ay = 50.0) {
  std::vector<double> y(cycles), vy_vec(cycles);
  for (int i = 0; i < cycles; ++i) {
    const double t = static_cast<double>(i);
    y[i] = y0 + vy0 * t + 0.5 * ay * t * t;
    vy_vec[i] = vy0 + ay * t;
  }
  return {y, vy_vec};
}

/// @brief 末端机动：匀速飞行 → 突然横向偏移（模拟规避机动）。
std::pair<std::vector<double>, std::vector<double>> MakeTerminalManeuverScenario(
    int cycles, int maneuver_start = 30, double vy = 3000.0, double lateral_accel = 100.0) {
  std::vector<double> y(cycles), vy_vec(cycles);
  for (int i = 0; i < cycles; ++i) {
    const double t = static_cast<double>(i);
    if (i < maneuver_start) {
      y[i] = vy * t;
      vy_vec[i] = vy;
    } else {
      const double tm = t - static_cast<double>(maneuver_start);
      y[i] = vy * static_cast<double>(maneuver_start) + vy * tm + 0.5 * lateral_accel * tm * tm;
      vy_vec[i] = vy + lateral_accel * tm;
    }
  }
  return {y, vy_vec};
}

// ============================================================================
// 测试用例
// ============================================================================

TEST(SbirsImmEvaluationTest, EkfAndImmProduceFiniteStates) {
  const auto [truth_y, truth_vy] = MakeCvScenario(20);
  const EvalResult ekf = RunSingleEkf(truth_y, truth_vy, 0.5f);
  const EvalResult imm = RunImmEkf2(truth_y, truth_vy, 0.5f, 50.0f);

  ASSERT_EQ(ekf.position_errors.size(), 20U);
  ASSERT_EQ(imm.position_errors.size(), 20U);
  for (std::size_t i = 0; i < 20; ++i) {
    EXPECT_TRUE(std::isfinite(ekf.position_errors[i])) << "EKF error non-finite at " << i;
    EXPECT_TRUE(std::isfinite(imm.position_errors[i])) << "IMM error non-finite at " << i;
  }
}

TEST(SbirsImmEvaluationTest, ConstantVelocityBothSimilar) {
  // 中段匀速：IMM 不应明显差于单 EKF。
  const auto [truth_y, truth_vy] = MakeCvScenario(60);
  const EvalResult ekf = RunSingleEkf(truth_y, truth_vy, 0.5f);
  const EvalResult imm = RunImmEkf2(truth_y, truth_vy, 0.5f, 50.0f);

  const float ekf_rmse = PositionRmse(ekf.position_errors);
  const float imm_rmse = PositionRmse(imm.position_errors);

  EXPECT_NEAR(ekf_rmse, imm_rmse, std::max(ekf_rmse * 0.3f, 500.0f))
      << "EKF=" << ekf_rmse << " IMM=" << imm_rmse;
}

TEST(SbirsImmEvaluationTest, BoostPhaseImmOutperformsSingleEkf) {
  // 助推段加速度：CV 模型持续失配，IMM 应显著优于单 EKF。
  const auto [truth_y, truth_vy] = MakeBoostScenario(80);
  const EvalResult ekf = RunSingleEkf(truth_y, truth_vy, 1.0f);
  const EvalResult imm = RunImmEkf2(truth_y, truth_vy, 1.0f, 150.0f);

  const float ekf_rmse = PositionRmse(ekf.position_errors);
  const float imm_rmse = PositionRmse(imm.position_errors);

  EXPECT_LT(imm_rmse, ekf_rmse)
      << "IMM should outperform single EKF in boost phase: EKF=" << ekf_rmse << " IMM=" << imm_rmse;
}

TEST(SbirsImmEvaluationTest, TerminalManeuverImmOutperformsSingleEkf) {
  // 末端机动：突然横向加速，IMM 应更快适应。
  const auto [truth_y, truth_vy] = MakeTerminalManeuverScenario(60, 30);
  const EvalResult ekf = RunSingleEkf(truth_y, truth_vy, 1.0f);
  const EvalResult imm = RunImmEkf2(truth_y, truth_vy, 1.0f, 100.0f);

  // 机动阶段的 RMSE
  std::vector<float> ekf_mnv(ekf.position_errors.begin() + 30, ekf.position_errors.end());
  std::vector<float> imm_mnv(imm.position_errors.begin() + 30, imm.position_errors.end());
  const float ekf_mnv_rmse = PositionRmse(ekf_mnv);
  const float imm_mnv_rmse = PositionRmse(imm_mnv);

  EXPECT_LT(imm_mnv_rmse, ekf_mnv_rmse)
      << "IMM should outperform single EKF in terminal maneuver: "
      << "EKF=" << ekf_mnv_rmse << " IMM=" << imm_mnv_rmse;
}

TEST(SbirsImmEvaluationTest, ImprovementMagnitudeAcrossScenarios) {
  // 汇总三种场景的改善幅度。
  struct Scenario {
    const char* name;
    float single_rmse;
    float imm_rmse;
    float improvement_pct;
  };
  std::vector<Scenario> scenarios;

  // 场景 1：中段匀速
  {
    const auto [y, vy] = MakeCvScenario(80);
    auto ekf = RunSingleEkf(y, vy, 0.5f);
    auto imm = RunImmEkf2(y, vy, 0.5f, 50.0f);
    float sr = PositionRmse(ekf.position_errors);
    float ir = PositionRmse(imm.position_errors);
    scenarios.push_back({"Midcourse CV", sr, ir, (sr - ir) / std::max(sr, 1.0f) * 100.0f});
  }

  // 场景 2：助推段
  {
    const auto [y, vy] = MakeBoostScenario(100, 0.0, 3000.0, 60.0);
    auto ekf = RunSingleEkf(y, vy, 1.0f);
    auto imm = RunImmEkf2(y, vy, 1.0f, 150.0f);
    float sr = PositionRmse(ekf.position_errors);
    float ir = PositionRmse(imm.position_errors);
    scenarios.push_back({"Boost", sr, ir, (sr - ir) / std::max(sr, 1.0f) * 100.0f});
  }

  // 场景 3：末端机动
  {
    const auto [y, vy] = MakeTerminalManeuverScenario(60, 30);
    auto ekf = RunSingleEkf(y, vy, 1.0f);
    auto imm = RunImmEkf2(y, vy, 1.0f, 100.0f);
    float sr = PositionRmse(ekf.position_errors);
    float ir = PositionRmse(imm.position_errors);
    scenarios.push_back({"TerminalManeuver", sr, ir, (sr - ir) / std::max(sr, 1.0f) * 100.0f});
  }

  std::cout << "\n=== SBIRS IMM vs Single EKF Improvement Summary ===\n";
  for (const auto& sc : scenarios) {
    std::cout << "[" << sc.name << "] single_rmse=" << sc.single_rmse
              << " imm_rmse=" << sc.imm_rmse
              << " improvement=" << sc.improvement_pct << "%\n";
  }
  std::cout << "Threshold: <5%=noise, 5-15%=marginal, >15%=meaningful\n";

  for (const auto& sc : scenarios) {
    EXPECT_GT(sc.improvement_pct, -100.0f) << sc.name << " data invalid";
  }
}

}  // namespace
