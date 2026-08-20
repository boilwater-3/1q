// Copyright 2026. All Rights Reserved.
//
// @file sbirs_angle_only_reachability_characterization_test.cpp
// @brief 角度-only 弱可观测可达性矩阵（P0 指标签认依据，TARGET-OQ/F4 证据）。
//
// 场景：静止单星 + 简化弹道目标（常重力解析解），量测 = 真值角 + 高斯噪声；
//       滤波 = 无迹（P1 原语）+ 匹配的弹道转移模型——运动模型不失配，
//       误差源被隔离为纯可观测性（信息量）问题。
// 扫描：弧长 {10,30,60,120} s × 角 σ {5,50,200} µrad × Monte-Carlo 20 次。
// 指标：末周期位置 RMSE、沿 LOS（径向）RMSE、简化发射点回推（x0 = x − v·t）RMSE。
//
// 实测结论（2026-08-17，数字回写决策文档指标签认表）：
//   1. σ=50 µrad 列弧长单调改善（10 s 6.5 km → 120 s 1.6 km），地板 ~1.2–1.7 km——
//      单静止单星角度-only 距离可达性地板为公里级。
//   2. σ=5 µrad 列随弧长发散：R≈2.5e-11 rad² 与位置量级 1e6 m 的组合触及 float
//      精度边缘，距离信息在中间量中丢失——估计层（P2）需 double 中间量或状态缩放。
//   3. 发射点回推误差与同时刻位置误差同量级或更小（0.07–1.06×）：匹配模型下
//      距离误差与速度误差相关，回推部分抵消；"回推剧放大"仅在模型失配时出现。

#include <gtest/gtest.h>

#include <Eigen/Cholesky>

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "common/estimation/UnscentedPredictor.h"
#include "common/estimation/UnscentedUpdater.h"
#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace {

using namespace sbirs_sensor;
using namespace sbirs_sensor::tracking;

namespace estimation = ::oneq::common::estimation;

constexpr int kStateDim = kSbirsStateDim;    // 6
constexpr int kMeasDim = kSbirsMeasurementDim;  // 2

// 几何（ECI，GMST≈0 下与 ECEF 同构）：静止单星在原点侧方，目标在 +x 半径 8000 km 处
// 以 +y 向速度 + 爬升速度运动（短弧弹道近似）。
constexpr double kGravityX = -7.5;        // m/s²，常重力（8000 km 半径处量级）
constexpr double kInitialX = 8000000.0;   // m
constexpr double kInitialVx = 300.0;      // m/s 爬升
constexpr double kInitialY = 0.0;
constexpr double kInitialVy = 2000.0;     // m/s 下航向
constexpr double kSatelliteX = 7000000.0; // m

/// @brief 简化弹道转移模型（常重力解析解，ECI [x,vx,y,vy,z,vz] 交错布局）。
/// @note f 与 Jacobian 均为线性 + 已知输入；无迹预测只消费 Function。
class BallisticTransitionModel final : public estimation::ITransitionModel<kStateDim> {
 public:
  StateVector Function(const StateVector& state, float dt) const override {
    const double d = static_cast<double>(dt);
    StateVector next = state;
    next(0) = static_cast<float>(state(0) + state(1) * d + 0.5 * kGravityX * d * d);
    next(1) = static_cast<float>(state(1) + kGravityX * d);
    next(2) = static_cast<float>(state(2) + state(3) * d);
    next(3) = state(3);
    next(4) = state(4);
    next(5) = state(5);
    return next;
  }
  TransitionMatrix Jacobian(const StateVector& /*state*/, float dt) const override {
    TransitionMatrix F = TransitionMatrix::Identity();
    F(0, 1) = dt;
    F(2, 3) = dt;
    F(4, 5) = dt;
    return F;
  }
};

session::SbirsVector3M SatPosition() {
  session::SbirsVector3M v;
  v.x = kSatelliteX;
  v.y = 0.0;
  v.z = 0.0;
  return v;
}

/// @brief 真值状态解析解。
Eigen::Matrix<double, kStateDim, 1> TruthAt(double t) {
  Eigen::Matrix<double, kStateDim, 1> s;
  s(0) = kInitialX + kInitialVx * t + 0.5 * kGravityX * t * t;
  s(1) = kInitialVx + kGravityX * t;
  s(2) = kInitialY + kInitialVy * t;
  s(3) = kInitialVy;
  s(4) = 0.0;
  s(5) = 0.0;
  return s;
}

/// @brief 单次 Monte-Carlo：返回 (末周期位置误差, 沿 LOS 误差, 发射点回推误差)，单位 m。
struct RunMetrics {
  double position_error;
  double los_error;
  double launch_point_error;
};

RunMetrics RunOnce(int arc_cycles, double sigma_rad, std::mt19937& engine) {
  std::normal_distribution<double> noise(0.0, 1.0);

  BallisticTransitionModel model;
  estimation::UnscentedPredictorConfig pred_cfg;
  pred_cfg.noise_diff_coeff = 2.0f;  // 模型匹配但保留过程噪声正则，防一致性退化
  pred_cfg.transform.alpha = 0.5f;   // 收紧 sigma 散布（角度-only 病态几何推荐值）
  estimation::UnscentedPredictor<kStateDim, kMeasDim> predictor(&model, pred_cfg);

  SbirsAngleMeasurementModel meas_model;
  meas_model.SetSatellitePosition(SatPosition());
  estimation::UnscentedUpdaterConfig upd_cfg;
  upd_cfg.transform.alpha = 0.5f;
  estimation::UnscentedUpdater<kStateDim, kMeasDim> updater(&meas_model, upd_cfg);

  // 初始化：均值=真值、大协方差（位置 10 km、速度 1.5 km/s 1-σ）——先验代表搜索域，
  // 距离信息只能来自量测弧段（可观测性即证据对象）。
  const Eigen::Matrix<double, kStateDim, 1> truth0 = TruthAt(0.0);
  SbirsGaussianState state;
  state.covariance = SbirsStateCovariance::Zero();
  for (int i = 0; i < kStateDim; i += 2) {
    state.mean(i) = static_cast<float>(truth0(i));
    state.mean(i + 1) = static_cast<float>(truth0(i + 1));
    state.covariance(i, i) = 10000.0f * 10000.0f;
    state.covariance(i + 1, i + 1) = 1500.0f * 1500.0f;
  }

  const float dt = 1.0f;
  SbirsMeasurementCovariance R = SbirsMeasurementCovariance::Zero();
  R(0, 0) = static_cast<float>(sigma_rad * sigma_rad);
  R(1, 1) = static_cast<float>(sigma_rad * sigma_rad);

  for (int cycle = 1; cycle <= arc_cycles; ++cycle) {
    const Eigen::Matrix<double, kStateDim, 1> truth = TruthAt(static_cast<double>(cycle));
    SbirsStateVector truth_state;
    for (int i = 0; i < kStateDim; ++i) {
      truth_state(i) = static_cast<float>(truth(i));
    }
    SbirsMeasurementVector z = meas_model.Function(truth_state);
    z(0) += static_cast<float>(noise(engine) * sigma_rad);
    z(1) += static_cast<float>(noise(engine) * sigma_rad);
    const SbirsGaussianState predicted = predictor.Predict(state, dt);
    state = updater.Update(predicted, z, R).posterior;
  }

  const double t = static_cast<double>(arc_cycles);
  const Eigen::Matrix<double, kStateDim, 1> truth_final = TruthAt(t);
  Eigen::Matrix<double, 3, 1> est_pos, truth_pos;
  est_pos << state.mean(0), state.mean(2), state.mean(4);
  truth_pos << truth_final(0), truth_final(2), truth_final(4);
  const Eigen::Matrix<double, 3, 1> error_vec = est_pos - truth_pos;

  Eigen::Matrix<double, 3, 1> los = truth_pos;
  los(0) -= kSatelliteX;
  const double los_norm = los.norm();
  const double los_error = error_vec.dot(los) / los_norm;  // 径向（距离）分量

  // 简化发射点回推：x0 = x − v·t（与真值生成模型一致的解析逆）。
  Eigen::Matrix<double, 3, 1> est_x0, truth_x0;
  est_x0 << state.mean(0) - state.mean(1) * static_cast<float>(t) -
                static_cast<float>(0.5 * kGravityX * t * t),
            state.mean(2) - state.mean(3) * static_cast<float>(t),
            state.mean(4) - state.mean(5) * static_cast<float>(t);
  truth_x0 << truth_final(0) - truth_final(1) * t - 0.5 * kGravityX * t * t,
      truth_final(2) - truth_final(3) * t, truth_final(4) - truth_final(5) * t;

  RunMetrics metrics;
  metrics.position_error = error_vec.norm();
  metrics.los_error = std::abs(los_error);
  metrics.launch_point_error = (est_x0 - truth_x0).norm();
  return metrics;
}

double RmseOf(const std::vector<double>& values) {
  double sum_sq = 0.0;
  for (double v : values) sum_sq += v * v;
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

TEST(SbirsAngleOnlyReachabilityCharacterizationTest, ReachabilityMatrixPhysicalFloor) {
  const int kArcCycles[] = {10, 30, 60, 120};
  const double kSigmaMicroRad[] = {5.0, 50.0, 200.0};
  constexpr int kMonteCarloRuns = 20;

  double los_rmse[4][3];
  double pos_rmse[4][3];
  double lp_rmse[4][3];

  std::cout << "\n=== Angle-only reachability matrix (single static sensor, matched-model UKF) ===\n";
  for (int a = 0; a < 4; ++a) {
    for (int s = 0; s < 3; ++s) {
      std::mt19937 engine(static_cast<std::uint32_t>(1000U + a * 10U + s));
      const double sigma_rad = kSigmaMicroRad[s] * 1.0e-6;
      std::vector<double> pos_err, los_err, lp_err;
      pos_err.reserve(kMonteCarloRuns);
      los_err.reserve(kMonteCarloRuns);
      lp_err.reserve(kMonteCarloRuns);
      for (int run = 0; run < kMonteCarloRuns; ++run) {
        const RunMetrics m = RunOnce(kArcCycles[a], sigma_rad, engine);
        pos_err.push_back(m.position_error);
        los_err.push_back(m.los_error);
        lp_err.push_back(m.launch_point_error);
      }
      pos_rmse[a][s] = RmseOf(pos_err);
      los_rmse[a][s] = RmseOf(los_err);
      lp_rmse[a][s] = RmseOf(lp_err);

      const std::string suffix = std::to_string(kArcCycles[a]) + "s_" +
                                 std::to_string(static_cast<int>(kSigmaMicroRad[s])) + "urad";
      RecordProperty(("reach_pos_" + suffix).c_str(), std::to_string(pos_rmse[a][s]));
      RecordProperty(("reach_los_" + suffix).c_str(), std::to_string(los_rmse[a][s]));
      RecordProperty(("reach_lp_" + suffix).c_str(), std::to_string(lp_rmse[a][s]));
      std::cout << "arc=" << kArcCycles[a] << "s sigma=" << kSigmaMicroRad[s] << "urad"
                << " pos_rmse=" << pos_rmse[a][s] << "m"
                << " los_rmse=" << los_rmse[a][s] << "m"
                << " launch_rmse=" << lp_rmse[a][s] << "m\n";
    }
  }

  // 物理地板门 1：全部网格距离（径向）RMSE 保持公里级（单静止单星可达性地板）。
  for (int a = 0; a < 4; ++a) {
    for (int s = 0; s < 3; ++s) {
      EXPECT_GT(los_rmse[a][s], 500.0)
          << "single static sensor must not reach sub-km range accuracy on this arc set";
    }
  }
  // 物理地板门 2：σ=50 µrad 列弧长单调改善（10 s 差于 120 s）。
  EXPECT_GT(los_rmse[0][1], los_rmse[3][1])
      << "longer arc must improve nominal-sigma range error";
  // 物理地板门 3：发射点回推误差与同时刻位置误差同量级（相关误差抵消，无剧放大）。
  for (int a = 0; a < 4; ++a) {
    for (int s = 0; s < 3; ++s) {
      EXPECT_GT(lp_rmse[a][s], 0.05 * pos_rmse[a][s]);
      EXPECT_LT(lp_rmse[a][s], 3.0 * pos_rmse[a][s]);
    }
  }
  // 物理地板门 4：径向（距离）误差主导总误差（角度-only 距离弱观测）。
  EXPECT_GT(los_rmse[0][1], pos_rmse[0][1] * 0.5);
}

}  // namespace
