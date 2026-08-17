// Copyright 2026. All Rights Reserved.
//
// @file rir_signal_detector_test.cpp
// @brief 验证 RIR 统计级 CFAR 探测判决器（副本改写自 ar_signal_detection_test.cpp
//        的 SignalDetector 段；阶段 2-M M4：新增 Pfa 闭环与统计验证用例）。

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "remote_identification_radar/dwell/RirSignalDetector.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using dwell::RirDetectionResult;
using dwell::RirDetectorConfig;
using dwell::RirEnvironmentNoise;
using dwell::RirSignalDetector;
using dwell::RirTargetReturn;
using internal::RirRadarEquations;
using internal::RirSwerlingModel;

/// @brief 近距大 RCS 目标 → 高 SNR，必然探测成功。
TEST(RirSignalDetectorTest, CloseTarget_HighSNR_Detected) {
  RirDetectorConfig config;  // 默认参数（1MW, S-band, 35dB gain）
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;  // 近距大目标：5km, RCS=100m²
  target.rcs_m2 = 100.0f;
  target.range_m = 5000.0f;

  RirEnvironmentNoise env;
  env.propagation_loss_db = 2.0f;

  const RirDetectionResult result = detector.Detect(target, env);

  EXPECT_TRUE(result.detected);
  EXPECT_GT(result.snr_db, 10.0f);
  EXPECT_GT(result.detection_prob, 0.9f);
}

/// @brief 远距小 RCS 目标 → 低 SNR。
TEST(RirSignalDetectorTest, FarTarget_LowSNR_NotDetected) {
  RirDetectorConfig config;
  config.transmitter.peak_power_w = 1e4f;  // 降低发射功率
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;  // 远距小目标：300km, RCS=0.1m²
  target.rcs_m2 = 0.1f;
  target.range_m = 300000.0f;

  RirEnvironmentNoise env;
  env.propagation_loss_db = 5.0f;
  env.clutter_noise_w = 1e-14f;

  const RirDetectionResult result = detector.Detect(target, env);

  EXPECT_LT(result.snr_db, 0.0f);
}

/// @brief 相同随机种子 → 完全确定性输出（replay 语义）。
TEST(RirSignalDetectorTest, DeterministicWithSeed) {
  const RirDetectorConfig config;

  RirSignalDetector d1(config);
  d1.SetRandomSeed(123u);
  RirTargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;
  RirEnvironmentNoise env;
  env.propagation_loss_db = 2.0f;
  const RirDetectionResult r1 = d1.Detect(target, env);

  RirSignalDetector d2(config);
  d2.SetRandomSeed(123u);
  const RirDetectionResult r2 = d2.Detect(target, env);

  EXPECT_EQ(r1.detected, r2.detected);
  EXPECT_FLOAT_EQ(r1.snr_db, r2.snr_db);
  EXPECT_FLOAT_EQ(r1.detection_prob, r2.detection_prob);
}

/// @brief 注入干扰噪声 → SNR 下降。
TEST(RirSignalDetectorTest, JammingIncreasesNoise) {
  const RirDetectorConfig config;
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;

  RirEnvironmentNoise env_clean;
  env_clean.propagation_loss_db = 2.0f;
  const RirDetectionResult clean = detector.Detect(target, env_clean);

  detector.SetRandomSeed(42u);
  RirEnvironmentNoise env_jam = env_clean;
  env_jam.jam_noise_w = 1e-10f;
  const RirDetectionResult jammed = detector.Detect(target, env_jam);

  EXPECT_LT(jammed.snr_db, clean.snr_db);
}

/// @brief 同能量下带宽加倍 → 噪声底抬升 → SNR 下降（B 与 τ 的能量折算）。
TEST(RirSignalDetectorTest, WiderBandwidthReducesSnrWithSameEnergyInputs) {
  RirDetectorConfig narrow;
  narrow.transmitter.bandwidth_hz = 1.0e6f;
  narrow.transmitter.pulse_width_s = 13.0e-6f;
  RirSignalDetector detector_narrow(narrow);
  detector_narrow.SetRandomSeed(42u);

  RirDetectorConfig wide = narrow;
  wide.transmitter.bandwidth_hz = 4.0e6f;  // 带宽 ×4 → 噪声底 ×4
  RirSignalDetector detector_wide(wide);
  detector_wide.SetRandomSeed(42u);

  RirTargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;
  RirEnvironmentNoise env;
  env.propagation_loss_db = 2.0f;

  const RirDetectionResult r_narrow = detector_narrow.Detect(target, env);
  const RirDetectionResult r_wide = detector_wide.Detect(target, env);

  EXPECT_NEAR(r_narrow.snr_db - r_wide.snr_db, 6.0206f, 1.0e-3f);
}

/// @brief 同 SNR 下脉冲数 N=8 的 Pd 高于 N=1。
TEST(RirSignalDetectorTest, HigherPulseCountYieldsHigherPd) {
  const RirDetectorConfig config;
  RirSignalDetector detector(config);
  detector.SetRandomSeed(42u);

  RirTargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 120000.0f;
  RirEnvironmentNoise env;
  env.propagation_loss_db = 3.0f;
  env.clutter_noise_w = 1.0e-14f;

  const RirDetectionResult r_n1 = detector.Detect(target, env);
  detector.SetRandomSeed(42u);
  const RirDetectionResult r_n8 = detector.Detect(target, env);
  const float pd_n1 = internal::RirRadarEquations::ComputeDetectionProbability(
      r_n1.snr_db, config.detection_policy.cfar_pfa, target.swerling_type, 1);
  const float pd_n8 = internal::RirRadarEquations::ComputeDetectionProbability(
      r_n8.snr_db, config.detection_policy.cfar_pfa, target.swerling_type, 8);
  EXPECT_GT(pd_n8, pd_n1);
}

// ===========================================================================
// 统计级 CFAR 口径验证（《能力边界界定》§3.1 决策；v2 计划 §7.3）
// ===========================================================================

/// @brief Pfa 闭环：SNR → -∞ 时 Pd 收敛到配置的 Pfa（门限由 Pfa 反解）。
TEST(RirCfarTest, PdConvergesToPfaAtZeroSignal) {
  const float pfa = 1.0e-2f;
  const float pd = RirRadarEquations::ComputeDetectionProbability(
      -300.0f, pfa, RirSwerlingModel::kSwerling0, 1);
  EXPECT_NEAR(pd, pfa, 1.0e-3f);
}

/// @brief 蒙特卡洛判决的统计频率收敛到输入概率（100k 抽样，±20% 相对容差）。
TEST(RirCfarTest, ThresholdDecisionRateMatchesProbability) {
  const float prob = 1.0e-2f;
  const int trials = 100000;
  std::mt19937 rng(7u);
  int hits = 0;
  for (int i = 0; i < trials; ++i) {
    if (RirRadarEquations::ThresholdDecision(prob, rng)) {
      ++hits;
    }
  }
  const double rate = static_cast<double>(hits) / trials;
  EXPECT_NEAR(rate, prob, prob * 0.2);
}

/// @brief 虚警率统计：门限判决 P=0.5 时频率接近 0.5（对称抽样快速收敛）。
TEST(RirCfarTest, ThresholdDecisionFairCoin) {
  const int trials = 100000;
  std::mt19937 rng(11u);
  int hits = 0;
  for (int i = 0; i < trials; ++i) {
    if (RirRadarEquations::ThresholdDecision(0.5f, rng)) {
      ++hits;
    }
  }
  const double rate = static_cast<double>(hits) / trials;
  EXPECT_NEAR(rate, 0.5, 5.0e-3);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
