// Copyright 2026. All Rights Reserved.
//
// @file rir_radar_equations_test.cpp
// @brief 验证 RIR 链路预算与检测物理纯函数（副本改写自 ar_signal_detection_test.cpp
//        的 RadarEquations/Swerling 段；阶段 2-M M1）。

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::hardware::RirAntennaConfig;
using config::hardware::RirReceiverConfig;
using config::hardware::RirTransmitterConfig;
using internal::RirRadarEquations;
using internal::RirSwerlingModel;

// ===========================================================================
// RirRadarEquations 纯函数单元测试
// ===========================================================================

/// @brief 典型参数场景下回波功率手算验证。
TEST(RirRadarEquationsTest, EchoPower_KnownValues) {
  RirTransmitterConfig tx;
  tx.peak_power_w = 1e6f;  // 1 MW
  tx.frequency_hz = 3e9f;  // S-band
  tx.transmit_loss_db = 3.0f;

  RirAntennaConfig ant;
  ant.main_beam_gain_db = 35.0f;

  // 目标: RCS=10m², 距离=50km，无额外传播损耗
  const float pr = RirRadarEquations::ComputeEchoPower_dBW(tx, ant, 10.0f, 50000.0f, 0.0f);

  // 手算验证思路：
  // Pt_dB = 60, Gt=Gr=35, λ=0.1m→λ_dB=-10, σ_dB=10
  // R_dB=10*log10(50000)=46.99
  // Pr = 60 + 35 + 35 + 2*(-10) + 10 - 30*log10(4π) - 4*46.99 - 3
  //    ≈ -103.9 dBW
  EXPECT_NEAR(pr, -103.9f, 1.0f);  // 允许 ±1 dB 误差
}

/// @brief 距离加倍 → 功率下降约 12 dB（R⁴ 规律）。
TEST(RirRadarEquationsTest, EchoPower_RangeInverseQuartic) {
  RirTransmitterConfig tx;
  tx.peak_power_w = 1e6f;
  tx.frequency_hz = 3e9f;

  RirAntennaConfig ant;
  ant.main_beam_gain_db = 35.0f;

  const float near_dbw = RirRadarEquations::ComputeEchoPower_dBW(tx, ant, 10.0f, 50000.0f, 0.0f);
  const float far_dbw = RirRadarEquations::ComputeEchoPower_dBW(tx, ant, 10.0f, 100000.0f, 0.0f);

  EXPECT_NEAR(near_dbw - far_dbw, 12.0412f, 0.1f);
}

/// @brief 热噪声底 N₀ = k·T₀·B·F 手算验证。
TEST(RirRadarEquationsTest, NoisePower_BoltzmannFormula) {
  RirTransmitterConfig tx;
  tx.bandwidth_hz = 1e6f;  // 1 MHz

  RirReceiverConfig rx;
  rx.noise_figure_db = 3.0f;  // F ≈ 2

  const float noise_w = RirRadarEquations::ComputeThermalNoisePower_W(tx, rx);
  // N = 1.38e-23 * 290 * 1e6 * 2 ≈ 8.0e-15 W
  EXPECT_NEAR(noise_w, 8.0e-15f, 1e-16f);
}

/// @brief 判决逻辑应先将 Pd 钳位到 [0,1]，越界输入与边界输入等价。
TEST(RirRadarEquationsTest, ThresholdDecision_ClampsOutOfRangePd) {
  std::mt19937 rng_negative(2026u);
  std::mt19937 rng_zero(2026u);
  std::mt19937 rng_over_one(2027u);
  std::mt19937 rng_one(2027u);

  for (int i = 0; i < 512; ++i) {
    EXPECT_EQ(RirRadarEquations::ThresholdDecision(-0.5f, rng_negative),
              RirRadarEquations::ThresholdDecision(0.0f, rng_zero));
    EXPECT_EQ(RirRadarEquations::ThresholdDecision(1.5f, rng_over_one),
              RirRadarEquations::ThresholdDecision(1.0f, rng_one));
  }
}

/// @brief 高信噪比下测距方差较小。
TEST(RirRadarEquationsTest, RangeStdDev_HighSNR) {
  // SNR=20dB, BW=1MHz
  const float std_dev = RirRadarEquations::ComputeRangeErrorStdDev(20.0f, 1e6f);
  // δ_R = c/(2B) = 150m, σ = 0.5*150/√100 = 7.5m（纯随机项；20m 偏置已拆至均值侧）
  EXPECT_NEAR(std_dev, 7.5f, 0.5f);
}

/// @brief 低信噪比下测距方差放大。
TEST(RirRadarEquationsTest, RangeStdDev_LowSNR) {
  // SNR=0dB, BW=1MHz
  const float std_high = RirRadarEquations::ComputeRangeErrorStdDev(20.0f, 1e6f);
  const float std_low = RirRadarEquations::ComputeRangeErrorStdDev(0.0f, 1e6f);
  EXPECT_GT(std_low, std_high);
}

/// @brief 测角方差与波束宽度成正比。
TEST(RirRadarEquationsTest, AngleStdDev_Proportional) {
  const float kDeg2Rad = 3.14159265f / 180.0f;
  const float std_narrow =
      RirRadarEquations::ComputeAngleErrorStdDev(13.0f, 2.0f * kDeg2Rad);
  const float std_wide = RirRadarEquations::ComputeAngleErrorStdDev(13.0f, 4.0f * kDeg2Rad);

  // 波束宽度翻倍 → 误差应近似翻倍
  EXPECT_NEAR(std_wide / std_narrow, 2.0f, 0.1f);
}

/// @brief N=1 门限闭式解 T = -ln(P_fa)。
TEST(RirRadarEquationsTest, Threshold_N1_ClosedForm) {
  const double pfa = 1e-6;
  const double threshold = RirRadarEquations::ComputeThreshold(pfa, 1);
  EXPECT_NEAR(threshold, -std::log(pfa), 1e-9);
}

/// @brief Marcum Q 边界：b=0 → 1；a >> b → 1（渐进钳位）。
TEST(RirRadarEquationsTest, MarcumQ_Boundaries) {
  EXPECT_DOUBLE_EQ(RirRadarEquations::MarcumQ(1, 0.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(RirRadarEquations::MarcumQ(4, 100.0, 5.0), 1.0);
  // a=0（纯噪声）时 Q_M(0, b) = exp(-b²/2)
  const double b = std::sqrt(2.0 * 13.8155);  // T ≈ -ln(1e-6)
  EXPECT_NEAR(RirRadarEquations::MarcumQ(1, 0.0, b), std::exp(-b * b / 2.0), 1e-9);
}

// ===========================================================================
// Swerling 检测概率测试
// ===========================================================================

/// @brief Swerling 1 单脉冲 ≡ Swerling 2 单脉冲（单脉冲无所谓快慢起伏）。
TEST(RirSwerlingDetectionTest, Sw1_N1_Equals_Sw2_N1) {
  const float pfa = 1e-6f;
  for (float snr_db = -5.0f; snr_db <= 25.0f; snr_db += 5.0f) {
    const float pd1 = RirRadarEquations::ComputeDetectionProbability(snr_db, pfa,
                                                     RirSwerlingModel::kSwerling1, 1);
    const float pd2 = RirRadarEquations::ComputeDetectionProbability(snr_db, pfa,
                                                     RirSwerlingModel::kSwerling2, 1);
    EXPECT_NEAR(pd1, pd2, 1e-6f) << "SNR=" << snr_db << " dB: Sw1(N=1) should equal Sw2(N=1)";
  }
}

/// @brief Swerling 3 单脉冲 ≡ Swerling 4 单脉冲。
TEST(RirSwerlingDetectionTest, Sw3_N1_Equals_Sw4_N1) {
  const float pfa = 1e-6f;
  for (float snr_db = -5.0f; snr_db <= 25.0f; snr_db += 5.0f) {
    const float pd3 = RirRadarEquations::ComputeDetectionProbability(snr_db, pfa,
                                                     RirSwerlingModel::kSwerling3, 1);
    const float pd4 = RirRadarEquations::ComputeDetectionProbability(snr_db, pfa,
                                                     RirSwerlingModel::kSwerling4, 1);
    EXPECT_NEAR(pd3, pd4, 1e-6f) << "SNR=" << snr_db << " dB: Sw3(N=1) should equal Sw4(N=1)";
  }
}

/// @brief 所有模型：高 SNR → Pd > 0.95。
TEST(RirSwerlingDetectionTest, HighSNR_AllModels_PdNearOne) {
  const float pfa = 1e-6f;
  const float high_snr = 35.0f;
  const RirSwerlingModel models[] = {RirSwerlingModel::kSwerling0, RirSwerlingModel::kSwerling1,
                                     RirSwerlingModel::kSwerling2, RirSwerlingModel::kSwerling3,
                                     RirSwerlingModel::kSwerling4};
  for (auto model : models) {
    const float pd = RirRadarEquations::ComputeDetectionProbability(high_snr, pfa, model, 1);
    EXPECT_GT(pd, 0.95f) << "Model=" << static_cast<int>(model) << ": Pd should be > 0.95 at 35 dB";
  }
}

/// @brief 所有模型：极低 SNR → Pd 应接近 0（或虚警量级）。
TEST(RirSwerlingDetectionTest, LowSNR_AllModels_PdNearZero) {
  const float pfa = 1e-6f;
  const float low_snr = -20.0f;
  const RirSwerlingModel models[] = {RirSwerlingModel::kSwerling0, RirSwerlingModel::kSwerling1,
                                     RirSwerlingModel::kSwerling2, RirSwerlingModel::kSwerling3,
                                     RirSwerlingModel::kSwerling4};
  for (auto model : models) {
    const float pd = RirRadarEquations::ComputeDetectionProbability(low_snr, pfa, model, 1);
    EXPECT_LT(pd, 0.01f) << "Model=" << static_cast<int>(model) << ": Pd should be near 0 at -20 dB";
  }
}

/// @brief 所有模型：Pd 随 SNR 单调递增。
TEST(RirSwerlingDetectionTest, Monotone_Pd_Increases_With_SNR) {
  const float pfa = 1e-6f;
  const RirSwerlingModel models[] = {RirSwerlingModel::kSwerling0, RirSwerlingModel::kSwerling1,
                                     RirSwerlingModel::kSwerling2, RirSwerlingModel::kSwerling3,
                                     RirSwerlingModel::kSwerling4};
  for (auto model : models) {
    float prev_pd = 0.0f;
    for (float snr_db = -10.0f; snr_db <= 25.0f; snr_db += 1.0f) {
      const float pd = RirRadarEquations::ComputeDetectionProbability(snr_db, pfa, model, 1);
      EXPECT_GE(pd, prev_pd - 1e-6f)
          << "Model=" << static_cast<int>(model) << " SNR=" << snr_db
          << ": Pd must be monotonically non-decreasing";
      prev_pd = pd;
    }
  }
}

/// @brief 多脉冲积累：同 SNR 下 N=8 的 Pd 高于 N=1。
TEST(RirSwerlingDetectionTest, HigherPulseCountYieldsHigherPd) {
  const float pfa = 1e-6f;
  const float snr_db = 5.0f;
  const RirSwerlingModel models[] = {RirSwerlingModel::kSwerling0, RirSwerlingModel::kSwerling1,
                                     RirSwerlingModel::kSwerling2, RirSwerlingModel::kSwerling3,
                                     RirSwerlingModel::kSwerling4};
  for (auto model : models) {
    const float pd_n1 = RirRadarEquations::ComputeDetectionProbability(snr_db, pfa, model, 1);
    const float pd_n8 = RirRadarEquations::ComputeDetectionProbability(snr_db, pfa, model, 8);
    EXPECT_GT(pd_n8, pd_n1)
        << "Model=" << static_cast<int>(model) << ": N=8 should beat N=1 at 5 dB";
  }
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
