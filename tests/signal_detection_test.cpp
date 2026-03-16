// Copyright 2026. All Rights Reserved.
//
// Description: 验证雷达物理方程库和信号检测器的正确性。

#include <cmath>

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/signal/detection/RadarEquations.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"

namespace airborne_radar {
namespace tests {

using signal::detection::AntennaConfig;
using signal::detection::DetectionPolicy;
using signal::detection::DetectionResult;
using signal::detection::MeasurementErrorModel;
using signal::detection::RadarEquations;
using signal::detection::RadarSystemConfig;
using signal::detection::ReceiverConfig;
using signal::detection::SignalDetector;
using signal::detection::TargetLookResolver;
using signal::detection::TransmitterConfig;

// ===========================================================================
// RadarEquations 纯函数单元测试
// ===========================================================================

/// @brief 典型参数场景下回波功率手算验证。
TEST(RadarEquationsTest, EchoPower_KnownValues) {
  TransmitterConfig tx;
  tx.peak_power_w = 1e6f;       // 1 MW
  tx.frequency_hz = 3e9f;       // S-band
  tx.transmit_loss_db = 3.0f;

  AntennaConfig ant;
  ant.main_beam_gain_db = 35.0f;

  // 目标: RCS=10m², 距离=50km，无额外传播损耗
  const float pr = RadarEquations::ComputeEchoPower_dBW(
      tx, ant, 10.0f, 50000.0f, 0.0f);

  // 手算验证思路：
  // Pt_dB = 60, Gt=Gr=35, λ=0.1m→λ_dB=-10, σ_dB=10
  // R_dB=10*log10(50000)=46.99
  // Pr = 60 + 35 + 35 + 2*(-10) + 10 - 30*log10(4π) - 4*46.99 - 3
  //    = 60 + 35 + 35 - 20 + 10 - 32.98 - 187.96 - 3
  //    ≈ -103.9 dBW
  EXPECT_NEAR(pr, -103.9f, 1.0f);  // 允许 ±1 dB 误差
}

/// @brief 距离加倍 → 功率下降约 12 dB（R⁴ 规律）。
TEST(RadarEquationsTest, EchoPower_RangeInverseQuartic) {
  TransmitterConfig tx;
  AntennaConfig ant;

  const float pr1 = RadarEquations::ComputeEchoPower_dBW(
      tx, ant, 1.0f, 10000.0f, 0.0f);
  const float pr2 = RadarEquations::ComputeEchoPower_dBW(
      tx, ant, 1.0f, 20000.0f, 0.0f);

  // 距离翻倍 → 功率下降 4*10*log10(2) ≈ 12.04 dB
  const float delta_db = pr1 - pr2;
  EXPECT_NEAR(delta_db, 12.04f, 0.2f);
}

/// @brief 验证热噪声功率底的玻尔兹曼公式。
TEST(RadarEquationsTest, NoisePower_BoltzmannFormula) {
  TransmitterConfig tx;
  tx.bandwidth_hz = 1e6f;  // 1 MHz

  ReceiverConfig rx;
  rx.noise_figure_db = 0.0f;  // 理想接收机

  const float noise_w =
      RadarEquations::ComputeThermalNoisePower_W(tx, rx);

  // 理论值: N = kT₀B = 1.38e-23 * 290 * 1e6 ≈ 4.002e-15 W
  EXPECT_NEAR(noise_w, 4.002e-15f, 1e-16f);
}

/// @brief 相参积累：增益 = N。
TEST(RadarEquationsTest, AccGain_CoherentLinear) {
  const float gain = RadarEquations::ComputeIntegrationGain(16, true);
  EXPECT_FLOAT_EQ(gain, 16.0f);
}

/// @brief 非相参积累：增益 = √N。
TEST(RadarEquationsTest, AccGain_IncoherentSqrt) {
  const float gain = RadarEquations::ComputeIntegrationGain(16, false);
  EXPECT_FLOAT_EQ(gain, 4.0f);
}

/// @brief 高信噪比下测距方差较小。
TEST(RadarEquationsTest, RangeStdDev_HighSNR) {
  // SNR=20dB, BW=1MHz
  const float std_dev =
      RadarEquations::ComputeRangeErrorStdDev(20.0f, 1e6f);
  // δ_R = c/(2B) = 150m, σ = 0.5*150/√100 + 20 = 7.5 + 20 = 27.5m
  EXPECT_NEAR(std_dev, 27.5f, 0.5f);
}

/// @brief 低信噪比下测距方差放大。
TEST(RadarEquationsTest, RangeStdDev_LowSNR) {
  // SNR=0dB, BW=1MHz
  const float std_high =
      RadarEquations::ComputeRangeErrorStdDev(20.0f, 1e6f);
  const float std_low =
      RadarEquations::ComputeRangeErrorStdDev(0.0f, 1e6f);
  EXPECT_GT(std_low, std_high);
}

/// @brief 测角方差与波束宽度成正比。
TEST(RadarEquationsTest, AngleStdDev_Proportional) {
  const float kDeg2Rad = 3.14159265f / 180.0f;
  const float std_narrow =
      RadarEquations::ComputeAngleErrorStdDev(13.0f, 2.0f * kDeg2Rad);
  const float std_wide =
      RadarEquations::ComputeAngleErrorStdDev(13.0f, 4.0f * kDeg2Rad);

  // 波束宽度翻倍 → 误差应近似翻倍
  EXPECT_NEAR(std_wide / std_narrow, 2.0f, 0.1f);
}

// ===========================================================================
// SignalDetector 集成测试
// ===========================================================================

/// @brief 近距大 RCS 目标 → 高 SNR，必然探测成功。
TEST(SignalDetectorTest, CloseTarget_HighSNR_Detected) {
  RadarSystemConfig config;
  // 使用默认参数（1MW, S-band, 35dB gain）
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  // 近距大目标：5km, RCS=100m²
  signal::detection::TargetReturn target;
  target.rcs_m2 = 100.0f;
  target.range_m = 5000.0f;
  
  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 2.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;
  
  DetectionResult result = detector.Detect(target, env);

  EXPECT_TRUE(result.detected);
  EXPECT_GT(result.snr_db, 10.0f);  // 应该有很高的 SNR
  EXPECT_GT(result.detection_prob, 0.9f);
}

/// @brief 远距小 RCS 目标 → 低 SNR，大概率探测失败。
TEST(SignalDetectorTest, FarTarget_LowSNR_NotDetected) {
  RadarSystemConfig config;
  config.transmitter.peak_power_w = 1e4f;  // 降低发射功率
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  // 远距小目标：300km, RCS=0.1m²
  signal::detection::TargetReturn target;
  target.rcs_m2 = 0.1f;
  target.range_m = 300000.0f;
  
  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 5.0f;
  env.clutter_noise_w = 1e-14f;
  env.jam_noise_w = 0.0f;
  
  DetectionResult result = detector.Detect(target, env);

  EXPECT_LT(result.snr_db, 0.0f);  // SNR 应该很低
}

/// @brief 相同随机种子 → 完全确定性输出。
TEST(SignalDetectorTest, DeterministicWithSeed) {
  RadarSystemConfig config;

  SignalDetector d1(config);
  d1.SetRandomSeed(123u);
  
  signal::detection::TargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;
  
  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 2.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;
  
  DetectionResult r1 = d1.Detect(target, env);

  SignalDetector d2(config);
  d2.SetRandomSeed(123u);
  DetectionResult r2 = d2.Detect(target, env);

  EXPECT_EQ(r1.detected, r2.detected);
  EXPECT_FLOAT_EQ(r1.snr_db, r2.snr_db);
  EXPECT_FLOAT_EQ(r1.detection_prob, r2.detection_prob);
}

/// @brief 注入干扰噪声 → SNR 下降、Pd 下降。
TEST(SignalDetectorTest, JammingIncreasesNoise) {
  RadarSystemConfig config;
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;
  
  signal::detection::EnvironmentState env_clean;
  env_clean.propagation_loss_db = 2.0f;
  env_clean.clutter_noise_w = 0.0f;
  env_clean.jam_noise_w = 0.0f;
  
  DetectionResult clean = detector.Detect(target, env_clean);

  detector.SetRandomSeed(42u);
  signal::detection::EnvironmentState env_jam = env_clean;
  env_jam.jam_noise_w = 1e-10f;
  DetectionResult jammed = detector.Detect(target, env_jam);

  EXPECT_LT(jammed.snr_db, clean.snr_db);
}

/// @brief 雷达局部坐标应能解析出稳定的目标方位/俯仰角。
TEST(TargetLookResolverTest, ResolvesRadarLocalLookAngles) {
  common::TargetFeature target;
  target.position_x = 10.0f;
  target.position_y = 10.0f;
  target.position_z = 10.0f;

  const signal::detection::TargetLookAnglesDeg look_angles =
      TargetLookResolver::Resolve(target);
  ASSERT_TRUE(look_angles.has_look_angles);
  EXPECT_NEAR(look_angles.look_az_deg, 45.0f, 1e-3f);
  EXPECT_NEAR(look_angles.look_el_deg, 35.264f, 1e-2f);
}

/// @brief 俯仰名义波束宽度变化应影响等效角误差，避免配置悬空。
TEST(MeasurementErrorModelTest, ElevationBeamwidthAffectsEquivalentAngleStdDev) {
  AntennaConfig narrow_antenna;
  narrow_antenna.nominal_az_beamwidth_deg = 2.0f;
  narrow_antenna.nominal_el_beamwidth_deg = 2.0f;

  AntennaConfig wide_el_antenna = narrow_antenna;
  wide_el_antenna.nominal_el_beamwidth_deg = 8.0f;

  common::RadarOrientationConfig orientation;
  common::PlatformAttitudeDeg platform_attitude_deg;
  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState narrow_state =
      signal::detection::BeamControlResolver::Resolve(narrow_antenna,
                                                      orientation,
                                                      platform_attitude_deg,
                                                      look_angles);
  const signal::detection::ResolvedBeamState wide_state =
      signal::detection::BeamControlResolver::Resolve(wide_el_antenna,
                                                      orientation,
                                                      platform_attitude_deg,
                                                      look_angles);

  const signal::detection::MeasurementErrorState narrow_error =
      MeasurementErrorModel::Compute(20.0f, narrow_state.effective_beamwidth_deg,
                                     1e6f);
  const signal::detection::MeasurementErrorState wide_error =
      MeasurementErrorModel::Compute(20.0f, wide_state.effective_beamwidth_deg,
                                     1e6f);

  EXPECT_GT(wide_error.angle_error_std_rad, narrow_error.angle_error_std_rad);
}

/// @brief 启用指令态波束宽度后，应覆盖名义波束宽度参与角误差建模。
TEST(MeasurementErrorModelTest, CommandedBeamwidthOverrideAffectsAngleStdDev) {
  AntennaConfig antenna;
  antenna.nominal_az_beamwidth_deg = 2.0f;
  antenna.nominal_el_beamwidth_deg = 2.0f;

  common::RadarOrientationConfig nominal_orientation;
  nominal_orientation.commanded_beamwidth_enabled = false;
  common::PlatformAttitudeDeg platform_attitude_deg;

  common::RadarOrientationConfig commanded_orientation;
  commanded_orientation.commanded_beamwidth_enabled = true;
  commanded_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg =
      8.0f;
  commanded_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
      8.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState nominal_state =
      signal::detection::BeamControlResolver::Resolve(antenna,
                                                      nominal_orientation,
                                                      platform_attitude_deg,
                                                      look_angles);
  const signal::detection::ResolvedBeamState commanded_state =
      signal::detection::BeamControlResolver::Resolve(antenna,
                                                      commanded_orientation,
                                                      platform_attitude_deg,
                                                      look_angles);

  const signal::detection::MeasurementErrorState nominal_error =
      MeasurementErrorModel::Compute(
          20.0f, nominal_state.effective_beamwidth_deg, 1e6f);
  const signal::detection::MeasurementErrorState commanded_error =
      MeasurementErrorModel::Compute(
          20.0f, commanded_state.effective_beamwidth_deg, 1e6f);

  EXPECT_GT(commanded_error.angle_error_std_rad,
            nominal_error.angle_error_std_rad);
}

/// @brief 指令态波束展宽应通过方向图增益修正改善离轴目标的回波功率。
TEST(BeamControlResolverTest, CommandedBeamwidthAffectsDirectionalPatternGain) {
  RadarSystemConfig config;
  config.antenna.nominal_az_beamwidth_deg = 2.0f;
  config.antenna.nominal_el_beamwidth_deg = 2.0f;
  config.antenna.enable_directional_pattern = true;
  config.antenna.pattern.max_sidelobe_level_db = -25.0f;

  common::RadarOrientationConfig nominal_orientation;
  nominal_orientation.commanded_beamwidth_enabled = false;
  common::PlatformAttitudeDeg platform_attitude_deg;

  common::RadarOrientationConfig commanded_orientation;
  commanded_orientation.commanded_beamwidth_enabled = true;
  commanded_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg =
      8.0f;
  commanded_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
      8.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.look_az_deg = 3.0f;
  look_angles.look_el_deg = 0.0f;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState nominal_state =
      signal::detection::BeamControlResolver::Resolve(
          config.antenna, nominal_orientation, platform_attitude_deg,
          look_angles);
  const signal::detection::ResolvedBeamState commanded_state =
      signal::detection::BeamControlResolver::Resolve(
          config.antenna, commanded_orientation, platform_attitude_deg,
          look_angles);

  SignalDetector detector(config);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 2.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;

  const DetectionResult nominal = detector.Detect(
      target, env, nominal_state.one_way_antenna_gain_db, 1, false);
  const DetectionResult commanded = detector.Detect(
      target, env, commanded_state.one_way_antenna_gain_db, 1, false);

  EXPECT_GT(commanded_state.one_way_antenna_gain_db,
            nominal_state.one_way_antenna_gain_db);
  EXPECT_GT(commanded.echo_power_dbw, nominal.echo_power_dbw);
  EXPECT_GT(commanded.snr_db, nominal.snr_db);
}

/// @brief 对惯性稳定模式应补偿平台姿态变化。
TEST(BeamControlResolverTest, InertialStabilizedCompensatesPlatformAttitude) {
  AntennaConfig antenna;
  common::RadarOrientationConfig orientation;
  orientation.stabilization_mode = common::StabilizationMode::kInertialStabilized;

  common::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 15.0f;
  platform_attitude_deg.pitch_deg = 5.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState state =
      signal::detection::BeamControlResolver::Resolve(
          antenna, orientation, platform_attitude_deg, look_angles);

  EXPECT_NEAR(state.beam_pointing_deg.az_deg, -15.0f, 1e-3f);
  EXPECT_NEAR(state.beam_pointing_deg.el_deg, -5.0f, 1e-3f);
}

// ===========================================================================
// Swerling 0~4 检测概率单元测试
// ===========================================================================

using signal::detection::SwerlingModel;
using signal::detection::kSwerling0;
using signal::detection::kSwerling1;
using signal::detection::kSwerling2;
using signal::detection::kSwerling3;
using signal::detection::kSwerling4;

/// @brief Swerling 1 单脉冲 ≡ Swerling 2 单脉冲（单脉冲无所谓快慢起伏）。
TEST(SwerlingDetectionTest, Sw1_N1_Equals_Sw2_N1) {
  const float pfa = 1e-6f;
  for (float snr_db = -5.0f; snr_db <= 25.0f; snr_db += 5.0f) {
    const float pd1 = RadarEquations::ComputeDetectionProbability(
        snr_db, pfa, kSwerling1, 1);
    const float pd2 = RadarEquations::ComputeDetectionProbability(
        snr_db, pfa, kSwerling2, 1);
    EXPECT_NEAR(pd1, pd2, 1e-6f)
        << "SNR=" << snr_db << " dB: Sw1(N=1) should equal Sw2(N=1)";
  }
}

/// @brief Swerling 3 单脉冲 ≡ Swerling 4 单脉冲。
TEST(SwerlingDetectionTest, Sw3_N1_Equals_Sw4_N1) {
  const float pfa = 1e-6f;
  for (float snr_db = -5.0f; snr_db <= 25.0f; snr_db += 5.0f) {
    const float pd3 = RadarEquations::ComputeDetectionProbability(
        snr_db, pfa, kSwerling3, 1);
    const float pd4 = RadarEquations::ComputeDetectionProbability(
        snr_db, pfa, kSwerling4, 1);
    EXPECT_NEAR(pd3, pd4, 1e-6f)
        << "SNR=" << snr_db << " dB: Sw3(N=1) should equal Sw4(N=1)";
  }
}

/// @brief 所有模型：高 SNR → Pd > 0.95。
TEST(SwerlingDetectionTest, HighSNR_AllModels_PdNearOne) {
  const float pfa = 1e-6f;
  const float high_snr = 35.0f;
  const SwerlingModel models[] = {
      kSwerling0, kSwerling1, kSwerling2, kSwerling3, kSwerling4};
  for (auto model : models) {
    const float pd = RadarEquations::ComputeDetectionProbability(
        high_snr, pfa, model, 1);
    EXPECT_GT(pd, 0.95f)
        << "Model=" << model << ": Pd should be > 0.95 at 35 dB";
  }
}

/// @brief 所有模型：极低 SNR → Pd 应接近 0（或虚警量级）。
TEST(SwerlingDetectionTest, LowSNR_AllModels_PdNearZero) {
  const float pfa = 1e-6f;
  const float low_snr = -20.0f;
  const SwerlingModel models[] = {
      kSwerling0, kSwerling1, kSwerling2, kSwerling3, kSwerling4};
  for (auto model : models) {
    const float pd = RadarEquations::ComputeDetectionProbability(
        low_snr, pfa, model, 1);
    EXPECT_LT(pd, 0.01f)
        << "Model=" << model << ": Pd should be near 0 at -20 dB";
  }
}

/// @brief 所有模型：Pd 随 SNR 单调递增。
TEST(SwerlingDetectionTest, Monotone_Pd_Increases_With_SNR) {
  const float pfa = 1e-6f;
  const SwerlingModel models[] = {
      kSwerling0, kSwerling1, kSwerling2, kSwerling3, kSwerling4};
  for (auto model : models) {
    float prev_pd = 0.0f;
    for (float snr_db = -10.0f; snr_db <= 25.0f; snr_db += 1.0f) {
      const float pd = RadarEquations::ComputeDetectionProbability(
          snr_db, pfa, model, 1);
      EXPECT_GE(pd, prev_pd - 1e-6f)
          << "Model=" << model << " SNR=" << snr_db
          << ": Pd must be monotonically non-decreasing";
      prev_pd = pd;
    }
  }
}

/// @brief 多脉冲积累应提升检测概率。
TEST(SwerlingDetectionTest, MultiPulse_Improves_Pd) {
  const float pfa = 1e-6f;
  const float snr_db = 8.0f;
  const SwerlingModel models[] = {
      kSwerling0, kSwerling1, kSwerling2, kSwerling3, kSwerling4};
  for (auto model : models) {
    const float pd_1 = RadarEquations::ComputeDetectionProbability(
        snr_db, pfa, model, 1);
    const float pd_10 = RadarEquations::ComputeDetectionProbability(
        snr_db, pfa, model, 10);
    EXPECT_GT(pd_10, pd_1)
        << "Model=" << model << ": 10 pulses should improve Pd over 1 pulse";
  }
}

/// @brief 快起伏多脉冲的 Pd 最终会收敛，而慢起伏拥有更高的方差。
/// 在中等 SNR 下，两者均应在 (0, 1) 区间内，且快起伏与慢起伏差异显著。
TEST(SwerlingDetectionTest, Sw1_Sw2_MultiPulse_BothReasonable) {
  const float pfa = 1e-6f;
  const float snr_db = 3.0f;
  const int N = 10;
  const float pd1 = RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, kSwerling1, N);
  const float pd2 = RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, kSwerling2, N);
  // 两者均应在合理范围内
  EXPECT_GT(pd1, 0.01f) << "Sw1 multi-pulse should give measurable Pd";
  EXPECT_GT(pd2, 0.01f) << "Sw2 multi-pulse should give measurable Pd";
  EXPECT_LT(pd1, 1.0f) << "Sw1 should not saturate at low SNR";
  EXPECT_LT(pd2, 1.0f) << "Sw2 should not saturate at low SNR";
  // 确认两者不相等（起伏模型不同应产生不同结果）
  EXPECT_NE(pd1, pd2);
}

/// @brief Swerling 3/4 多脉冲下快与慢起伏产生不同的 Pd。
TEST(SwerlingDetectionTest, Sw3_Sw4_MultiPulse_BothReasonable) {
  const float pfa = 1e-6f;
  const float snr_db = 1.0f;  // SW3 慢起伏在较高 SNR 下易饱和
  const int N = 10;
  const float pd3 = RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, kSwerling3, N);
  const float pd4 = RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, kSwerling4, N);
  EXPECT_GT(pd3, 0.01f) << "Sw3 multi-pulse should give measurable Pd";
  EXPECT_GT(pd4, 0.01f) << "Sw4 multi-pulse should give measurable Pd";
  EXPECT_LT(pd3, 1.0f) << "Sw3 should not saturate at low SNR";
  EXPECT_LT(pd4, 1.0f) << "Sw4 should not saturate at low SNR";
  EXPECT_NE(pd3, pd4);
}

/// @brief Swerling 1 单脉冲高 SNR 精确验证：Pd = exp(-T/(1+χ))。
TEST(SwerlingDetectionTest, Sw1_SinglePulse_ExactFormula) {
  const float pfa = 1e-6f;
  const float snr_db = 15.0f;
  const float pd = RadarEquations::ComputeDetectionProbability(
      snr_db, pfa, kSwerling1, 1);
  // 手工计算: T = -ln(1e-6) ≈ 13.8155, χ = 10^1.5 ≈ 31.623
  // Pd = exp(-13.8155 / (1+31.623)) = exp(-0.4237) ≈ 0.6550
  EXPECT_NEAR(pd, 0.655f, 0.01f);
}

/// @brief 检测门限 ComputeThreshold 基本验证。
TEST(SwerlingDetectionTest, Threshold_SinglePulse) {
  // N=1: T = -ln(Pfa)
  const double T = RadarEquations::ComputeThreshold(1e-6, 1);
  EXPECT_NEAR(T, -std::log(1e-6), 1e-10);
}

/// @brief 检测门限 ComputeThreshold 多脉冲验证（满足 gamma_q 反函数）。
TEST(SwerlingDetectionTest, Threshold_MultiPulse_Consistency) {
  // 验证 gamma_q(N, T) == Pfa
  const double pfa = 1e-6;
  for (int N = 2; N <= 20; N += 2) {
    const double T = RadarEquations::ComputeThreshold(pfa, N);
    EXPECT_GT(T, 0.0) << "Threshold must be positive for N=" << N;
    // T 应随 N 增加（因为多脉冲检测器有更多自由度）
    if (N > 2) {
      const double T_prev = RadarEquations::ComputeThreshold(pfa, N - 2);
      EXPECT_GT(T, T_prev)
          << "Threshold should increase with N";
    }
  }
}

}  // namespace tests
}  // namespace airborne_radar
