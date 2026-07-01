// Copyright 2026. All Rights Reserved.
//
// @file signal_detection_test.cpp
// @brief 验证雷达物理方程库和信号检测器的正确性。

#include <gtest/gtest.h>

#include <boost/math/special_functions/gamma.hpp>
#include <cmath>
#include <limits>

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/RadarEquations.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"
#include "common/timing/TimingRegimeModel.h"

namespace airborne_radar {
namespace tests {

using config::engineering::AntennaConfig;
using config::engineering::DetectionPolicy;
using signal::detection::DetectionResult;
using signal::detection::MeasurementErrorModel;
using signal::detection::RadarEquations;
using config::engineering::DetectionConfig;
using config::engineering::ReceiverConfig;
using signal::detection::SignalDetector;
using signal::detection::TargetLookResolver;
using config::engineering::TransmitterConfig;

// ===========================================================================
// RadarEquations 纯函数单元测试
// ===========================================================================

/// @brief 典型参数场景下回波功率手算验证。
TEST(RadarEquationsTest, EchoPower_KnownValues) {
  TransmitterConfig tx;
  tx.peak_power_w = 1e6f;  // 1 MW
  tx.frequency_hz = 3e9f;  // S-band
  tx.transmit_loss_db = 3.0f;

  AntennaConfig ant;
  ant.main_beam_gain_db = 35.0f;

  // 目标: RCS=10m², 距离=50km，无额外传播损耗
  const float pr = RadarEquations::ComputeEchoPower_dBW(tx, ant, 10.0f, 50000.0f, 0.0f);

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

  const float pr1 = RadarEquations::ComputeEchoPower_dBW(tx, ant, 1.0f, 10000.0f, 0.0f);
  const float pr2 = RadarEquations::ComputeEchoPower_dBW(tx, ant, 1.0f, 20000.0f, 0.0f);

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

  const float noise_w = RadarEquations::ComputeThermalNoisePower_W(tx, rx);

  // 理论值: N = kT₀B = 1.38e-23 * 290 * 1e6 ≈ 4.002e-15 W
  EXPECT_NEAR(noise_w, 4.002e-15f, 1e-16f);
}

/// @brief 单脉冲能量语义下，脉宽翻倍应带来约 3 dB 回波功率提升。
TEST(RadarEquationsTest, EchoPowerIncreasesWithPulseWidthByEnergyScale) {
  TransmitterConfig tx;
  tx.peak_power_w = 1e6f;
  tx.frequency_hz = 3e9f;
  tx.pulse_width_s = 13.0e-6f;
  AntennaConfig ant;
  ant.main_beam_gain_db = 35.0f;

  const float baseline_dbw =
      RadarEquations::ComputeEchoPower_dBW(tx, ant, 5.0f, 30000.0f, 0.0f);

  tx.pulse_width_s = 26.0e-6f;
  const float widened_dbw =
      RadarEquations::ComputeEchoPower_dBW(tx, ant, 5.0f, 30000.0f, 0.0f);

  EXPECT_NEAR(widened_dbw - baseline_dbw, 3.0103f, 0.1f);
}

/// @brief 当前统一采用线性脉冲积累：增益 = N。
TEST(RadarEquationsTest, AccGain_PulseCountLinear) {
  const float gain = RadarEquations::ComputeIntegrationGain(16);
  EXPECT_FLOAT_EQ(gain, 16.0f);
}

/// @brief 判决逻辑应先将 Pd 钳位到 [0,1]，越界输入与边界输入等价。
TEST(RadarEquationsTest, ThresholdDecision_ClampsOutOfRangePd) {
  std::mt19937 rng_negative(2026u);
  std::mt19937 rng_zero(2026u);
  std::mt19937 rng_over_one(2027u);
  std::mt19937 rng_one(2027u);

  for (int i = 0; i < 512; ++i) {
    EXPECT_EQ(RadarEquations::ThresholdDecision(-0.5f, rng_negative),
              RadarEquations::ThresholdDecision(0.0f, rng_zero));
    EXPECT_EQ(RadarEquations::ThresholdDecision(1.5f, rng_over_one),
              RadarEquations::ThresholdDecision(1.0f, rng_one));
  }
}

/// @brief 高信噪比下测距方差较小。
TEST(RadarEquationsTest, RangeStdDev_HighSNR) {
  // SNR=20dB, BW=1MHz
  const float std_dev = RadarEquations::ComputeRangeErrorStdDev(20.0f, 1e6f);
  // δ_R = c/(2B) = 150m, σ = 0.5*150/√100 + 20 = 7.5 + 20 = 27.5m
  EXPECT_NEAR(std_dev, 27.5f, 0.5f);
}

/// @brief 低信噪比下测距方差放大。
TEST(RadarEquationsTest, RangeStdDev_LowSNR) {
  // SNR=0dB, BW=1MHz
  const float std_high = RadarEquations::ComputeRangeErrorStdDev(20.0f, 1e6f);
  const float std_low = RadarEquations::ComputeRangeErrorStdDev(0.0f, 1e6f);
  EXPECT_GT(std_low, std_high);
}

/// @brief 测角方差与波束宽度成正比。
TEST(RadarEquationsTest, AngleStdDev_Proportional) {
  const float kDeg2Rad = 3.14159265f / 180.0f;
  const float std_narrow = RadarEquations::ComputeAngleErrorStdDev(13.0f, 2.0f * kDeg2Rad);
  const float std_wide = RadarEquations::ComputeAngleErrorStdDev(13.0f, 4.0f * kDeg2Rad);

  // 波束宽度翻倍 → 误差应近似翻倍
  EXPECT_NEAR(std_wide / std_narrow, 2.0f, 0.1f);
}

// ===========================================================================
// SignalDetector 集成测试
// ===========================================================================

/// @brief 近距大 RCS 目标 → 高 SNR，必然探测成功。
TEST(SignalDetectorTest, CloseTarget_HighSNR_Detected) {
  DetectionConfig config;
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
  DetectionConfig config;
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
  DetectionConfig config;

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

/// @brief 共享周期级时序状态解析后的脉冲数应继续提高检测概率。
TEST(SignalDetectorTest, SharedTimingStatePulseCountStillImprovesDetectionProbability) {
  DetectionConfig config;
  config.detection_policy.cfar_pfa = 1.0e-6f;
  config.detection_policy.min_snr_db = -50.0f;
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 1.0f;
  target.range_m = 120000.0f;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 3.0f;
  env.clutter_noise_w = 1.0e-14f;
  env.jam_noise_w = 0.0f;

  oneq::internal::timing::CycleTimingBaseParams base_params;
  base_params.base_pulse_count = 8;
  base_params.base_prf_hz = config.transmitter.prf_hz;

  oneq::internal::timing::CycleTimingControlAdjustments baseline_adjustments;
  const oneq::internal::timing::ResolvedCycleTimingState baseline_state =
      oneq::internal::timing::ResolveCycleTimingState(base_params, baseline_adjustments);

  oneq::internal::timing::CycleTimingControlAdjustments dwell_adjustments;
  dwell_adjustments.dwell_scale = 2.0f;
  const oneq::internal::timing::ResolvedCycleTimingState dwell_state =
      oneq::internal::timing::ResolveCycleTimingState(base_params, dwell_adjustments);

  const DetectionResult baseline_result =
      detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(),
                      static_cast<int>(baseline_state.effective_pulse_count));
  detector.SetRandomSeed(42u);
  const DetectionResult dwell_result =
      detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(),
                      static_cast<int>(dwell_state.effective_pulse_count));

  EXPECT_EQ(baseline_state.effective_pulse_count, 8U);
  EXPECT_EQ(dwell_state.effective_pulse_count, 16U);
  EXPECT_GT(dwell_result.detection_prob, baseline_result.detection_prob);
}

/// @brief 注入干扰噪声 → SNR 下降、Pd 下降。
TEST(SignalDetectorTest, JammingIncreasesNoise) {
  DetectionConfig config;
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

/// @brief Detect 应直接使用“每脉冲 SNR + N”语义计算 Pd。
TEST(SignalDetectorTest, DetectionProbabilityUsesIntegratedSnr) {
  DetectionConfig config;
  config.detection_policy.min_snr_db = -80.0f;  // 关闭硬门限影响
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 0.5f;
  target.range_m = 120000.0f;
  target.swerling_type = config::profiles::SwerlingModel::kSwerling2;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 2e-14f;
  env.jam_noise_w = 0.0f;

  const DetectionResult n1 =
      detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(), 1);
  const DetectionResult n8 =
      detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(), 8);

  const float expected_pd_n1 = RadarEquations::ComputeDetectionProbability(
      n1.snr_db, config.detection_policy.cfar_pfa, target.swerling_type, 1);
  const float expected_pd_n8 = RadarEquations::ComputeDetectionProbability(
      n8.snr_db, config.detection_policy.cfar_pfa, target.swerling_type, 8);

  EXPECT_NEAR(n1.detection_prob, expected_pd_n1, 1e-6f);
  EXPECT_NEAR(n8.detection_prob, expected_pd_n8, 1e-6f);
  EXPECT_GT(n8.detection_prob, n1.detection_prob);
}

/// @brief 带宽越大热噪声越大，保持其他参数不变时单脉冲 SNR 应下降。
TEST(SignalDetectorTest, WiderBandwidthReducesSnrWithSameEnergyInputs) {
  DetectionConfig narrow_band_config;
  narrow_band_config.transmitter.bandwidth_hz = 2.0e6f;
  narrow_band_config.detection_policy.min_snr_db = -80.0f;

  DetectionConfig wide_band_config = narrow_band_config;
  wide_band_config.transmitter.bandwidth_hz = 8.0e6f;

  SignalDetector narrow_detector(narrow_band_config);
  SignalDetector wide_detector(wide_band_config);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 1.0f;
  target.range_m = 90000.0f;
  target.swerling_type = config::profiles::SwerlingModel::kSwerling1;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;

  const DetectionResult narrow_result =
      narrow_detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(), 4);
  const DetectionResult wide_result =
      wide_detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(), 4);

  EXPECT_GT(narrow_result.snr_db, wide_result.snr_db);
}

/// @brief 更多脉冲应带来更高的积分增益与检测概率。
TEST(SignalDetectorTest, HigherPulseCountYieldsHigherPd) {
  DetectionConfig config;
  config.detection_policy.min_snr_db = -80.0f;
  SignalDetector detector(config);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 1.0f;
  target.range_m = 90000.0f;
  target.swerling_type = config::profiles::SwerlingModel::kSwerling1;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 1e-14f;
  env.jam_noise_w = 0.0f;

  const DetectionResult low_pulse_count =
      detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(), 3);
  const DetectionResult high_pulse_count =
      detector.Detect(target, env, std::numeric_limits<float>::quiet_NaN(), 6);

  // 单脉冲 SNR 相同（积累前）
  EXPECT_NEAR(high_pulse_count.snr_db, low_pulse_count.snr_db, 1e-6f);
  EXPECT_GT(high_pulse_count.detection_prob, low_pulse_count.detection_prob);
}

/// @brief 雷达局部坐标应能解析出稳定的目标方位/俯仰角。
TEST(TargetLookResolverTest, ResolvesRadarLocalLookAngles) {
  session::ArSceneTarget target;
  target.position_x = 10.0f;
  target.position_y = 10.0f;
  target.position_z = 10.0f;

  const signal::detection::TargetLookAnglesDeg look_angles = TargetLookResolver::Resolve(target);
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

  config::ArOrientationConfig orientation;
  config::PlatformAttitudeDeg platform_attitude_deg;
  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState narrow_state =
      signal::detection::BeamControlResolver::Resolve(narrow_antenna, orientation,
                                                      platform_attitude_deg, look_angles);
  const signal::detection::ResolvedBeamState wide_state =
      signal::detection::BeamControlResolver::Resolve(wide_el_antenna, orientation,
                                                      platform_attitude_deg, look_angles);

  const signal::detection::MeasurementErrorState narrow_error =
      MeasurementErrorModel::Compute(20.0f, narrow_state.effective_beamwidth_deg, 1e6f);
  const signal::detection::MeasurementErrorState wide_error =
      MeasurementErrorModel::Compute(20.0f, wide_state.effective_beamwidth_deg, 1e6f);

  EXPECT_GT(wide_error.angle_error_std_rad, narrow_error.angle_error_std_rad);
}

/// @brief 启用指令态波束宽度后，应覆盖名义波束宽度参与角误差建模。
TEST(MeasurementErrorModelTest, CommandedBeamwidthOverrideAffectsAngleStdDev) {
  AntennaConfig antenna;
  antenna.nominal_az_beamwidth_deg = 2.0f;
  antenna.nominal_el_beamwidth_deg = 2.0f;

  config::ArOrientationConfig nominal_orientation;
  nominal_orientation.commanded_beamwidth_enabled = false;
  config::PlatformAttitudeDeg platform_attitude_deg;

  config::ArOrientationConfig commanded_orientation;
  commanded_orientation.commanded_beamwidth_enabled = true;
  commanded_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 8.0f;
  commanded_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg = 8.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState nominal_state =
      signal::detection::BeamControlResolver::Resolve(antenna, nominal_orientation,
                                                      platform_attitude_deg, look_angles);
  const signal::detection::ResolvedBeamState commanded_state =
      signal::detection::BeamControlResolver::Resolve(antenna, commanded_orientation,
                                                      platform_attitude_deg, look_angles);

  const signal::detection::MeasurementErrorState nominal_error =
      MeasurementErrorModel::Compute(20.0f, nominal_state.effective_beamwidth_deg, 1e6f);
  const signal::detection::MeasurementErrorState commanded_error =
      MeasurementErrorModel::Compute(20.0f, commanded_state.effective_beamwidth_deg, 1e6f);

  EXPECT_GT(commanded_error.angle_error_std_rad, nominal_error.angle_error_std_rad);
}

/// @brief 指令态波束展宽应通过方向图增益修正改善离轴目标的回波功率。
TEST(BeamControlResolverTest, CommandedBeamwidthAffectsDirectionalPatternGain) {
  DetectionConfig config;
  config.antenna.nominal_az_beamwidth_deg = 2.0f;
  config.antenna.nominal_el_beamwidth_deg = 2.0f;
  config.antenna.enable_directional_pattern = true;
  config.antenna.pattern.max_sidelobe_level_db = -25.0f;

  config::ArOrientationConfig nominal_orientation;
  nominal_orientation.commanded_beamwidth_enabled = false;
  config::PlatformAttitudeDeg platform_attitude_deg;

  config::ArOrientationConfig commanded_orientation;
  commanded_orientation.commanded_beamwidth_enabled = true;
  commanded_orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 8.0f;
  commanded_orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg = 8.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.look_az_deg = 3.0f;
  look_angles.look_el_deg = 0.0f;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState nominal_state =
      signal::detection::BeamControlResolver::Resolve(config.antenna, nominal_orientation,
                                                      platform_attitude_deg, look_angles);
  const signal::detection::ResolvedBeamState commanded_state =
      signal::detection::BeamControlResolver::Resolve(config.antenna, commanded_orientation,
                                                      platform_attitude_deg, look_angles);

  SignalDetector detector(config);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 10.0f;
  target.range_m = 50000.0f;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 2.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;

  const DetectionResult nominal =
      detector.Detect(target, env, nominal_state.one_way_antenna_gain_db, 1);
  const DetectionResult commanded =
      detector.Detect(target, env, commanded_state.one_way_antenna_gain_db, 1);

  EXPECT_GT(commanded_state.one_way_antenna_gain_db, nominal_state.one_way_antenna_gain_db);
  EXPECT_GT(commanded.echo_power_dbw, nominal.echo_power_dbw);
  EXPECT_GT(commanded.snr_db, nominal.snr_db);
}

/// @brief 对惯性稳定模式应补偿平台姿态变化。
TEST(BeamControlResolverTest, InertialStabilizedCompensatesPlatformAttitude) {
  AntennaConfig antenna;
  config::ArOrientationConfig orientation;
  orientation.stabilization_mode = config::StabilizationMode::kInertialStabilized;

  config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 15.0f;
  platform_attitude_deg.pitch_deg = 5.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState state =
      signal::detection::BeamControlResolver::Resolve(antenna, orientation, platform_attitude_deg,
                                                      look_angles);

  // 3D 姿态逆变换后，az/el 会出现轻微耦合，结果不再等同于简单 yaw/pitch 相减。
  EXPECT_NEAR(state.beam_pointing_deg.az_deg, -15.0547f, 1e-3f);
  EXPECT_NEAR(state.beam_pointing_deg.el_deg, -4.8292f, 1e-3f);
}

/// @brief 对惯性稳定模式应显式补偿平台滚转角。
TEST(BeamControlResolverTest, InertialStabilizedAccountsForPlatformRoll) {
  AntennaConfig antenna;
  config::ArOrientationConfig orientation;
  orientation.stabilization_mode = config::StabilizationMode::kInertialStabilized;
  orientation.scan_center_deg.el_deg = 30.0f;

  config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.roll_deg = 90.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState state =
      signal::detection::BeamControlResolver::Resolve(antenna, orientation, platform_attitude_deg,
                                                      look_angles);

  EXPECT_NEAR(state.beam_pointing_deg.az_deg, 30.0f, 1e-3f);
  EXPECT_NEAR(state.beam_pointing_deg.el_deg, 0.0f, 1e-3f);
}

/// @brief 对地稳定当前无地理参考输入，代码上显式等同于惯性稳定。
TEST(BeamControlResolverTest, GroundStabilizedCurrentlyMatchesInertialStabilized) {
  AntennaConfig antenna;
  config::ArOrientationConfig inertial_orientation;
  inertial_orientation.stabilization_mode = config::StabilizationMode::kInertialStabilized;
  inertial_orientation.scan_center_deg.az_deg = 5.0f;
  inertial_orientation.scan_center_deg.el_deg = 20.0f;

  config::ArOrientationConfig ground_orientation = inertial_orientation;
  ground_orientation.stabilization_mode = config::StabilizationMode::kGroundStabilized;

  config::PlatformAttitudeDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 15.0f;
  platform_attitude_deg.pitch_deg = -8.0f;
  platform_attitude_deg.roll_deg = 20.0f;

  signal::detection::TargetLookAnglesDeg look_angles;
  look_angles.has_look_angles = true;

  const signal::detection::ResolvedBeamState inertial_state =
      signal::detection::BeamControlResolver::Resolve(antenna, inertial_orientation,
                                                      platform_attitude_deg, look_angles);
  const signal::detection::ResolvedBeamState ground_state =
      signal::detection::BeamControlResolver::Resolve(antenna, ground_orientation,
                                                      platform_attitude_deg, look_angles);

  EXPECT_NEAR(inertial_state.beam_pointing_deg.az_deg, ground_state.beam_pointing_deg.az_deg,
              1e-6f);
  EXPECT_NEAR(inertial_state.beam_pointing_deg.el_deg, ground_state.beam_pointing_deg.el_deg,
              1e-6f);
}

// ===========================================================================
// Swerling 0~4 检测概率单元测试
// ===========================================================================






using config::profiles::SwerlingModel;

/// @brief Swerling 1 单脉冲 ≡ Swerling 2 单脉冲（单脉冲无所谓快慢起伏）。
TEST(SwerlingDetectionTest, Sw1_N1_Equals_Sw2_N1) {
  const float pfa = 1e-6f;
  for (float snr_db = -5.0f; snr_db <= 25.0f; snr_db += 5.0f) {
    const float pd1 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling1, 1);
    const float pd2 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling2, 1);
    EXPECT_NEAR(pd1, pd2, 1e-6f) << "SNR=" << snr_db << " dB: Sw1(N=1) should equal Sw2(N=1)";
  }
}

/// @brief Swerling 3 单脉冲 ≡ Swerling 4 单脉冲。
TEST(SwerlingDetectionTest, Sw3_N1_Equals_Sw4_N1) {
  const float pfa = 1e-6f;
  for (float snr_db = -5.0f; snr_db <= 25.0f; snr_db += 5.0f) {
    const float pd3 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling3, 1);
    const float pd4 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling4, 1);
    EXPECT_NEAR(pd3, pd4, 1e-6f) << "SNR=" << snr_db << " dB: Sw3(N=1) should equal Sw4(N=1)";
  }
}

/// @brief 所有模型：高 SNR → Pd > 0.95。
TEST(SwerlingDetectionTest, HighSNR_AllModels_PdNearOne) {
  const float pfa = 1e-6f;
  const float high_snr = 35.0f;
  const SwerlingModel models[] = {SwerlingModel::kSwerling0, SwerlingModel::kSwerling1, SwerlingModel::kSwerling2, SwerlingModel::kSwerling3, SwerlingModel::kSwerling4};
  for (auto model : models) {
    const float pd = RadarEquations::ComputeDetectionProbability(high_snr, pfa, model, 1);
    EXPECT_GT(pd, 0.95f) << "Model=" << static_cast<int>(model) << ": Pd should be > 0.95 at 35 dB";
  }
}

/// @brief 所有模型：极低 SNR → Pd 应接近 0（或虚警量级）。
TEST(SwerlingDetectionTest, LowSNR_AllModels_PdNearZero) {
  const float pfa = 1e-6f;
  const float low_snr = -20.0f;
  const SwerlingModel models[] = {SwerlingModel::kSwerling0, SwerlingModel::kSwerling1, SwerlingModel::kSwerling2, SwerlingModel::kSwerling3, SwerlingModel::kSwerling4};
  for (auto model : models) {
    const float pd = RadarEquations::ComputeDetectionProbability(low_snr, pfa, model, 1);
    EXPECT_LT(pd, 0.01f) << "Model=" << static_cast<int>(model) << ": Pd should be near 0 at -20 dB";
  }
}

/// @brief 所有模型：Pd 随 SNR 单调递增。
TEST(SwerlingDetectionTest, Monotone_Pd_Increases_With_SNR) {
  const float pfa = 1e-6f;
  const SwerlingModel models[] = {SwerlingModel::kSwerling0, SwerlingModel::kSwerling1, SwerlingModel::kSwerling2, SwerlingModel::kSwerling3, SwerlingModel::kSwerling4};
  for (auto model : models) {
    float prev_pd = 0.0f;
    for (float snr_db = -10.0f; snr_db <= 25.0f; snr_db += 1.0f) {
      const float pd = RadarEquations::ComputeDetectionProbability(snr_db, pfa, model, 1);
      EXPECT_GE(pd, prev_pd - 1e-6f)
          << "Model=" << static_cast<int>(model) << " SNR=" << snr_db << ": Pd must be monotonically non-decreasing";
      prev_pd = pd;
    }
  }
}

/// @brief 多脉冲积累应提升检测概率。
TEST(SwerlingDetectionTest, MultiPulse_Improves_Pd) {
  const float pfa = 1e-6f;
  const float snr_db = 8.0f;
  const SwerlingModel models[] = {SwerlingModel::kSwerling0, SwerlingModel::kSwerling1, SwerlingModel::kSwerling2, SwerlingModel::kSwerling3, SwerlingModel::kSwerling4};
  for (auto model : models) {
    const float pd_1 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, model, 1);
    const float pd_10 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, model, 10);
    EXPECT_GT(pd_10, pd_1) << "Model=" << static_cast<int>(model) << ": 10 pulses should improve Pd over 1 pulse";
  }
}

/// @brief 快起伏多脉冲的 Pd 最终会收敛，而慢起伏拥有更高的方差。
/// 在中等 SNR 下，两者均应在 (0, 1) 区间内，且快起伏与慢起伏差异显著。
TEST(SwerlingDetectionTest, Sw1_Sw2_MultiPulse_BothReasonable) {
  const float pfa = 1e-6f;
  const float snr_db = 3.0f;
  const int N = 10;
  const float pd1 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling1, N);
  const float pd2 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling2, N);
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
  const float pd3 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling3, N);
  const float pd4 = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling4, N);
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
  const float pd = RadarEquations::ComputeDetectionProbability(snr_db, pfa, SwerlingModel::kSwerling1, 1);
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
      EXPECT_GT(T, T_prev) << "Threshold should increase with N";
    }
  }
}

/// @brief MarcumQ 对负 a 输入应与 a=0 等价（内部会做非负钳位）。
TEST(RadarEquationsTest, MarcumQ_NegativeAIsEquivalentToZeroA) {
  const int order = 4;
  const double b = 2.7;
  const double q_negative_a = RadarEquations::MarcumQ(order, -3.5, b);
  const double q_zero_a = RadarEquations::MarcumQ(order, 0.0, b);
  EXPECT_NEAR(q_negative_a, q_zero_a, 1e-12);
}

/// @brief b=0 时 Q_M(a, 0) = 1（检测门限为零则必然检测）。
TEST(RadarEquationsTest, MarcumQ_BZeroReturnsOne) {
  for (int order : {1, 2, 4, 8}) {
    EXPECT_NEAR(RadarEquations::MarcumQ(order, 0.0, 0.0), 1.0, 1e-12);
    EXPECT_NEAR(RadarEquations::MarcumQ(order, 5.0, 0.0), 1.0, 1e-12);
    EXPECT_NEAR(RadarEquations::MarcumQ(order, 20.0, 0.0), 1.0, 1e-12);
  }
}

/// @brief a >> b 时检测概率近于 1（极高信噪比场景）。
TEST(RadarEquationsTest, MarcumQ_LargeSnrReturnsNearOne) {
  for (int order : {1, 3, 5}) {
    const double q = RadarEquations::MarcumQ(order, 30.0, 2.0);
    EXPECT_GT(q, 0.9999) << "order=" << order;
  }
}

/// @brief Q_M(a, b) 对 b 单调递减——门限越高检测概率越低。
TEST(RadarEquationsTest, MarcumQ_MonotonicDecreasingInB) {
  for (int order : {1, 2, 4}) {
    const double q_small_b = RadarEquations::MarcumQ(order, 3.0, 1.0);
    const double q_large_b = RadarEquations::MarcumQ(order, 3.0, 5.0);
    EXPECT_GT(q_small_b, q_large_b) << "order=" << order;
  }
}

/// @brief a=0 时 MarcumQ 退化为正则化不完全 Gamma 函数。
///   Q_M(0, b) = Γ(M, b²/2) / Γ(M) = gamma_q(M, b²/2)
TEST(RadarEquationsTest, MarcumQ_ZeroAReducesToGammaQ) {
  const int order = 3;
  const double b = 4.0;
  const double q = RadarEquations::MarcumQ(order, 0.0, b);
  const double x = b * b / 2.0;
  const double expected = boost::math::gamma_q(static_cast<double>(order), x);
  EXPECT_NEAR(q, expected, 1e-12);
}

/// @brief 高阶数且大 SNR 时 Q_M(a, b) 接近 1（多脉冲积累增益占主导）。
TEST(RadarEquationsTest, MarcumQ_HighOrderLargeSnrNearOne) {
  const double q = RadarEquations::MarcumQ(8, 12.0, 3.0);
  EXPECT_GT(q, 0.999);
}

// ===========================================================================
// SignalDetector — 边界条件补充
// ===========================================================================

/// @brief 目标距离为零时，ComputeEchoPower 返回保护值 -300 dBW，触发硬截断，不检测。
TEST(SignalDetectorTest, ZeroRange_TargetNotDetected) {
  DetectionConfig config;
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 100.0f;
  target.range_m = 0.0f;  // 非正距离

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;

  DetectionResult result = detector.Detect(target, env);

  EXPECT_FALSE(result.detected);
  EXPECT_FLOAT_EQ(result.detection_prob, 0.0f);
}

/// @brief 目标 RCS 为零时，回波功率为保护值，不检测。
TEST(SignalDetectorTest, ZeroRcs_TargetNotDetected) {
  DetectionConfig config;
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 0.0f;  // 零 RCS
  target.range_m = 5000.0f;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;

  DetectionResult result = detector.Detect(target, env);

  EXPECT_FALSE(result.detected);
  EXPECT_FLOAT_EQ(result.detection_prob, 0.0f);
}

/// @brief 超大杂波噪声（远超回波功率）应压制检测。
TEST(SignalDetectorTest, MassiveClutterSuppressesDetection) {
  DetectionConfig config;
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 1.0f;
  target.range_m = 5000.0f;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 1e6f;  // 极大杂波 → SNR << 0
  env.jam_noise_w = 0.0f;

  DetectionResult result = detector.Detect(target, env);

  // SNR << 0 → snr < min_snr_db → 不检测
  EXPECT_LT(result.snr_db, 0.0f);
  EXPECT_FALSE(result.detected);
  EXPECT_FLOAT_EQ(result.detection_prob, 0.0f);
}

/// @brief 总噪声接近零时，SNR 使用噪声下限保护，结果应有限且不崩溃。
TEST(SignalDetectorTest, NearZeroTotalNoise_GivesSafetySnr) {
  DetectionConfig config;
  // 将热噪声降至最低（极低带宽 + 零噪声指数）
  config.transmitter.bandwidth_hz = 1.0f;  // 1 Hz 带宽 → 极小热噪声
  config.receiver.noise_figure_db = 0.0f;  // 理想接收机
  config.receiver.receive_loss_db = 0.0f;
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 100.0f;
  target.range_m = 100.0f;

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;

  DetectionResult result = detector.Detect(target, env);

  // 噪声极小时，SNR 应为正值且不产生 NaN/Inf
  EXPECT_TRUE(std::isfinite(result.snr_db));
  EXPECT_GT(result.snr_db, 0.0f);
}

/// @brief 非正噪声输入应被钳位为非负，不得造成虚高 SNR。
TEST(SignalDetectorTest, NonPositiveNoiseInputDoesNotInflateSnr) {
  DetectionConfig config;
  config.detection_policy.min_snr_db = -80.0f;
  SignalDetector detector(config);
  detector.SetRandomSeed(42u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 1.0f;
  target.range_m = 100000.0f;
  target.swerling_type = SwerlingModel::kSwerling0;

  signal::detection::EnvironmentState baseline_env;
  baseline_env.propagation_loss_db = 0.0f;
  baseline_env.clutter_noise_w = 0.0f;
  baseline_env.jam_noise_w = 0.0f;

  const DetectionResult baseline = detector.Detect(target, baseline_env);

  detector.SetRandomSeed(42u);
  signal::detection::EnvironmentState invalid_env = baseline_env;
  invalid_env.clutter_noise_w = -1e5f;
  invalid_env.jam_noise_w = -1e5f;
  const DetectionResult invalid = detector.Detect(target, invalid_env);

  EXPECT_TRUE(std::isfinite(invalid.snr_db));
  EXPECT_NEAR(invalid.snr_db, baseline.snr_db, 1e-4f);
  EXPECT_NEAR(invalid.detection_prob, baseline.detection_prob, 1e-6f);
}

/// @brief snr < min_snr_db 时，detection_prob 强制为 0，detected = false。
TEST(SignalDetectorTest, BelowMinSnrIsNeverDetected) {
  DetectionConfig config;
  config.detection_policy.min_snr_db = 20.0f;  // 极高硬门限
  SignalDetector detector(config);
  detector.SetRandomSeed(0u);

  signal::detection::TargetReturn target;
  target.rcs_m2 = 0.001f;      // 很小 RCS
  target.range_m = 500000.0f;  // 极远

  signal::detection::EnvironmentState env;
  env.propagation_loss_db = 0.0f;
  env.clutter_noise_w = 0.0f;
  env.jam_noise_w = 0.0f;

  DetectionResult result = detector.Detect(target, env);

  EXPECT_FALSE(result.detected);
  EXPECT_FLOAT_EQ(result.detection_prob, 0.0f);
}

/// @brief 积累脉冲数 <= 0 时，积累增益退化为 1.0（不崩溃）。
TEST(RadarEquationsTest, IntegrationGain_ZeroPulseCount_IsOne) {
  EXPECT_FLOAT_EQ(RadarEquations::ComputeIntegrationGain(0), 1.0f);
  EXPECT_FLOAT_EQ(RadarEquations::ComputeIntegrationGain(-1), 1.0f);
}

/// @brief 单脉冲积累增益固定为 1.0。
TEST(RadarEquationsTest, IntegrationGain_OnePulse_IsOne) {
  EXPECT_FLOAT_EQ(RadarEquations::ComputeIntegrationGain(1), 1.0f);
}

}  // namespace tests
}  // namespace airborne_radar
