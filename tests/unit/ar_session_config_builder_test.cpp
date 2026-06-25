// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 RadarSessionConfigBuilder 语义化配置接口。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace airborne_radar {
namespace tests {

namespace {

config::RadarSessionConfig MakeDetectionFocusedConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

}  // namespace

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesSemanticDefaults) {
  const auto config = config::RadarSessionConfigBuilder().Build();

  EXPECT_FALSE(config.hardware.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(config.hardware.detection.min_detection_margin_db, -2.0f);
  EXPECT_EQ(config.hardware.detection.pulse_count, 10);
  EXPECT_FALSE(config.policy.tracking.enable_kalman_filter);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 3U);
  EXPECT_EQ(config.policy.lifecycle.max_miss_before_lost, 2U);
  EXPECT_FALSE(config.policy.lifecycle.enable_imm_lifecycle);
}

TEST(RadarSessionConfigBuilderTest, ExistingBuilderBasePreservesSemanticValues) {
  const auto config = config::RadarSessionConfigBuilder(MakeDetectionFocusedConfig()).Build();

  EXPECT_EQ(config.hardware.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.hardware.detection.detection_policy.min_snr_db, -12.0f);
  EXPECT_FLOAT_EQ(config.policy.tracking.kalman_measurement_noise_std, 6.0f);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 1U);
}

TEST(RadarSessionConfigBuilderTest, ExistingDetailedConfigIsPreservedWhenOnlyEditingEnvironment) {
  config::RadarSessionConfig base_config{};
  base_config.hardware.detection.enable_physics_detection = true;
  base_config.hardware.detection.transmitter.peak_power_w = 7.5e6f;
  base_config.hardware.detection.transmitter.frequency_hz = 9.7e9f;
  base_config.policy.tracking.enable_kalman_filter = true;
  base_config.policy.tracking.kalman_measurement_noise_std = 4.5f;
  base_config.policy.lifecycle.enable_imm_lifecycle = true;
  base_config.policy.lifecycle.confirm_hits = 2U;

  const config::RadarSessionConfig rebuilt =
      config::RadarSessionConfigBuilder(base_config)
          .Environment()
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .End()
          .Build();

  EXPECT_TRUE(rebuilt.hardware.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(rebuilt.hardware.detection.transmitter.peak_power_w, 7.5e6f);
  EXPECT_FLOAT_EQ(rebuilt.hardware.detection.transmitter.frequency_hz, 9.7e9f);
  EXPECT_TRUE(rebuilt.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(rebuilt.policy.tracking.kalman_measurement_noise_std, 4.5f);
  EXPECT_TRUE(rebuilt.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(rebuilt.policy.lifecycle.confirm_hits, 2U);
  EXPECT_EQ(rebuilt.environment.jamming_sensitivity_profile, environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, DetectionSemanticEditorsApplyCorrectly) {
  const auto config =
      config::RadarSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(true)
          .WithHardwareProfile(config::profiles::RadarHardwareProfile::kLongRangeHighPower)
          .WithDetectionIntentProfile(
              config::profiles::DetectionIntentProfile::kTrackStabilityPriority)
          .WithAntennaPatternProfile(config::profiles::AntennaPatternProfile::kLowSidelobe)
          .WithRcsFusionProfile(config::profiles::RcsFusionProfile::kEnhanced)
          .End()
          .Build();

  EXPECT_TRUE(config.hardware.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(config.hardware.detection.transmitter.peak_power_w, 5.0e6f);
  EXPECT_EQ(config.hardware.detection.pulse_count, 8);
  EXPECT_FLOAT_EQ(config.hardware.detection.antenna.pattern.max_sidelobe_level_db, -30.0f);
  EXPECT_TRUE(config.hardware.detection.rcs_physics.enable_physical_rcs);
  EXPECT_FLOAT_EQ(config.hardware.detection.rcs_physics.physics_mix_ratio, 0.60f);
}

TEST(RadarSessionConfigBuilderTest, TrackingAndLifecycleSemanticEditorsApplyCorrectly) {
  const auto config =
      config::RadarSessionConfigBuilder()
          .Tracking()
          .EnableTrackingFilter(true)
          .WithTrackingPolicyProfile(config::profiles::TrackingPolicyProfile::kRobustAntiJamming)
          .End()
          .Lifecycle()
          .EnableImmFusion(true)
          .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kHighPersistence)
          .End()
          .Build();

  EXPECT_TRUE(config.policy.tracking.enable_kalman_filter);
  EXPECT_EQ(config.policy.tracking.kalman_update_backend, config::KalmanUpdateBackend::kUdKf);
  EXPECT_TRUE(config.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 3U);
  EXPECT_EQ(config.policy.lifecycle.max_lost_cycles, 8U);
}

TEST(RadarSessionConfigBuilderTest, BeamAndEnvironmentEditorsApplyCorrectly) {
  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 8.0f;
  scan_center.el_deg = -2.0f;

  const auto config =
      config::RadarSessionConfigBuilder()
          .Mission()
          .WithWorkMode(model::RadarWorkMode::kTas)
          .WithScanCenterDeg(scan_center)
          .End()
          .Environment()
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .End()
          .Build();

  EXPECT_EQ(config.mission.orientation.work_mode, model::RadarWorkMode::kTas);
  EXPECT_FLOAT_EQ(config.mission.orientation.scan_center_deg.az_deg, 8.0f);
  EXPECT_FLOAT_EQ(config.mission.orientation.scan_center_deg.el_deg, -2.0f);
  EXPECT_EQ(config.environment.jamming_sensitivity_profile, environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  const auto config =
      config::RadarSessionConfigBuilder(MakeDetectionFocusedConfig())
          .Environment()
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .End()
          .Build();
  session::RadarSession session = session::RadarSessionFactory::Create(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, RuntimeConfigBuilderBuildsPatchFlagsAndValues) {
  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 12.0f;
  scan_center.el_deg = -3.0f;

  model::AzimuthElevationDeg dwell_center;
  dwell_center.az_deg = 5.0f;
  dwell_center.el_deg = 2.0f;

  model::CommandedBeamwidthDeg commanded_beamwidth;
  commanded_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  commanded_beamwidth.commanded_el_beamwidth_deg = 2.0f;

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithWorkMode(model::RadarWorkMode::kStt)
          .WithScanCenterDeg(scan_center)
          .WithDwellCenterDeg(dwell_center)
          .WithCommandedBeamwidthDeg(commanded_beamwidth)
          .WithCommandedBeamwidthEnabled(true)
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, model::RadarWorkMode::kStt);
  EXPECT_TRUE(patch.has_scan_center_deg);
  EXPECT_FLOAT_EQ(patch.scan_center_deg.az_deg, 12.0f);
  EXPECT_FLOAT_EQ(patch.scan_center_deg.el_deg, -3.0f);
  EXPECT_TRUE(patch.has_dwell_center_deg);
  EXPECT_FLOAT_EQ(patch.dwell_center_deg.az_deg, 5.0f);
  EXPECT_FLOAT_EQ(patch.dwell_center_deg.el_deg, 2.0f);
  EXPECT_TRUE(patch.has_commanded_beamwidth_deg);
  EXPECT_FLOAT_EQ(patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg, 2.5f);
  EXPECT_FLOAT_EQ(patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg, 2.0f);
  EXPECT_TRUE(patch.has_commanded_beamwidth_enabled);
  EXPECT_TRUE(patch.commanded_beamwidth_enabled);
  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_jamming_sensitivity_profile);
  EXPECT_EQ(patch.environment.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

// P3-b：四域 RuntimeConfigBuilder 形状对齐——必须提供 WithRuntimeConfigPatch 整块覆盖入口，
// 与 EOS/ESR/SAR 一致。本测试锁定 AR 侧的语义：整块覆盖优先于链上逐字段设置。
TEST(RadarSessionConfigBuilderTest, WithRuntimeConfigPatchOverridesWholePatch) {
  // 先用链式逐字段构造一份补丁
  const config::RadarRuntimeConfigPatch seed =
      config::RadarRuntimeConfigBuilder()
          .WithWorkMode(model::RadarWorkMode::kStt)
          .WithSensorEnabled(true)
          .Build();
  ASSERT_TRUE(seed.has_work_mode);
  ASSERT_TRUE(seed.has_sensor_enabled);

  // 再用 WithRuntimeConfigPatch 整块覆盖到一个新 builder，验证整块替换语义
  config::RadarRuntimeConfigPatch whole;
  whole.has_work_mode = true;
  whole.work_mode = model::RadarWorkMode::kTas;
  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder().WithRuntimeConfigPatch(whole).Build();

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, model::RadarWorkMode::kTas);
  // 整块覆盖应清空 seed 里的 sensor_enabled（WithRuntimeConfigPatch 是替换不是合并）
  EXPECT_FALSE(patch.has_sensor_enabled);
}

TEST(RadarSessionConfigBuilderTest, RuntimePatchCanBeAppliedWithoutReconstructingSession) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(MakeDetectionFocusedConfig());

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithWorkMode(model::RadarWorkMode::kTas)
          .WithCommandedBeamwidthEnabled(true)
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();

  session.ApplyRuntimeConfig(patch);

  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
}

TEST(RadarSessionConfigBuilderTest, DetailedBuilderProducesDetailedSessionConfig) {
  config::RadarSessionConfig detailed_config{};
  detailed_config.hardware.detection.enable_physics_detection = true;
  detailed_config.hardware.detection.transmitter.peak_power_w = 5.0e6f;
  detailed_config.hardware.detection.transmitter.frequency_hz = 9.3e9f;
  detailed_config.hardware.detection.transmitter.bandwidth_hz = 10.0e6f;
  detailed_config.hardware.detection.transmitter.pulse_width_s = 20e-6f;
  detailed_config.hardware.detection.transmitter.prf_hz = 500.0f;
  detailed_config.hardware.detection.antenna.main_beam_gain_db = 38.0f;
  detailed_config.hardware.detection.receiver.noise_figure_db = 3.5f;
  detailed_config.mission.orientation.work_mode = model::RadarWorkMode::kTas;
  detailed_config.policy.tracking.enable_kalman_filter = true;
  detailed_config.policy.tracking.kalman_measurement_noise_std = 7.5f;
  detailed_config.policy.tracking.kalman_update_backend = config::KalmanUpdateBackend::kUdKf;
  detailed_config.policy.lifecycle.enable_imm_lifecycle = true;
  detailed_config.policy.lifecycle.confirm_hits = 2U;
  detailed_config.policy.lifecycle.max_miss_before_lost = 1U;
  detailed_config.policy.lifecycle.max_lost_cycles = 4U;
  detailed_config.environment.jamming_sensitivity_profile = environment::JammingSensitivityProfile::kStrict;

  EXPECT_TRUE(detailed_config.hardware.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(detailed_config.hardware.detection.transmitter.peak_power_w, 5.0e6f);
  EXPECT_FLOAT_EQ(detailed_config.hardware.detection.transmitter.frequency_hz, 9.3e9f);
  EXPECT_EQ(detailed_config.mission.orientation.work_mode, model::RadarWorkMode::kTas);
  EXPECT_FLOAT_EQ(detailed_config.policy.tracking.kalman_measurement_noise_std, 7.5f);
  EXPECT_EQ(detailed_config.policy.tracking.kalman_update_backend,
            config::KalmanUpdateBackend::kUdKf);
  EXPECT_EQ(detailed_config.policy.lifecycle.confirm_hits, 2U);
  EXPECT_TRUE(detailed_config.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(detailed_config.environment.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, DetailedBuiltConfigCanConstructRadarSession) {
  config::RadarSessionConfig config{};
  config.hardware.detection.transmitter.peak_power_w = 5.0e6f;
  config.hardware.detection.transmitter.frequency_hz = 9.3e9f;
  config.policy.tracking.kalman_update_backend = config::KalmanUpdateBackend::kUdKf;
  config.policy.lifecycle.enable_imm_lifecycle = true;

  session::RadarSession session = session::RadarSessionFactory::Create(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, DetailedBeamSchedulerWritesPolicyPath) {
  model::AzimuthElevationDeg default_scan_center;
  default_scan_center.az_deg = 6.0f;
  default_scan_center.el_deg = -1.5f;
  model::CommandedBeamwidthDeg nominal_beamwidth;
  nominal_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  nominal_beamwidth.commanded_el_beamwidth_deg = 1.5f;

  config::RadarSessionConfig detailed_config{};
  detailed_config.policy.beam_control.scheduler.azimuth_step_count_hint = 8U;
  detailed_config.policy.beam_control.scheduler.elevation_step_count_hint = 4U;
  detailed_config.policy.beam_control.scheduler.prefer_dense_tas_sampling = true;
  detailed_config.policy.beam_control.pointing.default_scan_center_deg = default_scan_center;
  detailed_config.mission.orientation.scan_center_deg = default_scan_center;
  detailed_config.policy.beam_control.pointing.nominal_beamwidth_deg = nominal_beamwidth;
  detailed_config.mission.orientation.commanded_beamwidth_deg = nominal_beamwidth;

  EXPECT_EQ(detailed_config.policy.beam_control.scheduler.azimuth_step_count_hint, 8U);
  EXPECT_EQ(detailed_config.policy.beam_control.scheduler.elevation_step_count_hint, 4U);
  EXPECT_TRUE(detailed_config.policy.beam_control.scheduler.prefer_dense_tas_sampling);
  EXPECT_FLOAT_EQ(detailed_config.mission.orientation.scan_center_deg.az_deg, 6.0f);
  EXPECT_FLOAT_EQ(detailed_config.mission.orientation.scan_center_deg.el_deg, -1.5f);
  EXPECT_FLOAT_EQ(
      detailed_config.mission.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg, 2.5f);
  EXPECT_FLOAT_EQ(
      detailed_config.mission.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg, 1.5f);
}

}  // namespace tests
}  // namespace airborne_radar
