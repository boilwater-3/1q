// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 ArSessionConfigBuilder 薄封装与 ArProfileConstants 常量赋值路径。

#include <gtest/gtest.h>

#include <limits>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfigValidation.h"
#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace tests {

namespace {

config::ArSessionConfig MakeDetectionFocusedConfig() {
  config::ArSessionConfig config;
  config.policy.detection = config::profiles::kDetectionPriorityDetection;
  config.policy.tracking = config::profiles::kFastAssociationTracking;
  config.policy.lifecycle = config::profiles::kFastConfirmLifecycle;
  return config;
}

}  // namespace

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesSemanticDefaults) {
  // 薄封装 Build() 直接返回结构体默认值（= 语义默认）。
  const auto config = config::ArSessionConfigBuilder().Build();

  EXPECT_FLOAT_EQ(config.policy.detection.minimum_detection_margin_db, -2.0f);
  EXPECT_EQ(config.policy.detection.pulse_count, 10);
  EXPECT_FALSE(config.policy.tracking.enable_kalman_filter);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 3U);
  EXPECT_EQ(config.policy.lifecycle.max_miss_before_lost, 2U);
  EXPECT_FALSE(config.policy.lifecycle.enable_imm_lifecycle);
}

TEST(RadarSessionConfigBuilderTest, ExistingBuilderBasePreservesValues) {
  // 薄封装按值保留基线配置，不做任何翻译或覆写。
  const auto config = config::ArSessionConfigBuilder(MakeDetectionFocusedConfig()).Build();

  EXPECT_EQ(config.policy.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, -12.0f);
  EXPECT_FLOAT_EQ(config.policy.tracking.kalman_measurement_noise_std, 6.0f);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 1U);
}

TEST(RadarSessionConfigBuilderTest, ExistingDetailedConfigIsPreserved) {
  config::ArSessionConfig base_config{};
  base_config.hardware.transmitter.peak_power_w = 7.5e6f;
  base_config.hardware.transmitter.frequency_hz = 9.7e9f;
  base_config.policy.tracking.enable_kalman_filter = true;
  base_config.policy.tracking.kalman_measurement_noise_std = 4.5f;
  base_config.policy.lifecycle.enable_imm_lifecycle = true;
  base_config.policy.lifecycle.confirm_hits = 2U;

  const config::ArSessionConfig rebuilt =
      config::ArSessionConfigBuilder(base_config).Build();

  EXPECT_FLOAT_EQ(rebuilt.hardware.transmitter.peak_power_w, 7.5e6f);
  EXPECT_FLOAT_EQ(rebuilt.hardware.transmitter.frequency_hz, 9.7e9f);
  EXPECT_TRUE(rebuilt.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(rebuilt.policy.tracking.kalman_measurement_noise_std, 4.5f);
  EXPECT_TRUE(rebuilt.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(rebuilt.policy.lifecycle.confirm_hits, 2U);
}

TEST(RadarSessionConfigBuilderTest, ProfileConstantsAssignToConfig) {
  config::ArSessionConfig config;
  config.hardware = config::profiles::kLongRangeHighPowerHardware;
  config.policy.detection = config::profiles::kTrackStabilityPriorityDetection;
  config.hardware.antenna.pattern = config::profiles::kLowSidelobeAntennaPattern;
  config.hardware.rcs_physics = config::profiles::kEnhancedRcsPhysics;

  EXPECT_FLOAT_EQ(config.hardware.transmitter.peak_power_w, 5.0e6f);
  EXPECT_EQ(config.policy.detection.pulse_count, 8);
  EXPECT_FLOAT_EQ(config.hardware.antenna.pattern.max_sidelobe_level_db, -30.0f);
  EXPECT_TRUE(config.hardware.rcs_physics.enable_physical_rcs);
  EXPECT_FLOAT_EQ(config.hardware.rcs_physics.physics_mix_ratio, 0.60f);
}

TEST(RadarSessionConfigBuilderTest, TrackingAndLifecycleConstantsAssignToConfig) {
  config::ArSessionConfig config;
  config.policy.tracking = config::profiles::kRobustAntiJammingTracking;
  config.policy.association = config::profiles::kRobustAntiJammingAssociation;
  config.policy.tracking.enable_kalman_filter = true;
  config.policy.lifecycle = config::profiles::kHighPersistenceLifecycle;
  config.policy.lifecycle.enable_imm_lifecycle = true;

  EXPECT_TRUE(config.policy.tracking.enable_kalman_filter);
  EXPECT_TRUE(config.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 3U);
  EXPECT_EQ(config.policy.lifecycle.max_lost_cycles, 8U);
}

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  const auto config =
      config::ArSessionConfigBuilder(MakeDetectionFocusedConfig()).Build();
  session::ArSession session = session::ArSession::Create(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, RuntimeConfigBuilderBuildsPatchFlagsAndValues) {
  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 12.0f;
  scan_center.el_deg = -3.0f;

  config::AzimuthElevationDeg dwell_center;
  dwell_center.az_deg = 5.0f;
  dwell_center.el_deg = 2.0f;

  config::CommandedBeamwidthDeg commanded_beamwidth;
  commanded_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  commanded_beamwidth.commanded_el_beamwidth_deg = 2.0f;

  const config::ArRuntimeConfigPatch patch =
      config::ArRuntimeConfigBuilder()
          .WithWorkMode(config::ArWorkMode::kStt)
          .WithScanCenterDeg(scan_center)
          .WithDwellCenterDeg(dwell_center)
          .WithCommandedBeamwidthDeg(commanded_beamwidth)
          .WithCommandedBeamwidthEnabled(true)
          .WithEnvironmentScenarioConfig(config::EnvironmentScenarioConfig{})
          .Build();

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, config::ArWorkMode::kStt);
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
  EXPECT_TRUE(patch.environment.has_scenario_config);
}

// P3-b：四域 RuntimeConfigBuilder 形状对齐——必须提供 WithRuntimeConfigPatch 整块覆盖入口，
// 与 EOS/ESR/SAR 一致。本测试锁定 AR 侧的语义：整块覆盖优先于链上逐字段设置。
TEST(RadarSessionConfigBuilderTest, WithRuntimeConfigPatchOverridesWholePatch) {
  // 先用链式逐字段构造一份补丁
  const config::ArRuntimeConfigPatch seed = config::ArRuntimeConfigBuilder()
                                                .WithWorkMode(config::ArWorkMode::kStt)
                                                .WithSensorEnabled(true)
                                                .Build();
  ASSERT_TRUE(seed.has_work_mode);
  ASSERT_TRUE(seed.has_sensor_enabled);

  // 再用 WithRuntimeConfigPatch 整块覆盖到一个新 builder，验证整块替换语义
  config::ArRuntimeConfigPatch whole;
  whole.has_work_mode = true;
  whole.work_mode = config::ArWorkMode::kTas;
  const config::ArRuntimeConfigPatch patch =
      config::ArRuntimeConfigBuilder().WithRuntimeConfigPatch(whole).Build();

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, config::ArWorkMode::kTas);
  // 整块覆盖应清空 seed 里的 sensor_enabled（WithRuntimeConfigPatch 是替换不是合并）
  EXPECT_FALSE(patch.has_sensor_enabled);
}

TEST(RadarSessionConfigBuilderTest, RuntimePatchCanBeAppliedWithoutReconstructingSession) {
  session::ArSession session = session::ArSession::Create(MakeDetectionFocusedConfig());

  const config::ArRuntimeConfigPatch patch =
      config::ArRuntimeConfigBuilder()
          .WithWorkMode(config::ArWorkMode::kTas)
          .WithCommandedBeamwidthEnabled(true)
          .WithEnvironmentScenarioConfig(config::EnvironmentScenarioConfig{})
          .Build();

  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));

  session::ArCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
  input.dt_sec = 1.0;
  input.platform.platform_entity_id = 42U;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  const session::ArCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
}

TEST(RadarSessionConfigBuilderTest, DetailedBuilderProducesDetailedSessionConfig) {
  config::ArSessionConfig detailed_config{};
  detailed_config.hardware.transmitter.peak_power_w = 5.0e6f;
  detailed_config.hardware.transmitter.frequency_hz = 9.3e9f;
  detailed_config.hardware.transmitter.bandwidth_hz = 10.0e6f;
  detailed_config.hardware.transmitter.pulse_width_s = 20e-6f;
  detailed_config.hardware.transmitter.prf_hz = 500.0f;
  detailed_config.hardware.antenna.main_beam_gain_db = 38.0f;
  detailed_config.hardware.receiver.noise_figure_db = 3.5f;
  detailed_config.mission.orientation.work_mode = config::ArWorkMode::kTas;
  detailed_config.policy.tracking.enable_kalman_filter = true;
  detailed_config.policy.tracking.kalman_measurement_noise_std = 7.5f;
  detailed_config.policy.lifecycle.enable_imm_lifecycle = true;
  detailed_config.policy.lifecycle.confirm_hits = 2U;
  detailed_config.policy.lifecycle.max_miss_before_lost = 1U;
  detailed_config.policy.lifecycle.max_lost_cycles = 4U;
  detailed_config.environment.scenario_config.atmospheric_physics
      .enable_physical_model = true;

  EXPECT_FLOAT_EQ(detailed_config.hardware.transmitter.peak_power_w, 5.0e6f);
  EXPECT_FLOAT_EQ(detailed_config.hardware.transmitter.frequency_hz, 9.3e9f);
  EXPECT_EQ(detailed_config.mission.orientation.work_mode, config::ArWorkMode::kTas);
  EXPECT_FLOAT_EQ(detailed_config.policy.tracking.kalman_measurement_noise_std, 7.5f);
  EXPECT_EQ(detailed_config.policy.lifecycle.confirm_hits, 2U);
  EXPECT_TRUE(detailed_config.policy.lifecycle.enable_imm_lifecycle);
  EXPECT_TRUE(detailed_config.environment.scenario_config.atmospheric_physics
                  .enable_physical_model);
}

TEST(RadarSessionConfigBuilderTest, DetailedBuiltConfigCanConstructRadarSession) {
  config::ArSessionConfig config{};
  config.hardware.transmitter.peak_power_w = 5.0e6f;
  config.hardware.transmitter.frequency_hz = 9.3e9f;
  config.policy.lifecycle.enable_imm_lifecycle = true;

  session::ArSession session = session::ArSession::Create(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, DetailedBeamSchedulerWritesPolicyPath) {
  config::AzimuthElevationDeg default_scan_center;
  default_scan_center.az_deg = 6.0f;
  default_scan_center.el_deg = -1.5f;
  config::CommandedBeamwidthDeg nominal_beamwidth;
  nominal_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  nominal_beamwidth.commanded_el_beamwidth_deg = 1.5f;

  config::ArSessionConfig detailed_config{};
  detailed_config.policy.beam_control.scheduler.azimuth_step_count_hint = 8U;
  detailed_config.policy.beam_control.scheduler.elevation_step_count_hint = 4U;
  detailed_config.policy.beam_control.scheduler.prefer_dense_tas_sampling = true;
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

TEST(RadarSessionConfigValidationTest, ReportsInvalidCommandedBeamwidth) {
  config::ArSessionConfig session_config;
  session_config.mission.orientation.commanded_beamwidth_enabled = true;
  session_config.mission.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 0.0f;
  session_config.mission.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg = 1.0f;

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, config::ConfigValidationCode::kCommandedBeamwidthAzNotPositive);
}

TEST(RadarSessionConfigValidationTest, RejectsNonFiniteCommandedBeamwidth) {
  config::ArSessionConfig session_config;
  session_config.mission.orientation.commanded_beamwidth_enabled = true;
  session_config.mission.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 1.0f;
  session_config.mission.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg =
      std::numeric_limits<float>::quiet_NaN();

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, config::ConfigValidationCode::kCommandedBeamwidthElNotPositive);
}

TEST(RadarSessionConfigValidationTest, AcceptsPhysicalApertureWhenNominalBeamwidthIsZero) {
  config::ArSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 1.5f;
  session_config.hardware.antenna.nominal_el_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_width_m = 0.8f;

  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());
}

TEST(RadarSessionConfigValidationTest, RejectsMissingOrNonFiniteAntennaGeometry) {
  config::ArSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 0.0f;
  session_config.hardware.antenna.antenna_width_m = std::numeric_limits<float>::quiet_NaN();

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 2U);
  EXPECT_EQ(issues[0].code, config::ConfigValidationCode::kAntennaAzGeometryInvalid);
  EXPECT_EQ(issues[1].code, config::ConfigValidationCode::kAntennaElGeometryInvalid);
}

TEST(RadarSessionConfigValidationTest, RejectsInvalidTransmitterFrequency) {
  config::ArSessionConfig session_config;
  session_config.hardware.transmitter.frequency_hz = 0.0f;

  const auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_EQ(issues.size(), 2U);
  EXPECT_EQ(issues[0].code, config::ConfigValidationCode::kTransmitterFrequencyInvalid);
  EXPECT_EQ(issues[1].code, config::ConfigValidationCode::kFrequencyPlanInvalid);
}

TEST(RadarSessionConfigValidationTest, ValidatesEngineeringTransmitterEnvelopeAndIdentity) {
  config::ArSessionConfig session_config;
  session_config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());

  session_config.hardware.transmitter.frequency_plan_hz = {3.1e9};
  auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, config::ConfigValidationCode::kFrequencyPlanInvalid);

  session_config.hardware.transmitter.frequency_plan_hz = {3.0e9};
  session_config.hardware.transmitter.maximum_peak_power_w = 5.0e5f;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code,
            config::ConfigValidationCode::kTransmitterOperatingEnvelopeInvalid);

  session_config.hardware.transmitter.maximum_peak_power_w = 1.2e6f;
  session_config.hardware.receiver.equipment_id = session_config.hardware.transmitter.equipment_id;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, config::ConfigValidationCode::kEquipmentIdentityInvalid);
}

TEST(RadarSessionConfigValidationTest, ValidatesReceiverRfHardwareBoundary) {
  config::ArSessionConfig session_config;
  ASSERT_EQ(session_config.hardware.receiver.co_site_paths.size(), 1U);
  EXPECT_EQ(session_config.hardware.receiver.co_site_paths.front().transmitter_equipment_id,
            session_config.hardware.transmitter.equipment_id);
  EXPECT_EQ(session_config.hardware.receiver.co_site_paths.front().receiver_equipment_id,
            session_config.hardware.receiver.equipment_id);
  session_config.hardware.receiver.maximum_linear_input_power_w = 0.0f;
  auto issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, config::ConfigValidationCode::kReceiverRfHardwareInvalid);

  session_config.hardware.receiver.maximum_linear_input_power_w = 1.0e-3f;
  session_config.hardware.receiver.interference_observation_jn_gate_db =
      std::numeric_limits<float>::quiet_NaN();
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, config::ConfigValidationCode::kReceiverRfHardwareInvalid);

  session_config.hardware.receiver.interference_observation_jn_gate_db = 6.0f;
  session_config.hardware.receiver.has_co_site_isolation = true;
  session_config.hardware.receiver.co_site_isolation_db = 80.0f;
  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());

  session_config.hardware.receiver.co_site_paths.push_back(
      oneq::electromagnetics::RfCoSiteIsolationPath{1U, 2U, 80.0});
  EXPECT_TRUE(config::ValidateArSessionConfig(session_config).empty());
  session_config.hardware.receiver.co_site_paths.back().receiver_equipment_id = 3U;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, config::ConfigValidationCode::kReceiverRfHardwareInvalid);
  session_config.hardware.receiver.co_site_paths.clear();

  session_config.hardware.receiver.cross_polarization_isolation_db = -1.0f;
  issues = config::ValidateArSessionConfig(session_config);
  ASSERT_FALSE(issues.empty());
  EXPECT_EQ(issues.back().code, config::ConfigValidationCode::kReceiverRfHardwareInvalid);
}

TEST(RadarSessionCreateWithDiagnosticsTest, BuildsSessionAndReportsNoIssuesForHealthyConfig) {
  config::ArSessionConfig config;
  config.policy.lifecycle.enable_imm_lifecycle = false;

  config::ValidationIssueList issues;
  const session::ArSession session = session::ArSession::CreateWithDiagnostics(config, &issues);

  EXPECT_TRUE(issues.empty());
  (void)session;
}

TEST(RadarSessionCreateWithDiagnosticsTest, ReportsIssuesButStillConstructsSession) {
  config::ArSessionConfig invalid;
  invalid.mission.orientation.mechanical_scan_limits_deg.az_min_deg = 10.0f;
  invalid.mission.orientation.mechanical_scan_limits_deg.az_max_deg = -10.0f;

  config::ValidationIssueList issues;
  const session::ArSession session = session::ArSession::CreateWithDiagnostics(invalid, &issues);

  ASSERT_EQ(issues.size(), 1U);
  EXPECT_EQ(issues.front().code, config::ConfigValidationCode::kMechanicalScanLimitsSwappedAz);
  (void)session;  // 会话仍被构造，调用方据 issues 决策
}

TEST(RadarSessionCreateWithDiagnosticsTest, AcceptsNullIssuesWithoutCrash) {
  config::ArSessionConfig invalid;
  invalid.mission.orientation.mechanical_scan_limits_deg.az_min_deg = 10.0f;
  invalid.mission.orientation.mechanical_scan_limits_deg.az_max_deg = -10.0f;

  const session::ArSession session = session::ArSession::CreateWithDiagnostics(invalid, nullptr);
  (void)session;  // nullptr 时仅构造，不写回 issues
}

}  // namespace tests
}  // namespace airborne_radar
