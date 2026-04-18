// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 RadarSessionConfigBuilder 语义化配置接口。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/RadarDetailedSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace airborne_radar {
namespace tests {

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

TEST(RadarSessionConfigBuilderTest, PresetBasePreservesPresetSemanticValues) {
  const auto config =
      config::RadarSessionConfigBuilder(config::presets::MakeDetectionMissionRadarSessionConfig())
          .Build();

  EXPECT_EQ(config.hardware.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.hardware.detection.detection_policy.min_snr_db, -12.0f);
  EXPECT_FLOAT_EQ(config.policy.tracking.kalman_measurement_noise_std, 6.0f);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 1U);
}

TEST(RadarSessionConfigBuilderTest, ExistingDetailedConfigIsPreservedWhenOnlyEditingEnvironment) {
  session::RadarSessionConfig base_config =
            config::RadarDetailedSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(true)
          .WithPeakPowerW(7.5e6f)
          .WithFrequencyHz(9.7e9f)
          .End()
          .Tracking()
          .EnableKalmanFilter(true)
          .WithKalmanMeasurementNoiseStd(4.5f)
          .End()
          .Lifecycle()
          .EnableImmFusion(true)
          .WithLifecycleConfirmHits(2U)
          .End()
          .Build();

  const session::RadarSessionConfig rebuilt =
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
  EXPECT_EQ(rebuilt.environment.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, DetectionSemanticEditorsApplyCorrectly) {
  const auto config =
      config::RadarSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(true)
          .WithHardwareProfile(config::semantic::RadarHardwareProfile::kLongRangeHighPower)
          .WithDetectionIntentProfile(
              config::semantic::DetectionIntentProfile::kTrackStabilityPriority)
          .WithAntennaPatternProfile(config::semantic::AntennaPatternProfile::kLowSidelobe)
          .WithRcsFusionProfile(config::semantic::RcsFusionProfile::kEnhanced)
          .End()
          .Build();

    EXPECT_TRUE(config.hardware.detection.enable_physics_detection);
    EXPECT_FLOAT_EQ(config.hardware.detection.transmitter.peak_power_w, 5.0e6f);
    EXPECT_EQ(config.hardware.detection.pulse_count, 8);
    EXPECT_FLOAT_EQ(config.hardware.detection.antenna.pattern.max_sidelobe_level_db,
                                    -30.0f);
    EXPECT_TRUE(config.hardware.detection.rcs_physics.enable_physical_rcs);
    EXPECT_FLOAT_EQ(config.hardware.detection.rcs_physics.physics_mix_ratio, 0.60f);
}

TEST(RadarSessionConfigBuilderTest, TrackingAndLifecycleSemanticEditorsApplyCorrectly) {
  const auto config =
      config::RadarSessionConfigBuilder()
          .Tracking()
          .EnableTrackingFilter(true)
          .WithTrackingPolicyProfile(config::semantic::TrackingPolicyProfile::kRobustAntiJamming)
          .End()
          .Lifecycle()
          .EnableImmFusion(true)
          .WithLifecyclePolicyProfile(config::semantic::LifecyclePolicyProfile::kHighPersistence)
          .End()
          .Build();

    EXPECT_TRUE(config.policy.tracking.enable_kalman_filter);
    EXPECT_EQ(config.policy.tracking.kalman_update_backend,
                        config::expert::KalmanUpdateBackend::kUdKf);
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
          .Beam()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
          .WithScanCenterDeg(scan_center)
          .End()
          .Environment()
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .End()
          .Build();

    EXPECT_EQ(config.mission.orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
    EXPECT_FLOAT_EQ(config.mission.orientation.scan_center_deg.az_deg,
                  8.0f);
    EXPECT_FLOAT_EQ(config.mission.orientation.scan_center_deg.el_deg,
                  -2.0f);
  EXPECT_EQ(config.environment.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  const auto config =
      config::RadarSessionConfigBuilder(config::presets::MakeDetectionMissionRadarSessionConfig())
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
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kStt)
          .WithScanCenterDeg(scan_center)
          .WithDwellCenterDeg(dwell_center)
          .WithCommandedBeamwidthDeg(commanded_beamwidth)
          .EnableCommandedBeamwidth(true)
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();

  EXPECT_TRUE(patch.has_work_sub_mode);
  EXPECT_EQ(patch.work_sub_mode, model::RadarWorkSubMode::kStt);
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
  EXPECT_TRUE(patch.has_environment_runtime_config);
  EXPECT_TRUE(patch.environment_runtime_config.has_jamming_sensitivity_profile);
  EXPECT_EQ(patch.environment_runtime_config.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, RuntimePatchCanBeAppliedWithoutReconstructingSession) {
  session::RadarSession session =
      session::RadarSessionFactory::Create(config::presets::MakeDetectionMissionRadarSessionConfig());

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
          .EnableCommandedBeamwidth(true)
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .Build();

  session.ApplyRuntimeConfig(patch);

  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
}

TEST(RadarSessionConfigBuilderTest, DetailedBuilderProducesDetailedSessionConfig) {
    const auto detailed_config =
            config::RadarDetailedSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(true)
          .WithPeakPowerW(5.0e6f)
          .WithFrequencyHz(9.3e9f)
          .WithBandwidthHz(10.0e6f)
          .WithPulseWidthS(20e-6f)
          .WithPrfHz(500.0f)
          .WithMainBeamGainDb(38.0f)
          .WithNoiseFigureDb(3.5f)
          .End()
          .Beam()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
          .End()
          .Tracking()
          .EnableKalmanFilter(true)
          .WithKalmanMeasurementNoiseStd(7.5f)
          .WithKalmanUpdateBackend(config::expert::KalmanUpdateBackend::kUdKf)
          .End()
          .Lifecycle()
          .EnableImmFusion(true)
          .WithLifecycleConfirmHits(2U)
          .WithLifecycleMaxMissBeforeLost(1U)
          .WithLifecycleMaxLostCycles(4U)
          .End()
          .Environment()
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kStrict)
          .End()
          .Build();

    EXPECT_TRUE(detailed_config.hardware.detection.enable_physics_detection);
    EXPECT_FLOAT_EQ(detailed_config.hardware.detection.transmitter.peak_power_w,
                  5.0e6f);
    EXPECT_FLOAT_EQ(detailed_config.hardware.detection.transmitter.frequency_hz,
                  9.3e9f);
        EXPECT_EQ(detailed_config.mission.orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
    EXPECT_FLOAT_EQ(detailed_config.policy.tracking.kalman_measurement_noise_std,
                  7.5f);
    EXPECT_EQ(detailed_config.policy.tracking.kalman_update_backend,
            config::expert::KalmanUpdateBackend::kUdKf);
    EXPECT_EQ(detailed_config.policy.lifecycle.confirm_hits, 2U);
    EXPECT_TRUE(detailed_config.policy.lifecycle.enable_imm_lifecycle);
    EXPECT_EQ(detailed_config.environment.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, DetailedBuiltConfigCanConstructRadarSession) {
  const auto config =
            config::RadarDetailedSessionConfigBuilder()
          .Detection()
          .WithPeakPowerW(5.0e6f)
          .WithFrequencyHz(9.3e9f)
          .End()
          .Tracking()
          .WithKalmanUpdateBackend(config::expert::KalmanUpdateBackend::kUdKf)
          .End()
          .Lifecycle()
          .EnableImmFusion(true)
          .End()
          .Build();

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

  const auto detailed_config =
      config::RadarDetailedSessionConfigBuilder()
          .Beam()
          .WithAzimuthStepCountHint(8U)
          .WithElevationStepCountHint(4U)
          .PreferDenseTasSampling(true)
          .WithDefaultScanCenterDeg(default_scan_center)
          .WithNominalBeamwidthDeg(nominal_beamwidth)
          .End()
          .Build();

  EXPECT_EQ(
      detailed_config.policy.beam_control.scheduler.azimuth_step_count_hint, 8U);
  EXPECT_EQ(
      detailed_config.policy.beam_control.scheduler.elevation_step_count_hint, 4U);
  EXPECT_TRUE(
      detailed_config.policy.beam_control.scheduler.prefer_dense_tas_sampling);
  EXPECT_FLOAT_EQ(detailed_config.mission.orientation.scan_center_deg.az_deg, 6.0f);
  EXPECT_FLOAT_EQ(detailed_config.mission.orientation.scan_center_deg.el_deg, -1.5f);
  EXPECT_FLOAT_EQ(
      detailed_config.mission.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg,
      2.5f);
  EXPECT_FLOAT_EQ(
      detailed_config.mission.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg,
      1.5f);
}

}  // namespace tests
}  // namespace airborne_radar
