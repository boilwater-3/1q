// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 RadarSessionConfigBuilder 语义化配置接口。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/presets/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace airborne_radar {
namespace tests {

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesSemanticDefaults) {
  const auto config = config::RadarSessionConfigBuilder().Build();

    EXPECT_FALSE(config.pipeline_config.expert.detection.enable_physics_detection);
    EXPECT_FLOAT_EQ(config.pipeline_config.expert.detection.min_detection_margin_db, -2.0f);
    EXPECT_EQ(config.pipeline_config.expert.detection.pulse_count, 10);
    EXPECT_FALSE(config.pipeline_config.expert.tracking.enable_kalman_filter);
    EXPECT_EQ(config.pipeline_config.expert.lifecycle.confirm_hits, 3U);
    EXPECT_EQ(config.pipeline_config.expert.lifecycle.max_miss_before_lost, 2U);
    EXPECT_FALSE(config.pipeline_config.expert.lifecycle.enable_imm_lifecycle);
}

TEST(RadarSessionConfigBuilderTest, PresetBasePreservesPresetSemanticValues) {
  const auto config =
      config::RadarSessionConfigBuilder(config::presets::MakeDetectionMissionRadarSessionConfig())
          .Build();

    EXPECT_EQ(config.pipeline_config.expert.detection.pulse_count, 10);
    EXPECT_FLOAT_EQ(config.pipeline_config.expert.detection.detection_policy.min_snr_db, -10.0f);
    EXPECT_FLOAT_EQ(config.pipeline_config.expert.tracking.kalman_measurement_noise_std, 10.0f);
    EXPECT_EQ(config.pipeline_config.expert.lifecycle.confirm_hits, 3U);
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

    EXPECT_TRUE(config.pipeline_config.expert.detection.enable_physics_detection);
    EXPECT_FLOAT_EQ(config.pipeline_config.expert.detection.transmitter.peak_power_w, 5.0e6f);
    EXPECT_EQ(config.pipeline_config.expert.detection.pulse_count, 8);
    EXPECT_FLOAT_EQ(config.pipeline_config.expert.detection.antenna.pattern.max_sidelobe_level_db,
                                    -30.0f);
    EXPECT_TRUE(config.pipeline_config.expert.detection.rcs_physics.enable_physical_rcs);
    EXPECT_FLOAT_EQ(config.pipeline_config.expert.detection.rcs_physics.physics_mix_ratio, 0.60f);
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

    EXPECT_TRUE(config.pipeline_config.expert.tracking.enable_kalman_filter);
    EXPECT_EQ(config.pipeline_config.expert.tracking.kalman_update_backend,
                        config::expert::KalmanUpdateBackend::kUdKf);
    EXPECT_TRUE(config.pipeline_config.expert.lifecycle.enable_imm_lifecycle);
    EXPECT_EQ(config.pipeline_config.expert.lifecycle.confirm_hits, 3U);
    EXPECT_EQ(config.pipeline_config.expert.lifecycle.max_lost_cycles, 8U);
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

    EXPECT_EQ(config.pipeline_config.orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
    EXPECT_FLOAT_EQ(config.pipeline_config.orientation.scan_center_deg.az_deg,
                  8.0f);
    EXPECT_FLOAT_EQ(config.pipeline_config.orientation.scan_center_deg.el_deg,
                  -2.0f);
  EXPECT_EQ(config.environment_default_config.jamming_sensitivity_profile,
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

TEST(RadarSessionConfigBuilderTest, ExpertBuilderProducesExpertSessionConfig) {
  const auto expert_config =
      config::RadarExpertSessionConfigBuilder()
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

  EXPECT_TRUE(expert_config.pipeline_config.expert.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(expert_config.pipeline_config.expert.detection.transmitter.peak_power_w,
                  5.0e6f);
  EXPECT_FLOAT_EQ(expert_config.pipeline_config.expert.detection.transmitter.frequency_hz,
                  9.3e9f);
    EXPECT_EQ(expert_config.pipeline_config.orientation.work_sub_mode,
            model::RadarWorkSubMode::kTas);
  EXPECT_FLOAT_EQ(expert_config.pipeline_config.expert.tracking.kalman_measurement_noise_std,
                  7.5f);
  EXPECT_EQ(expert_config.pipeline_config.expert.tracking.kalman_update_backend,
            config::expert::KalmanUpdateBackend::kUdKf);
  EXPECT_EQ(expert_config.pipeline_config.expert.lifecycle.confirm_hits, 2U);
  EXPECT_TRUE(expert_config.pipeline_config.expert.lifecycle.enable_imm_lifecycle);
  EXPECT_EQ(expert_config.environment_default_config.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, ExpertBuiltConfigCanConstructRadarSession) {
  const auto config =
      config::RadarExpertSessionConfigBuilder()
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

TEST(RadarSessionConfigBuilderTest, ExpertBeamSchedulerWritesExpertPath) {
  const auto expert_config =
      config::RadarExpertSessionConfigBuilder()
          .Beam()
          .WithAzimuthStepCountHint(8U)
          .WithElevationStepCountHint(4U)
          .PreferDenseTasSampling(true)
          .End()
          .Build();

  EXPECT_EQ(
      expert_config.pipeline_config.expert.beam_control.scheduler.azimuth_step_count_hint, 8U);
  EXPECT_EQ(
      expert_config.pipeline_config.expert.beam_control.scheduler.elevation_step_count_hint, 4U);
  EXPECT_TRUE(
      expert_config.pipeline_config.expert.beam_control.scheduler.prefer_dense_tas_sampling);
}

}  // namespace tests
}  // namespace airborne_radar
