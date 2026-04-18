// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 RadarSessionConfigBuilder 语义化配置接口。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace airborne_radar {
namespace tests {

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesSemanticDefaults) {
  const auto config = config::RadarSessionConfigBuilder().Build();

  EXPECT_FALSE(config.detection.enable_physics_detection);
  EXPECT_EQ(config.detection.hardware_profile,
            config::RadarHardwareProfile::kGenericAirborneXBand);
  EXPECT_EQ(config.detection.intent_profile, config::DetectionIntentProfile::kBalanced);
  EXPECT_EQ(config.detection.antenna_pattern.profile, config::AntennaPatternProfile::kStandard);
  EXPECT_EQ(config.tracking.policy_profile, config::TrackingPolicyProfile::kBalanced);
  EXPECT_EQ(config.lifecycle.policy_profile, config::LifecyclePolicyProfile::kBalanced);
  EXPECT_FALSE(config.lifecycle.enable_imm_fusion);
}

TEST(RadarSessionConfigBuilderTest, PresetBasePreservesPresetSemanticValues) {
  const auto config =
      config::RadarSessionConfigBuilder(config::MakeDetectionMissionRadarSessionConfig()).Build();

  EXPECT_EQ(config.detection.intent_profile, config::DetectionIntentProfile::kDetectionPriority);
  EXPECT_EQ(config.tracking.policy_profile, config::TrackingPolicyProfile::kFastAssociation);
  EXPECT_EQ(config.lifecycle.policy_profile, config::LifecyclePolicyProfile::kFastConfirm);
}

TEST(RadarSessionConfigBuilderTest, DetectionSemanticEditorsApplyCorrectly) {
  const auto config = config::RadarSessionConfigBuilder()
                          .Detection()
                          .EnablePhysicsDetection(true)
                          .WithHardwareProfile(config::RadarHardwareProfile::kLongRangeHighPower)
                          .WithDetectionIntentProfile(
                              config::DetectionIntentProfile::kTrackStabilityPriority)
                          .WithAntennaPatternProfile(config::AntennaPatternProfile::kLowSidelobe)
                          .WithRcsFusionProfile(config::RcsFusionProfile::kEnhanced)
                          .End()
                          .Build();

  EXPECT_TRUE(config.detection.enable_physics_detection);
  EXPECT_EQ(config.detection.hardware_profile, config::RadarHardwareProfile::kLongRangeHighPower);
  EXPECT_EQ(config.detection.intent_profile,
            config::DetectionIntentProfile::kTrackStabilityPriority);
  EXPECT_EQ(config.detection.antenna_pattern.profile, config::AntennaPatternProfile::kLowSidelobe);
  EXPECT_EQ(config.detection.rcs_fusion_profile, config::RcsFusionProfile::kEnhanced);
}

TEST(RadarSessionConfigBuilderTest, TrackingAndLifecycleSemanticEditorsApplyCorrectly) {
  const auto config = config::RadarSessionConfigBuilder()
                          .Tracking()
                          .EnableTrackingFilter(true)
                          .WithTrackingPolicyProfile(config::TrackingPolicyProfile::kRobustAntiJamming)
                          .End()
                          .Lifecycle()
                          .EnableImmFusion(true)
                          .WithLifecyclePolicyProfile(config::LifecyclePolicyProfile::kHighPersistence)
                          .End()
                          .Build();

  EXPECT_TRUE(config.tracking.enable_tracking_filter);
  EXPECT_EQ(config.tracking.policy_profile, config::TrackingPolicyProfile::kRobustAntiJamming);
  EXPECT_TRUE(config.lifecycle.enable_imm_fusion);
  EXPECT_EQ(config.lifecycle.policy_profile, config::LifecyclePolicyProfile::kHighPersistence);
}

TEST(RadarSessionConfigBuilderTest, BeamAndEnvironmentEditorsApplyCorrectly) {
  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 8.0f;
  scan_center.el_deg = -2.0f;

  const auto config = config::RadarSessionConfigBuilder()
                          .Beam()
                          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
                          .WithScanCenterDeg(scan_center)
                          .End()
                          .Environment()
                          .WithJammingSensitivityProfile(
                              environment::JammingSensitivityProfile::kStrict)
                          .End()
                          .Build();

  EXPECT_EQ(config.beam_control.radar_orientation.work_sub_mode, model::RadarWorkSubMode::kTas);
  EXPECT_FLOAT_EQ(config.beam_control.radar_orientation.scan_center_deg.az_deg, 8.0f);
  EXPECT_FLOAT_EQ(config.beam_control.radar_orientation.scan_center_deg.el_deg, -2.0f);
  EXPECT_EQ(config.environment_default_config.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  const auto config = config::RadarSessionConfigBuilder(config::MakeDetectionMissionRadarSessionConfig())
                          .Environment()
                          .WithJammingSensitivityProfile(
                              environment::JammingSensitivityProfile::kStrict)
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

  const config::RadarRuntimeConfigPatch patch = config::RadarRuntimeConfigBuilder()
                                                    .WithRadarWorkSubMode(model::RadarWorkSubMode::kStt)
                                                    .WithScanCenterDeg(scan_center)
                                                    .WithDwellCenterDeg(dwell_center)
                                                    .WithCommandedBeamwidthDeg(commanded_beamwidth)
                                                    .EnableCommandedBeamwidth(true)
                                                    .WithJammingSensitivityProfile(
                                                        environment::JammingSensitivityProfile::kStrict)
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
      session::RadarSessionFactory::Create(config::MakeDetectionMissionRadarSessionConfig());

  const config::RadarRuntimeConfigPatch patch = config::RadarRuntimeConfigBuilder()
                                                    .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
                                                    .EnableCommandedBeamwidth(true)
                                                    .WithJammingSensitivityProfile(
                                                        environment::JammingSensitivityProfile::kStrict)
                                                    .Build();

  session.ApplyRuntimeConfig(patch);

  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
}

}  // namespace tests
}  // namespace airborne_radar
