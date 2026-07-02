#include <gtest/gtest.h>

#include "1q/sar/config/SarSessionConfigBuilder.h"
#include "1q/sar/config/SarSessionConfigValidation.h"
#include "1q/sar/session/SarSession.h"

namespace sar {
namespace config {
namespace {

TEST(SarSessionConfigBuilderTest, DefaultBuildKeepsConfigDefaults) {
  SarSessionConfigBuilder builder;
  const SarSessionConfig config = builder.Build();

  // 未设置任何 Profile，应保持 struct 默认值。
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 1024U);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
  EXPECT_FALSE(config.policy.enable_l1_rda_imaging);
}

TEST(SarSessionConfigBuilderTest, MissionProfileTranslatesFields) {
  SarSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(SarMissionProfile::kHighResolutionImaging).End();
  const SarSessionConfig config = builder.Build();

  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 10000.0);
  EXPECT_DOUBLE_EQ(config.mission.platform_speed_mps, 150.0);
  EXPECT_DOUBLE_EQ(config.mission.synthetic_aperture_time_s, 4.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 2048U);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
  EXPECT_DOUBLE_EQ(config.mission.desired_ground_range_resolution_m, 0.5);
  EXPECT_DOUBLE_EQ(config.mission.desired_azimuth_resolution_m, 0.5);
}

TEST(SarSessionConfigBuilderTest, LongRangeSurveillanceProfile) {
  SarSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(SarMissionProfile::kLongRangeSurveillance).End();
  const SarSessionConfig config = builder.Build();

  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 50000.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 512U);
}

TEST(SarSessionConfigBuilderTest, ProcessingProfileRawEchoOnly) {
  SarSessionConfigBuilder builder;
  builder.Processing().WithProcessingProfile(SarProcessingProfile::kRawEchoOnly).End();
  const SarSessionConfig config = builder.Build();

  EXPECT_TRUE(config.policy.enable_raw_echo_generation);
  EXPECT_FALSE(config.policy.enable_range_compression);
  EXPECT_FALSE(config.policy.enable_l1_rda_imaging);
  EXPECT_FALSE(config.policy.enable_l3_bp_imaging);
  EXPECT_FALSE(config.policy.retain_focused_image);
}

TEST(SarSessionConfigBuilderTest, ProcessingProfileFullPipelineL3) {
  SarSessionConfigBuilder builder;
  builder.Processing().WithProcessingProfile(SarProcessingProfile::kFullPipelineL3).End();
  const SarSessionConfig config = builder.Build();

  EXPECT_TRUE(config.policy.enable_raw_echo_generation);
  EXPECT_TRUE(config.policy.enable_range_compression);
  EXPECT_TRUE(config.policy.enable_l2_motion_compensation);
  EXPECT_TRUE(config.policy.enable_l3_bp_imaging);
  EXPECT_TRUE(config.policy.retain_focused_image);
}

TEST(SarSessionConfigBuilderTest, DirectConfigOwnsSceneCenterFields) {
  SarSessionConfig config;
  config.mission.scene_center_latitude_deg = 35.0;
  config.mission.scene_center_longitude_deg = 120.0;
  config.mission.scene_center_altitude_m = 500.0;

  EXPECT_DOUBLE_EQ(config.mission.scene_center_latitude_deg, 35.0);
  EXPECT_DOUBLE_EQ(config.mission.scene_center_longitude_deg, 120.0);
  EXPECT_DOUBLE_EQ(config.mission.scene_center_altitude_m, 500.0);
}

TEST(SarSessionConfigBuilderTest, ProfileOverlayPreservesUnrelatedBaselineFields) {
  SarSessionConfig baseline;
  baseline.hardware.carrier_frequency_hz = 9.6e9;
  baseline.mission.scene_center_latitude_deg = 40.0;
  baseline.policy.minimum_snr_db = 6.5;
  baseline.environment.atmospheric_loss_db_per_km = 0.02;

  SarSessionConfigBuilder builder(baseline);
  builder.Mission().WithMissionProfile(SarMissionProfile::kStripmapSurvey).End();
  const SarSessionConfig config = builder.Build();

  // 基线字段保留。
  EXPECT_DOUBLE_EQ(config.hardware.carrier_frequency_hz, 9.6e9);
  EXPECT_DOUBLE_EQ(config.mission.scene_center_latitude_deg, 40.0);
  // Profile 翻译的字段应用。
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 1024U);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
  // 非 profile 字段仍由四域 config 直接负责。
  EXPECT_DOUBLE_EQ(config.policy.minimum_snr_db, 6.5);
  EXPECT_DOUBLE_EQ(config.environment.atmospheric_loss_db_per_km, 0.02);
}

TEST(SarSessionConfigBuilderTest, ProcessingProfileDoesNotOwnMinimumSnr) {
  SarSessionConfig baseline;
  baseline.policy.minimum_snr_db = 7.25;

  SarSessionConfigBuilder builder(baseline);
  builder.Processing().WithProcessingProfile(SarProcessingProfile::kFullPipelineL3).End();
  const SarSessionConfig config = builder.Build();

  EXPECT_TRUE(config.policy.enable_l3_bp_imaging);
  EXPECT_TRUE(config.policy.retain_focused_image);
  EXPECT_DOUBLE_EQ(config.policy.minimum_snr_db, 7.25);
}

TEST(SarSessionConfigValidationTest, DetectsNonPositiveFrequency) {
  SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 0.0;
  config.hardware.bandwidth_hz = 0.0;

  const ValidationIssueList issues = ValidateSarSessionConfig(config);

  EXPECT_FALSE(issues.empty());
  bool found_frequency = false;
  bool found_bandwidth = false;
  for (const ConfigValidationIssue& issue : issues) {
    if (issue.code == ConfigValidationCode::kCarrierFrequencyNotPositive) {
      found_frequency = true;
    }
    if (issue.code == ConfigValidationCode::kBandwidthNotPositive) {
      found_bandwidth = true;
    }
  }
  EXPECT_TRUE(found_frequency);
  EXPECT_TRUE(found_bandwidth);
}

TEST(SarSessionConfigValidationTest, PassesOnHealthyBuiltConfig) {
  SarSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(SarMissionProfile::kStripmapSurvey).End();
  const ValidationIssueList issues = ValidateSarSessionConfig(builder.Build());
  EXPECT_TRUE(issues.empty());
}

}  // namespace

TEST(SarSessionCreateWithValidationTest, BuildsSessionAndReportsNoIssuesForHealthyConfig) {
  SarSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(SarMissionProfile::kStripmapSurvey).End();
  const SarSessionConfig config = builder.Build();

  ValidationIssueList issues;
  const session::SarSession session = session::SarSession::CreateWithValidation(config, &issues);

  EXPECT_TRUE(issues.empty());
  (void)session;
}

TEST(SarSessionCreateWithValidationTest, ReportsIssuesButStillConstructsSession) {
  SarSessionConfig invalid;
  invalid.hardware.carrier_frequency_hz = 0.0;

  ValidationIssueList issues;
  const session::SarSession session = session::SarSession::CreateWithValidation(invalid, &issues);

  EXPECT_FALSE(issues.empty());
  EXPECT_EQ(issues.front().code, ConfigValidationCode::kCarrierFrequencyNotPositive);
  (void)session;  // 会话仍被构造，调用方据 issues 决策
}

TEST(SarSessionCreateWithValidationTest, AcceptsNullIssuesWithoutCrash) {
  SarSessionConfig invalid;
  invalid.hardware.carrier_frequency_hz = 0.0;

  const session::SarSession session = session::SarSession::CreateWithValidation(invalid, nullptr);
  (void)session;  // nullptr 时仅构造，不写回 issues
}

}  // namespace config
}  // namespace sar
