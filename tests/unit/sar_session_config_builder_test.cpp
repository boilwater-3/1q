#include <gtest/gtest.h>

#include "1q/sar/config/SarSessionConfigBuilder.h"

namespace sar {
namespace config {
namespace {

TEST(SarSessionConfigBuilderTest, DefaultBuildKeepsConfigDefaults) {
  SarSessionConfigBuilder builder;
  const SarSessionConfig config = builder.Build();

  // 未设置任何 Profile，应保持 struct 默认值。
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 1024U);
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

TEST(SarSessionConfigBuilderTest, SceneCenterViaEditor) {
  SarSessionConfigBuilder builder;
  builder.Mission().WithSceneCenter(35.0, 120.0, 500.0).End();
  const SarSessionConfig config = builder.Build();

  EXPECT_DOUBLE_EQ(config.mission.scene_center_latitude_deg, 35.0);
  EXPECT_DOUBLE_EQ(config.mission.scene_center_longitude_deg, 120.0);
  EXPECT_DOUBLE_EQ(config.mission.scene_center_altitude_m, 500.0);
}

TEST(SarSessionConfigBuilderTest, WithSessionConfigBaseline) {
  SarSessionConfig baseline;
  baseline.hardware.carrier_frequency_hz = 9.6e9;
  baseline.mission.scene_center_latitude_deg = 40.0;

  SarSessionConfigBuilder builder(baseline);
  builder.Mission().WithMissionProfile(SarMissionProfile::kStripmapSurvey).End();
  const SarSessionConfig config = builder.Build();

  // 基线字段保留。
  EXPECT_DOUBLE_EQ(config.hardware.carrier_frequency_hz, 9.6e9);
  EXPECT_DOUBLE_EQ(config.mission.scene_center_latitude_deg, 40.0);
  // Profile 翻译的字段应用。
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 1024U);
}

TEST(SarSessionConfigBuilderTest, ValidateDetectsNonPositiveFrequency) {
  SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 0.0;
  config.hardware.bandwidth_hz = 0.0;

  SarSessionConfigBuilder builder(config);
  const ValidationIssueList issues = builder.Validate();

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

TEST(SarSessionConfigBuilderTest, ValidatePassesOnHealthyConfig) {
  SarSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(SarMissionProfile::kStripmapSurvey).End();
  const ValidationIssueList issues = builder.Validate();
  EXPECT_TRUE(issues.empty());
}

}  // namespace
}  // namespace config
}  // namespace sar
