#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "1q/sar/config/SarProfileConstants.h"
#include "1q/sar/config/SarSessionConfigBuilder.h"
#include "1q/sar/config/SarSessionConfigValidation.h"
#include "1q/sar/session/SarSession.h"
#include "sar/session/SarRuntimeConfigValidation.h"

namespace sar {
namespace config {
namespace {

TEST(SarSessionConfigBuilderTest, DefaultBuildKeepsConfigDefaults) {
  SarSessionConfigBuilder builder;
  const SarSessionConfig config = builder.Build();

  // 未设置任何语义常量，应保持 struct 默认值。
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 1024U);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
  EXPECT_FALSE(config.policy.enable_l1_rda_imaging);
}

TEST(SarSessionConfigBuilderTest, MissionConstantsTranslateFields) {
  SarSessionConfig config;
  config.mission = profiles::kHighResolutionImagingMission;

  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 10000.0);
  EXPECT_DOUBLE_EQ(config.mission.platform_speed_mps, 150.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 2048U);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
  EXPECT_DOUBLE_EQ(config.mission.desired_ground_range_resolution_m, 0.5);
  EXPECT_DOUBLE_EQ(config.mission.desired_azimuth_resolution_m, 0.5);
}

TEST(SarSessionConfigBuilderTest, LongRangeSurveillanceConstants) {
  SarSessionConfig config;
  config.mission = profiles::kLongRangeSurveillanceMission;

  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 50000.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 512U);
}

TEST(SarSessionConfigBuilderTest, ProcessingConstantsRawEchoOnly) {
  SarSessionConfig config;
  config.policy = profiles::kRawEchoOnlyProcessing;

  EXPECT_TRUE(config.policy.enable_raw_echo_generation);
  EXPECT_FALSE(config.policy.enable_l1_rda_imaging);
  EXPECT_FALSE(config.policy.enable_l3_bp_imaging);
  EXPECT_FALSE(config.policy.retain_focused_image);
}

TEST(SarSessionConfigBuilderTest, ProcessingConstantsL3Backprojection) {
  SarSessionConfig config;
  config.policy = profiles::kL3BackprojectionProcessing;

  EXPECT_TRUE(config.policy.enable_raw_echo_generation);
  EXPECT_FALSE(config.policy.enable_l1_rda_imaging);
  EXPECT_FALSE(config.policy.enable_l2_motion_compensation);
  EXPECT_TRUE(config.policy.enable_l3_bp_imaging);
  EXPECT_TRUE(config.policy.retain_focused_image);
}

TEST(SarSessionConfigBuilderTest, ProcessingConstantsL3HasNoL1L2Conflict) {
  SarSessionConfig config;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy = profiles::kL3BackprojectionProcessing;

  session::SarCycleResult result;
  EXPECT_TRUE(session::ValidateRuntimeConfigForStep(
      config, /*has_external_raw_iq=*/true, &result))
      << result.abort_reason;
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
  SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 9.6e9;
  config.mission.scene_center_latitude_deg = 40.0;
  config.policy.minimum_snr_db = 6.5;
  config.environment.atmospheric_loss_db_per_km = 0.02;
  // kStripmapSurvey 与 struct 默认一致（no-op 档位），不赋常量即保持基线。

  // 基线字段保留。
  EXPECT_DOUBLE_EQ(config.hardware.carrier_frequency_hz, 9.6e9);
  EXPECT_DOUBLE_EQ(config.mission.scene_center_latitude_deg, 40.0);
  // struct 默认即条带档位翻译值（no-op 档位，逐字段锁定防漂移）。
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_DOUBLE_EQ(config.mission.platform_speed_mps, 180.0);
  EXPECT_EQ(config.mission.azimuth_pulse_count, 1024U);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
  EXPECT_DOUBLE_EQ(config.mission.desired_ground_range_resolution_m, 1.5);
  EXPECT_DOUBLE_EQ(config.mission.desired_azimuth_resolution_m, 1.5);
  // 非档位字段仍由四域 config 直接负责。
  EXPECT_DOUBLE_EQ(config.policy.minimum_snr_db, 6.5);
  EXPECT_DOUBLE_EQ(config.environment.atmospheric_loss_db_per_km, 0.02);
}

TEST(SarSessionConfigBuilderTest, ProcessingConstantsDoNotOwnMinimumSnr) {
  // 档位在前、微调在后：常量只含 5 个处理开关字段，minimum_snr_db 由显式赋值决定。
  SarSessionConfig config;
  config.policy = profiles::kL3BackprojectionProcessing;
  config.policy.minimum_snr_db = 7.25;

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
  // struct 默认（= 条带档位）即为合法配置。
  const ValidationIssueList issues = ValidateSarSessionConfig(SarSessionConfig{});
  EXPECT_TRUE(issues.empty());
}

TEST(SarSessionConfigValidationTest, RejectsRawHistoryWithoutRawEcho) {
  SarSessionConfig config;
  config.policy.enable_raw_echo_generation = false;
  config.policy.retain_raw_phase_history = true;
  const ValidationIssueList issues = ValidateSarSessionConfig(config);
  ASSERT_FALSE(issues.empty());
  EXPECT_TRUE(std::any_of(issues.begin(), issues.end(), [](const ConfigValidationIssue& issue) {
    return issue.code == ConfigValidationCode::kRetainRawHistoryRequiresRawEcho;
  }));
}

TEST(SarSessionConfigValidationTest, RejectsInvalidSquintLimit) {
  for (double value : {-1.0, 90.0, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    SarSessionConfig config;
    config.policy.max_allowed_squint_angle_deg = value;
    const ValidationIssueList issues = ValidateSarSessionConfig(config);
    EXPECT_TRUE(std::any_of(issues.begin(), issues.end(), [](const ConfigValidationIssue& issue) {
      return issue.code == ConfigValidationCode::kSquintAngleInvalid;
    }));
  }
}

TEST(SarSessionConfigValidationTest, RejectsInvalidHardwareLinkBudget) {
  SarSessionConfig config;
  config.hardware.peak_power_w = 0.0;
  config.hardware.receiver_noise_figure_db = -1.0;
  const ValidationIssueList issues = ValidateSarSessionConfig(config);
  EXPECT_TRUE(std::any_of(issues.begin(), issues.end(), [](const ConfigValidationIssue& issue) {
    return issue.code == ConfigValidationCode::kHardwareLinkBudgetInvalid;
  }));
}

TEST(SarSessionConfigValidationTest, RejectsInvalidEnvironmentScalars) {
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    SarSessionConfig config;
    config.environment.terrain_reference_altitude_m = value;
    const ValidationIssueList issues = ValidateSarSessionConfig(config);
    EXPECT_TRUE(std::any_of(issues.begin(), issues.end(), [](const ConfigValidationIssue& issue) {
      return issue.code == ConfigValidationCode::kEnvironmentConfigInvalid;
    }));
  }

  SarSessionConfig negative_loss;
  negative_loss.environment.atmospheric_loss_db_per_km = -0.01;
  const ValidationIssueList negative_loss_issues = ValidateSarSessionConfig(negative_loss);
  EXPECT_TRUE(std::any_of(negative_loss_issues.begin(), negative_loss_issues.end(),
                          [](const ConfigValidationIssue& issue) {
                            return issue.code == ConfigValidationCode::kEnvironmentConfigInvalid;
                          }));

  SarSessionConfig invalid_sigma0;
  invalid_sigma0.environment.surface_backscatter_sigma0_db =
      std::numeric_limits<double>::quiet_NaN();
  const ValidationIssueList sigma0_issues = ValidateSarSessionConfig(invalid_sigma0);
  EXPECT_TRUE(std::any_of(sigma0_issues.begin(), sigma0_issues.end(),
                          [](const ConfigValidationIssue& issue) {
                            return issue.code == ConfigValidationCode::kEnvironmentConfigInvalid;
                          }));
}

}  // namespace

TEST(SarSessionCreateWithValidationTest, BuildsSessionAndReportsNoIssuesForHealthyConfig) {
  const SarSessionConfig config;  // struct 默认即条带档位，合法。

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
