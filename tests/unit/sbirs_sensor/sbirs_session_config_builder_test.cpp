#include <gtest/gtest.h>

#include <limits>

#include "1q/sbirs_sensor/config/SbirsSessionConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace {

// 统一问题列表模型（规则 14）：检查校验问题列表中是否包含指定 code（机器消费字段）。
bool ContainsCode(const sbirs_sensor::session::SbirsIssueList& issues, const std::string& code) {
  for (const sbirs_sensor::session::SbirsIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

// 按 code 取条目（Q-2 审查修复：phase/severity 断言辅助）。
const sbirs_sensor::session::SbirsIssue* FindIssue(
    const sbirs_sensor::session::SbirsIssueList& issues, const std::string& code) {
  for (const sbirs_sensor::session::SbirsIssue& issue : issues) {
    if (issue.code == code) {
      return &issue;
    }
  }
  return nullptr;
}

TEST(SbirsSessionConfigBuilderTest, BuildsFourDomainConfiguration) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.optical_aperture_m = 0.7f;
  sbirs_sensor::config::SbirsMissionConfig mission;
  mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  mission.scan_rate_deg_per_sec = 2.0f;
  sbirs_sensor::config::SbirsPolicyConfig policy;
  policy.detection.wide_min_snr_linear = 3.0f;
  sbirs_sensor::config::SbirsEnvironmentConfig environment;
  environment.weather_type = sbirs_sensor::config::SbirsWeatherType::kCloudy;

  const sbirs_sensor::config::SbirsSessionConfig config =
      sbirs_sensor::config::SbirsSessionConfigBuilder()
          .WithHardware(hardware)
          .WithMission(mission)
          .WithPolicy(policy)
          .WithEnvironment(environment)
          .Build();

  EXPECT_FLOAT_EQ(config.hardware.optical_aperture_m, 0.7f);
  EXPECT_EQ(config.mission.work_mode, sbirs_sensor::config::SbirsWorkMode::kSearchAndStare);
  EXPECT_FLOAT_EQ(config.mission.scan_rate_deg_per_sec, 2.0f);
  EXPECT_FLOAT_EQ(config.policy.detection.wide_min_snr_linear, 3.0f);
  EXPECT_EQ(config.environment.weather_type, sbirs_sensor::config::SbirsWeatherType::kCloudy);
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigBuilderTest, RejectsInvalidScanRate) {
  sbirs_sensor::config::SbirsMissionConfig mission;
  mission.scan_rate_deg_per_sec = -1.0f;
  const sbirs_sensor::config::SbirsSessionConfig config =
      sbirs_sensor::config::SbirsSessionConfigBuilder().WithMission(mission).Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::config::ValidateSbirsSessionConfig(config);
  // config 域校验问题统一 phase=kInputValidation + severity=kError（规则 14 config 域；
  // HasValidationError 依赖 phase 判定，防误改时拒绝语义静默翻转）。
  const sbirs_sensor::session::SbirsIssue* issue =
      FindIssue(issues, "sbirs.validation.invalid_scan_rate");
  ASSERT_NE(issue, nullptr);
  EXPECT_EQ(issue->phase, sbirs_sensor::session::SbirsIssuePhase::kInputValidation);
  EXPECT_EQ(issue->severity, sbirs_sensor::session::SbirsIssueSeverity::kError);
}

TEST(SbirsSessionConfigBuilderTest, ValidatesCircularScanContract) {
  // ECI 方位约定（2026-08 正式变更）：scan_start ∈ [0, 360)；180 合法、
  // −180 与 360 非法（旧对称约定 [-180,180) 已废除）。
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.scan_start_az_deg = 180.0f;
  config.mission.scan_span_deg = 360.0f;
  config.mission.scan_direction = sbirs_sensor::config::SbirsScanDirection::kDecreasingAzimuth;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.scan_start_az_deg = -180.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_start_azimuth"));
  config.mission.scan_start_az_deg = 360.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_start_azimuth"));
  config.mission.scan_start_az_deg = 180.0f;
  config.mission.scan_span_deg = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_span"));
  config.mission.scan_span_deg = 360.1f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_span"));
  config.mission.scan_span_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_span"));
  config.mission.scan_span_deg = 120.0f;
  config.mission.scan_direction = static_cast<sbirs_sensor::config::SbirsScanDirection>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_scan_direction"));
}

TEST(SbirsSessionConfigBuilderTest, PointingDefaultsAreProductionValues) {
  const sbirs_sensor::config::SbirsMissionConfig mission;
  const sbirs_sensor::config::SbirsTrackingConfig tracking;

  EXPECT_FLOAT_EQ(mission.narrow_pointing_max_slew_rate_deg_per_sec, 30.0f);
  EXPECT_FLOAT_EQ(mission.narrow_pointing_settle_tolerance_deg, 0.01f);
  EXPECT_EQ(tracking.nfov_tracking_gate_loss_cycles, 2U);
  EXPECT_EQ(tracking.tracking_mode, sbirs_sensor::config::SbirsTrackingMode::kEstimated);
  EXPECT_EQ(tracking.estimated_backend,
            sbirs_sensor::config::SbirsEstimatedTrackingBackend::kEkf);
  const sbirs_sensor::config::SbirsPointingDisturbanceConfig disturbance;
  EXPECT_FLOAT_EQ(disturbance.common_attitude_sigma_deg, 0.0f);
  EXPECT_FLOAT_EQ(disturbance.channel_pointing_sigma_deg, 0.0f);
  EXPECT_FLOAT_EQ(disturbance.channel_vibration_amplitude_deg, 0.0f);
}

TEST(SbirsSessionConfigBuilderTest, RejectsUnknownTrackingEnums) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.tracking.tracking_mode =
      static_cast<sbirs_sensor::config::SbirsTrackingMode>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_tracking_mode"));

  config.policy.tracking.tracking_mode = sbirs_sensor::config::SbirsTrackingMode::kEstimated;
  config.policy.tracking.estimated_backend =
      static_cast<sbirs_sensor::config::SbirsEstimatedTrackingBackend>(99);
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_estimated_tracking_backend"));
}

TEST(SbirsSessionConfigBuilderTest, RejectsInvalidPointingParameters) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_slew_rate"));

  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = std::numeric_limits<float>::infinity();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_slew_rate"));

  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 30.0f;
  config.mission.narrow_pointing_settle_tolerance_deg = -0.01f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_settle_tolerance"));

  config.mission.narrow_pointing_settle_tolerance_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_narrow_pointing_settle_tolerance"));
}

TEST(SbirsSessionConfigBuilderTest, RejectsZeroTrackingGateLossCycles) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.tracking.nfov_tracking_gate_loss_cycles = 0U;

  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_tracking_gate_loss_cycles"));
}

TEST(SbirsSessionConfigBuilderTest, ValidatesPointingDisturbanceParameters) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.pointing_disturbance.common_attitude_sigma_deg = -0.1f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_values"));

  config.policy.pointing_disturbance.common_attitude_sigma_deg = 0.0f;
  config.policy.pointing_disturbance.common_attitude_correlation_time_s = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_correlation"));

  config.policy.pointing_disturbance.common_attitude_correlation_time_s = 1.0f;
  config.policy.pointing_disturbance.channel_pointing_sigma_deg =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_values"));

  config.policy.pointing_disturbance.channel_pointing_sigma_deg = 0.0f;
  config.policy.pointing_disturbance.channel_pointing_correlation_time_s = -1.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_correlation"));

  config.policy.pointing_disturbance.channel_pointing_correlation_time_s = 1.0f;
  config.policy.pointing_disturbance.channel_vibration_amplitude_deg = 0.1f;
  config.policy.pointing_disturbance.channel_vibration_frequency_hz = 0.0f;
  EXPECT_TRUE(ContainsCode(sbirs_sensor::config::ValidateSbirsSessionConfig(config),
                           "sbirs.validation.invalid_pointing_disturbance_vibration_frequency"));

  config.policy.pointing_disturbance.channel_vibration_frequency_hz = 2.0f;
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

}  // namespace
