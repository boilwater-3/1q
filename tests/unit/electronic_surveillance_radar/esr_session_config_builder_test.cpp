/**
 * @file esr_session_config_builder_test.cpp
 * @brief 验证 ESR ProfileConstants 常量赋值与配置校验。
 */

#include <gtest/gtest.h>

#include <limits>

#include "1q/electronic_surveillance_radar/config/EsrProfileConstants.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigValidation.h"

namespace electronic_surveillance_radar {
namespace config {
namespace {

bool ContainsCode(const ValidationIssueList& issues, ConfigValidationCode code) {
  for (const auto& issue : issues) {
    if (issue.code == code) return true;
  }
  return false;
}

// =============================================================================
// 语义档位常量赋值（等价迁移自旧 Profile 翻译）
// =============================================================================

TEST(EsrSessionConfigBuilderTest, ElectronicOrderOfBattleConstantsAssign) {
  EsrSessionConfig config;
  config.mission = profiles::kElectronicOrderOfBattleMission;
  EXPECT_GT(config.mission.scan.scan_rate_hz, 0.0f);
}

TEST(EsrSessionConfigBuilderTest, PrecisionEmitterAnalysisConstantsAssign) {
  EsrSessionConfig config;
  config.mission = profiles::kPrecisionEmitterAnalysisMission;
  EXPECT_GT(config.mission.scan.scan_rate_hz, 0.0f);
}

TEST(EsrSessionConfigBuilderTest, ThreatWarningConstantsAssign) {
  EsrSessionConfig config;
  config.mission = profiles::kThreatWarningMission;
  EXPECT_GT(config.mission.scan.scan_rate_hz, 0.0f);
}

TEST(EsrSessionConfigBuilderTest, SensitivityConstantsApplyDetectionDefaults) {
  // kHighSensitivity / kRobust 有有效覆盖。
  EsrSessionConfig config;
  config.policy.detection = profiles::kHighSensitivityDetection;
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 3.0f);
  config.policy.detection = profiles::kRobustDetection;
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 10.0f);
  // kStandard 与 struct 默认逐字段一致（no-op 档位），精确锁定防漂移。
  const EsrDetectionPolicyConfig default_detection{};
  EXPECT_FLOAT_EQ(default_detection.minimum_snr_db, 6.0f);
  EXPECT_EQ(default_detection.pulse_count, 8U);
  EXPECT_FLOAT_EQ(default_detection.pfa, 1.0e-6f);
  EXPECT_FLOAT_EQ(default_detection.threshold_scale, 1.0f);
  EXPECT_TRUE(default_detection.enable_statistical_detection);
}

// =============================================================================
// ValidateEsrSessionConfig 校验分支
// =============================================================================

TEST(EsrSessionConfigValidationTest, RejectsNonPositiveScanRate) {
  EsrSessionConfig config;
  config.mission.scan.scan_rate_hz = -1.0f;
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kScanRateNotPositive));
}

TEST(EsrSessionConfigValidationTest, RejectsNonFiniteScanRate) {
  EsrSessionConfig config;
  config.mission.scan.scan_rate_hz = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kScanRateNotPositive));
  config.mission.scan.scan_rate_hz = std::numeric_limits<float>::infinity();
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kScanRateNotPositive));
}

TEST(EsrSessionConfigValidationTest, RejectsReceiverBandLowerAboveUpper) {
  EsrSessionConfig config;
  config.mission.scan.scan_rate_hz = 2.0f;
  config.hardware.receiver_band_lower_hz = 2.0e9;
  config.hardware.receiver_band_upper_hz = 1.0e9;
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kReceiverBandLowerAboveUpper));
}

TEST(EsrSessionConfigValidationTest, RejectsNonPositiveBeamAzWidth) {
  EsrSessionConfig config;
  config.mission.scan.scan_rate_hz = 2.0f;
  config.hardware.beam_az_width_deg = 0.0f;
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kBeamAzWidthNotPositive));
}

TEST(EsrSessionConfigValidationTest, RejectsNonPositiveBeamElWidth) {
  EsrSessionConfig config;
  config.mission.scan.scan_rate_hz = 2.0f;
  config.hardware.beam_el_width_deg = -1.0f;
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kBeamElWidthNotPositive));
}

TEST(EsrSessionConfigValidationTest, RejectsExplicitScanBoundsAzSwapped) {
  EsrSessionConfig config;
  config.mission.scan.scan_rate_hz = 2.0f;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = 60.0f;
  config.mission.scan.scan_end_az_deg = -60.0f;
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kExplicitScanBoundsAzSwapped));
}

TEST(EsrSessionConfigValidationTest, RejectsExplicitScanBoundsElSwapped) {
  EsrSessionConfig config;
  config.mission.scan.scan_rate_hz = 2.0f;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_el_deg = 10.0f;
  config.mission.scan.scan_end_el_deg = -10.0f;
  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kExplicitScanBoundsElSwapped));
}

TEST(EsrSessionConfigValidationTest, RejectsNonFiniteExplicitScanBounds) {
  EsrSessionConfig config;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg =
      std::numeric_limits<float>::quiet_NaN();

  EXPECT_TRUE(ContainsCode(ValidateEsrSessionConfig(config),
                           ConfigValidationCode::kExplicitScanBoundsNotFinite));
}

TEST(EsrSessionConfigValidationTest, IgnoresExplicitFieldsWhenCenterModeIsSelected) {
  EsrSessionConfig config;
  config.mission.scan.use_explicit_scan_bounds = false;
  config.mission.scan.scan_start_az_deg =
      std::numeric_limits<float>::quiet_NaN();
  config.mission.scan.scan_end_el_deg =
      std::numeric_limits<float>::infinity();

  EXPECT_TRUE(ValidateEsrSessionConfig(config).empty());
}

TEST(EsrSessionConfigValidationTest, PassesOnHealthyBuiltConfig) {
  EsrSessionConfig config;
  config.mission = profiles::kElectronicOrderOfBattleMission;
  const ValidationIssueList issues = ValidateEsrSessionConfig(config);
  EXPECT_TRUE(issues.empty());
}

TEST(EsrSessionConfigValidationTest, RejectsUnknownEnumsAndInvalidDomains) {
  EsrSessionConfig config;
  config.mission.work_mode = static_cast<EsrWorkMode>(99);
  config.mission.scan.scan_sequence = static_cast<EsrScanSequence>(99);
  config.policy.detection.pfa = 1.0f;
  config.environment.scenario_config.preset =
      static_cast<EsrEnvironmentPreset>(99);

  const ValidationIssueList issues = ValidateEsrSessionConfig(config);
  EXPECT_TRUE(ContainsCode(issues, ConfigValidationCode::kMissionEnumInvalid));
  EXPECT_TRUE(
      ContainsCode(issues, ConfigValidationCode::kDetectionPolicyInvalid));
  EXPECT_TRUE(ContainsCode(issues, ConfigValidationCode::kEnvironmentInvalid));
}

TEST(EsrSessionConfigValidationTest,
     RejectsEnabledInvalidAtmosphereAndCenterAngles) {
  EsrSessionConfig config;
  config.mission.scan.use_explicit_scan_bounds = false;
  config.mission.scan.scan_center_az_deg =
      std::numeric_limits<float>::quiet_NaN();
  config.environment.scenario_config.atmospheric_physics
      .enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.relative_humidity =
      1.5f;

  const ValidationIssueList issues = ValidateEsrSessionConfig(config);
  EXPECT_TRUE(ContainsCode(issues, ConfigValidationCode::kScanCenterNotFinite));
  EXPECT_TRUE(ContainsCode(issues, ConfigValidationCode::kEnvironmentInvalid));
}

}  // namespace
}  // namespace config
}  // namespace electronic_surveillance_radar
