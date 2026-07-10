/**
 * @file esr_session_config_builder_test.cpp
 * @brief 验证 ESR SessionConfigBuilder 语义档位与配置校验（此前 0% 覆盖）。
 */

#include <gtest/gtest.h>

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
// Build() 语义档位（3-way switch 全覆盖）
// =============================================================================

TEST(EsrSessionConfigBuilderTest, ElectronicOrderOfBattleProfileSetsDefaults) {
  EsrSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EsrMissionProfile::kElectronicOrderOfBattle).End();
  const EsrSessionConfig config = builder.Build();
  EXPECT_GT(config.mission.scan.scan_rate_hz, 0.0f);
}

TEST(EsrSessionConfigBuilderTest, PrecisionEmitterAnalysisProfileSetsDefaults) {
  EsrSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EsrMissionProfile::kPrecisionEmitterAnalysis).End();
  const EsrSessionConfig config = builder.Build();
  EXPECT_GT(config.mission.scan.scan_rate_hz, 0.0f);
}

TEST(EsrSessionConfigBuilderTest, ThreatWarningProfileSetsDefaults) {
  EsrSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EsrMissionProfile::kThreatWarning).End();
  const EsrSessionConfig config = builder.Build();
  EXPECT_GT(config.mission.scan.scan_rate_hz, 0.0f);
}

TEST(EsrSessionConfigBuilderTest, SensitivityProfilesApplyDetectionDefaults) {
  for (auto profile : {EsrSensitivityProfile::kStandard,
                       EsrSensitivityProfile::kHighSensitivity,
                       EsrSensitivityProfile::kRobust}) {
    EsrSessionConfigBuilder builder;
    builder.Detection().WithSensitivityProfile(profile).End();
    const EsrSessionConfig config = builder.Build();
    EXPECT_GE(config.policy.detection.minimum_snr_db, 0.0f);
  }
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

TEST(EsrSessionConfigValidationTest, PassesOnHealthyBuiltConfig) {
  EsrSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EsrMissionProfile::kElectronicOrderOfBattle).End();
  const ValidationIssueList issues = ValidateEsrSessionConfig(builder.Build());
  EXPECT_TRUE(issues.empty());
}

}  // namespace
}  // namespace config
}  // namespace electronic_surveillance_radar
