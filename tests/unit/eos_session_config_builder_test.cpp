/**
 * @file eos_session_config_builder_test.cpp
 * @brief 验证 EOS SessionConfigBuilder 语义档位与配置校验（此前 5.6% 覆盖）。
 */

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigValidation.h"

namespace electro_optical_sensor {
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

TEST(EosSessionConfigBuilderTest, WideAreaSearchProfileSetsDefaults) {
  EosSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EosMissionProfile::kWideAreaSearch).End();
  const EosSessionConfig config = builder.Build();
  EXPECT_GT(config.mission.scan_rate_deg_per_sec, 0.0f);
}

TEST(EosSessionConfigBuilderTest, LongRangeSurveillanceProfileSetsDefaults) {
  EosSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EosMissionProfile::kLongRangeSurveillance).End();
  const EosSessionConfig config = builder.Build();
  EXPECT_GT(config.mission.scan_rate_deg_per_sec, 0.0f);
}

TEST(EosSessionConfigBuilderTest, HighResolutionTrackProfileSetsDefaults) {
  EosSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EosMissionProfile::kHighResolutionTrack).End();
  const EosSessionConfig config = builder.Build();
  EXPECT_GT(config.mission.scan_rate_deg_per_sec, 0.0f);
}

TEST(EosSessionConfigBuilderTest, HardwareProfilesApplyDefaults) {
  for (auto profile : {EosHardwareProfile::kStandardMidWaveIR,
                       EosHardwareProfile::kLongRangeLargeAperture,
                       EosHardwareProfile::kWideAreaCompact}) {
    EosSessionConfigBuilder builder;
    builder.Hardware().WithHardwareProfile(profile).End();
    const EosSessionConfig config = builder.Build();
    EXPECT_GT(config.mission.scan_rate_deg_per_sec, 0.0f);
  }
}

// =============================================================================
// ValidateEosSessionConfig 校验分支
// =============================================================================

TEST(EosSessionConfigValidationTest, RejectsNonPositiveHorizontalFov) {
  EosSessionConfig config;
  config.mission.horizontal_fov_deg = 0.0f;
  EXPECT_TRUE(ContainsCode(ValidateEosSessionConfig(config),
                           ConfigValidationCode::kHorizontalFovNotPositive));
}

TEST(EosSessionConfigValidationTest, RejectsNonPositiveVerticalFov) {
  EosSessionConfig config;
  config.mission.vertical_fov_deg = -1.0f;
  EXPECT_TRUE(ContainsCode(ValidateEosSessionConfig(config),
                           ConfigValidationCode::kVerticalFovNotPositive));
}

TEST(EosSessionConfigValidationTest, RejectsNonPositiveScanRate) {
  EosSessionConfig config;
  config.mission.scan_rate_deg_per_sec = -1.0f;
  EXPECT_TRUE(ContainsCode(ValidateEosSessionConfig(config),
                           ConfigValidationCode::kScanRateNotPositive));
}

TEST(EosSessionConfigValidationTest, RejectsNonPositiveFrameRate) {
  EosSessionConfig config;
  config.mission.frame_rate_hz = 0.0f;
  EXPECT_TRUE(ContainsCode(ValidateEosSessionConfig(config),
                           ConfigValidationCode::kFrameRateNotPositive));
}

TEST(EosSessionConfigValidationTest, RejectsScanRangeAzSwapped) {
  EosSessionConfig config;
  config.mission.scan_start_az_deg = 60.0f;
  config.mission.scan_end_az_deg = -60.0f;
  EXPECT_TRUE(ContainsCode(ValidateEosSessionConfig(config),
                           ConfigValidationCode::kScanRangeAzSwapped));
}

TEST(EosSessionConfigValidationTest, PassesOnHealthyBuiltConfig) {
  EosSessionConfigBuilder builder;
  builder.Mission().WithMissionProfile(EosMissionProfile::kWideAreaSearch).End();
  const ValidationIssueList issues = ValidateEosSessionConfig(builder.Build());
  EXPECT_TRUE(issues.empty());
}

}  // namespace
}  // namespace config
}  // namespace electro_optical_sensor
