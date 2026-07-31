// @file eos_session_config_builder_test.cpp
// @brief 验证 EOS ProfileConstants 常量赋值与配置校验。

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/config/EosProfileConstants.h"
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
// 语义档位常量赋值（等价迁移自旧 Profile 翻译）
// =============================================================================

TEST(EosSessionConfigBuilderTest, WideAreaSearchConstantsAssign) {
  EosSessionConfig config;
  config.mission = profiles::kWideAreaSearchMission;
  // snr=6dB 与 struct 默认一致（no-op 档位），无需赋值。
  EXPECT_GT(config.mission.scan_rate_deg_per_sec, 0.0f);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 6.0f);
}

TEST(EosSessionConfigBuilderTest, LongRangeSurveillanceConstantsAssign) {
  EosSessionConfig config;
  config.mission = profiles::kLongRangeSurveillanceMission;
  config.policy.detection = profiles::kLongRangeSurveillanceDetection;
  EXPECT_GT(config.mission.scan_rate_deg_per_sec, 0.0f);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 3.0f);
}

TEST(EosSessionConfigBuilderTest, HighResolutionTrackConstantsAssign) {
  EosSessionConfig config;
  config.mission = profiles::kHighResolutionTrackMission;
  config.policy.detection = profiles::kHighResolutionTrackDetection;
  EXPECT_GT(config.mission.scan_rate_deg_per_sec, 0.0f);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 2.0f);
}

TEST(EosSessionConfigBuilderTest, HardwareConstantsAssign) {
  EosSessionConfig config;
  config.hardware = profiles::kLongRangeLargeApertureHardware;
  EXPECT_FLOAT_EQ(config.hardware.optical_aperture_m, 0.4f);
  config.hardware = profiles::kWideAreaCompactHardware;
  EXPECT_FLOAT_EQ(config.hardware.wavelength_lower_um, 8.0f);
  // kStandardMidWaveIR 与 struct 默认逐字段一致（no-op 档位），精确锁定防漂移。
  const EosHardwareConfig default_hardware{};
  EXPECT_FLOAT_EQ(default_hardware.wavelength_lower_um, 3.0f);
  EXPECT_FLOAT_EQ(default_hardware.wavelength_upper_um, 5.0f);
  EXPECT_FLOAT_EQ(default_hardware.optical_aperture_m, 0.2f);
  EXPECT_FLOAT_EQ(default_hardware.detector_detectivity_cm_sqrt_hz_per_w, 1.0e10f);
}

// =============================================================================
// 直接字段赋值即最终决定（显式顺序语义，取代旧 Profile 隐式覆写）
// =============================================================================

TEST(EosSessionConfigBuilderTest, DirectSnrAssignmentIsFinal) {
  // 档位在前、微调在后：微调胜出。
  EosSessionConfig config;
  config.mission = profiles::kWideAreaSearchMission;  // 档位 snr 默认 6dB
  config.policy.detection.minimum_snr_db = 9.0f;      // 直接赋值
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 9.0f);
}

TEST(EosSessionConfigBuilderTest, AssignmentOrderDeterminesResult) {
  // 微调在前、档位在后：整域赋值覆写（C++ 拷贝语义，无隐式优先级）。
  EosSessionConfig config;
  config.policy.detection.minimum_snr_db = 9.0f;
  config.policy.detection = profiles::kLongRangeSurveillanceDetection;  // snr=3dB
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 3.0f);
}

TEST(EosSessionConfigBuilderTest, SnrSurvivesWithoutProfileAssignment) {
  EosSessionConfig config;
  config.policy.detection.minimum_snr_db = 7.5f;
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 7.5f);
}

TEST(EosSessionConfigBuilderTest, MissionConstantsDoNotTouchDetectionPolicy) {
  // Mission 档位不再跨域覆写 policy.detection（旧 Profile 隐式覆写已消除）。
  EosSessionConfig config;
  config.mission = profiles::kWideAreaSearchMission;
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 6.0f);  // struct 默认，未被触碰
  config.mission = profiles::kLongRangeSurveillanceMission;
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 6.0f);
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
  EosSessionConfig config;
  config.mission = profiles::kWideAreaSearchMission;
  const ValidationIssueList issues = ValidateEosSessionConfig(config);
  EXPECT_TRUE(issues.empty());
}

TEST(EosSessionConfigValidationTest, RejectsInvalidEnvironmentEnums) {
  EosSessionConfig config;
  config.environment.scenario_config.preset = static_cast<EosEnvironmentPreset>(99);

  const ValidationIssueList issues = ValidateEosSessionConfig(config);
  EXPECT_TRUE(ContainsCode(issues, ConfigValidationCode::kEnvironmentPresetInvalid));
}

TEST(EosSessionConfigValidationTest, RejectsInvalidEnabledAtmosphericPhysics) {
  EosSessionConfig config;
  config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.temperature_k = 0.0f;

  const ValidationIssueList issues = ValidateEosSessionConfig(config);
  EXPECT_TRUE(ContainsCode(issues, ConfigValidationCode::kAtmosphericPhysicsInvalid));
}

}  // namespace
}  // namespace config
}  // namespace electro_optical_sensor
