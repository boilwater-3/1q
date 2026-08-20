// @file esr_profile_constants_test.cpp
// @brief 验证 EsrProfileConstants 常量字段值与旧 Builder 翻译输出一致（迁移锚点）。

#include <gtest/gtest.h>

#include "1q/electronic_surveillance_radar/config/EsrProfileConstants.h"

namespace electronic_surveillance_radar {
namespace config {
namespace tests {

TEST(EsrProfileConstantsTest, ElectronicOrderOfBattleMission) {
  const auto& m = profiles::kElectronicOrderOfBattleMission;
  EXPECT_EQ(m.work_mode, config::EsrWorkMode::kEsm);
  EXPECT_FLOAT_EQ(m.scan.scan_rate_hz, 2.0f);
  EXPECT_TRUE(m.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(m.scan.scan_start_az_deg, -60.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_end_az_deg, 60.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_start_el_deg, -10.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_end_el_deg, 10.0f);
}

TEST(EsrProfileConstantsTest, PrecisionEmitterAnalysisMission) {
  const auto& m = profiles::kPrecisionEmitterAnalysisMission;
  EXPECT_EQ(m.work_mode, config::EsrWorkMode::kHgesm);
  EXPECT_FLOAT_EQ(m.scan.scan_rate_hz, 0.5f);
  EXPECT_TRUE(m.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(m.scan.scan_start_az_deg, -30.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_end_az_deg, 30.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_start_el_deg, -5.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_end_el_deg, 5.0f);
}

TEST(EsrProfileConstantsTest, ThreatWarningMission) {
  const auto& m = profiles::kThreatWarningMission;
  EXPECT_EQ(m.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_FLOAT_EQ(m.scan.scan_rate_hz, 5.0f);
  EXPECT_TRUE(m.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(m.scan.scan_start_az_deg, -60.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_end_az_deg, 60.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_start_el_deg, -10.0f);
  EXPECT_FLOAT_EQ(m.scan.scan_end_el_deg, 10.0f);
}

TEST(EsrProfileConstantsTest, HighSensitivityDetection) {
  const auto& c = profiles::kHighSensitivityDetection;
  EXPECT_FLOAT_EQ(c.minimum_snr_db, 3.0f);
  EXPECT_EQ(c.pulse_count, 16U);
  EXPECT_FLOAT_EQ(c.pfa, 5.0e-6f);
  EXPECT_FLOAT_EQ(c.threshold_scale, 1.0f);
  EXPECT_TRUE(c.enable_statistical_detection);
}

TEST(EsrProfileConstantsTest, RobustDetection) {
  const auto& c = profiles::kRobustDetection;
  EXPECT_FLOAT_EQ(c.minimum_snr_db, 10.0f);
  EXPECT_EQ(c.pulse_count, 4U);
  EXPECT_FLOAT_EQ(c.pfa, 1.0e-7f);
  EXPECT_FLOAT_EQ(c.threshold_scale, 1.0f);
  EXPECT_TRUE(c.enable_statistical_detection);
}

}  // namespace tests
}  // namespace config
}  // namespace electronic_surveillance_radar
