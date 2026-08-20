// @file eos_profile_constants_test.cpp
// @brief 验证 EosProfileConstants 常量字段值与旧 Builder 翻译输出一致（迁移锚点）。

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/config/EosProfileConstants.h"

namespace electro_optical_sensor {
namespace config {
namespace tests {

TEST(EosProfileConstantsTest, WideAreaSearchMission) {
  const auto& m = profiles::kWideAreaSearchMission;
  EXPECT_EQ(m.work_mode, config::EosWorkMode::kFused);
  EXPECT_FLOAT_EQ(m.horizontal_fov_deg, 12.0f);
  EXPECT_FLOAT_EQ(m.vertical_fov_deg, 8.0f);
  EXPECT_FLOAT_EQ(m.scan_rate_deg_per_sec, 30.0f);
  EXPECT_FLOAT_EQ(m.frame_rate_hz, 15.0f);
}

TEST(EosProfileConstantsTest, LongRangeSurveillanceMission) {
  const auto& m = profiles::kLongRangeSurveillanceMission;
  EXPECT_EQ(m.work_mode, config::EosWorkMode::kInfraredOnly);
  EXPECT_FLOAT_EQ(m.horizontal_fov_deg, 3.0f);
  EXPECT_FLOAT_EQ(m.vertical_fov_deg, 2.0f);
  EXPECT_FLOAT_EQ(m.scan_rate_deg_per_sec, 10.0f);
  EXPECT_FLOAT_EQ(m.frame_rate_hz, 10.0f);
}

TEST(EosProfileConstantsTest, HighResolutionTrackMission) {
  const auto& m = profiles::kHighResolutionTrackMission;
  EXPECT_EQ(m.work_mode, config::EosWorkMode::kFused);
  EXPECT_FLOAT_EQ(m.horizontal_fov_deg, 1.5f);
  EXPECT_FLOAT_EQ(m.vertical_fov_deg, 1.0f);
  EXPECT_FLOAT_EQ(m.scan_rate_deg_per_sec, 5.0f);
  EXPECT_FLOAT_EQ(m.frame_rate_hz, 60.0f);
}

TEST(EosProfileConstantsTest, MissionDetectionThresholds) {
  // kWideAreaSearch 的 snr=6dB 与 struct 默认一致（no-op，不提供常量）。
  EXPECT_FLOAT_EQ(EosDetectionPolicyConfig{}.minimum_snr_db, 6.0f);
  EXPECT_FLOAT_EQ(profiles::kLongRangeSurveillanceDetection.minimum_snr_db, 3.0f);
  EXPECT_FLOAT_EQ(profiles::kHighResolutionTrackDetection.minimum_snr_db, 2.0f);
}

TEST(EosProfileConstantsTest, LongRangeLargeApertureHardware) {
  const auto& h = profiles::kLongRangeLargeApertureHardware;
  EXPECT_FLOAT_EQ(h.wavelength_lower_um, 3.0f);
  EXPECT_FLOAT_EQ(h.wavelength_upper_um, 5.0f);
  EXPECT_FLOAT_EQ(h.optical_aperture_m, 0.4f);
  EXPECT_FLOAT_EQ(h.detector_detectivity_cm_sqrt_hz_per_w, 2.0e10f);
}

TEST(EosProfileConstantsTest, WideAreaCompactHardware) {
  const auto& h = profiles::kWideAreaCompactHardware;
  EXPECT_FLOAT_EQ(h.wavelength_lower_um, 8.0f);
  EXPECT_FLOAT_EQ(h.wavelength_upper_um, 12.0f);
  EXPECT_FLOAT_EQ(h.optical_aperture_m, 0.1f);
  EXPECT_FLOAT_EQ(h.detector_detectivity_cm_sqrt_hz_per_w, 5.0e9f);
}

}  // namespace tests
}  // namespace config
}  // namespace electro_optical_sensor
