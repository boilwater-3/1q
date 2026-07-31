// @file sar_profile_constants_test.cpp
// @brief 验证 SarProfileConstants 常量字段值与旧 Builder 翻译输出一致（迁移锚点）。

#include <gtest/gtest.h>

#include "1q/sar/config/SarProfileConstants.h"

namespace sar {
namespace config {
namespace tests {

TEST(SarProfileConstantsTest, HighResolutionImagingMission) {
  const auto& m = profiles::kHighResolutionImagingMission;
  EXPECT_DOUBLE_EQ(m.nominal_slant_range_m, 10000.0);
  EXPECT_DOUBLE_EQ(m.platform_speed_mps, 150.0);
  EXPECT_EQ(m.azimuth_pulse_count, 2048U);
  EXPECT_EQ(m.range_sample_count, 4096U);
  EXPECT_DOUBLE_EQ(m.desired_ground_range_resolution_m, 0.5);
  EXPECT_DOUBLE_EQ(m.desired_azimuth_resolution_m, 0.5);
}

TEST(SarProfileConstantsTest, LongRangeSurveillanceMission) {
  const auto& m = profiles::kLongRangeSurveillanceMission;
  EXPECT_DOUBLE_EQ(m.nominal_slant_range_m, 50000.0);
  EXPECT_DOUBLE_EQ(m.platform_speed_mps, 200.0);
  EXPECT_EQ(m.azimuth_pulse_count, 512U);
  EXPECT_EQ(m.range_sample_count, 1024U);
  EXPECT_DOUBLE_EQ(m.desired_ground_range_resolution_m, 3.0);
  EXPECT_DOUBLE_EQ(m.desired_azimuth_resolution_m, 3.0);
}

TEST(SarProfileConstantsTest, RawEchoOnlyProcessing) {
  const auto& p = profiles::kRawEchoOnlyProcessing;
  EXPECT_TRUE(p.enable_raw_echo_generation);
  EXPECT_FALSE(p.enable_l1_rda_imaging);
  EXPECT_FALSE(p.enable_l2_motion_compensation);
  EXPECT_FALSE(p.enable_l3_bp_imaging);
  EXPECT_FALSE(p.retain_focused_image);
}

TEST(SarProfileConstantsTest, RangeCompressedL1Processing) {
  const auto& p = profiles::kRangeCompressedL1Processing;
  EXPECT_TRUE(p.enable_raw_echo_generation);
  EXPECT_TRUE(p.enable_l1_rda_imaging);
  EXPECT_FALSE(p.enable_l2_motion_compensation);
  EXPECT_FALSE(p.enable_l3_bp_imaging);
  EXPECT_TRUE(p.retain_focused_image);
}

TEST(SarProfileConstantsTest, L3BackprojectionProcessing) {
  const auto& p = profiles::kL3BackprojectionProcessing;
  EXPECT_TRUE(p.enable_raw_echo_generation);
  EXPECT_FALSE(p.enable_l1_rda_imaging);
  EXPECT_FALSE(p.enable_l2_motion_compensation);
  EXPECT_TRUE(p.enable_l3_bp_imaging);
  EXPECT_TRUE(p.retain_focused_image);
}

}  // namespace tests
}  // namespace config
}  // namespace sar
