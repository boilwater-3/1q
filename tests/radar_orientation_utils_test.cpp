// Copyright 2026. All Rights Reserved.
//
// @file radar_orientation_utils_test.cpp
// @brief 验证机载雷达方向配置工具函数的组合与限幅行为。

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/RadarOrientationUtils.h"

namespace airborne_radar {
namespace tests {

using common::AzimuthElevationDeg;
using common::AzimuthElevationLimitsDeg;
using common::ComputeBodyFrameBeamPointing;
using common::ComputeMountFrameBeamPointing;
using common::ComputePlatformFrameBeamPointing;
using common::EulerAnglesDeg;
using common::IntersectScanLimits;
using common::IsValidScanLimits;
using common::RadarOrientationConfig;

TEST(RadarOrientationUtilsTest, IsValidScanLimitsAcceptsOrderedBounds) {
  AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -60.0f;
  limits.az_max_deg = 60.0f;
  limits.el_min_deg = -20.0f;
  limits.el_max_deg = 20.0f;
  EXPECT_TRUE(IsValidScanLimits(limits));
}

TEST(RadarOrientationUtilsTest, IsValidScanLimitsRejectsReversedBounds) {
  AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = 30.0f;
  limits.az_max_deg = -30.0f;
  limits.el_min_deg = -20.0f;
  limits.el_max_deg = 20.0f;
  EXPECT_FALSE(IsValidScanLimits(limits));
}

TEST(RadarOrientationUtilsTest, IntersectScanLimitsReturnsOverlapWindow) {
  AzimuthElevationLimitsDeg mechanical;
  mechanical.az_min_deg = -60.0f;
  mechanical.az_max_deg = 60.0f;
  mechanical.el_min_deg = -30.0f;
  mechanical.el_max_deg = 30.0f;

  AzimuthElevationLimitsDeg electronic;
  electronic.az_min_deg = -45.0f;
  electronic.az_max_deg = 50.0f;
  electronic.el_min_deg = -10.0f;
  electronic.el_max_deg = 20.0f;

  const AzimuthElevationLimitsDeg limits =
      IntersectScanLimits(mechanical, electronic);

  EXPECT_FLOAT_EQ(limits.az_min_deg, -45.0f);
  EXPECT_FLOAT_EQ(limits.az_max_deg, 50.0f);
  EXPECT_FLOAT_EQ(limits.el_min_deg, -10.0f);
  EXPECT_FLOAT_EQ(limits.el_max_deg, 20.0f);
}

TEST(RadarOrientationUtilsTest, ComputeMountFrameBeamPointingClampsToOverlap) {
  RadarOrientationConfig config;
  config.scan_center_deg.az_deg = 30.0f;
  config.scan_center_deg.el_deg = 5.0f;
  config.dwell_center_deg.az_deg = 25.0f;
  config.dwell_center_deg.el_deg = -25.0f;
  config.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  config.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  config.mechanical_scan_limits_deg.el_min_deg = -30.0f;
  config.mechanical_scan_limits_deg.el_max_deg = 30.0f;
  config.electronic_scan_limits_deg.az_min_deg = -45.0f;
  config.electronic_scan_limits_deg.az_max_deg = 45.0f;
  config.electronic_scan_limits_deg.el_min_deg = -15.0f;
  config.electronic_scan_limits_deg.el_max_deg = 15.0f;

  const AzimuthElevationDeg pointing =
      ComputeMountFrameBeamPointing(config);

  EXPECT_FLOAT_EQ(pointing.az_deg, 45.0f);
  EXPECT_FLOAT_EQ(pointing.el_deg, -15.0f);
}

TEST(RadarOrientationUtilsTest, ComputeBodyFrameBeamPointingAddsMountOffset) {
  RadarOrientationConfig config;
  config.mount_angles_deg.yaw_deg = 10.0f;
  config.mount_angles_deg.pitch_deg = 5.0f;
  config.mount_angles_deg.roll_deg = 2.0f;
  config.scan_center_deg.az_deg = 20.0f;
  config.scan_center_deg.el_deg = -10.0f;
  config.dwell_center_deg.az_deg = 15.0f;
  config.dwell_center_deg.el_deg = 5.0f;
  config.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  config.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  config.mechanical_scan_limits_deg.el_min_deg = -30.0f;
  config.mechanical_scan_limits_deg.el_max_deg = 30.0f;
  config.electronic_scan_limits_deg.az_min_deg = -60.0f;
  config.electronic_scan_limits_deg.az_max_deg = 60.0f;
  config.electronic_scan_limits_deg.el_min_deg = -30.0f;
  config.electronic_scan_limits_deg.el_max_deg = 30.0f;

  const EulerAnglesDeg pointing = ComputeBodyFrameBeamPointing(config);

  EXPECT_FLOAT_EQ(pointing.yaw_deg, 45.0f);
  EXPECT_FLOAT_EQ(pointing.pitch_deg, 0.0f);
  EXPECT_FLOAT_EQ(pointing.roll_deg, 2.0f);
}

TEST(RadarOrientationUtilsTest,
     ComputePlatformFrameBeamPointingAddsPlatformAttitude) {
  RadarOrientationConfig config;
  config.mount_angles_deg.yaw_deg = 10.0f;
  config.mount_angles_deg.pitch_deg = 5.0f;
  config.mount_angles_deg.roll_deg = 2.0f;
  config.scan_center_deg.az_deg = 20.0f;
  config.scan_center_deg.el_deg = -10.0f;
  config.dwell_center_deg.az_deg = 15.0f;
  config.dwell_center_deg.el_deg = 5.0f;
  config.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  config.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  config.mechanical_scan_limits_deg.el_min_deg = -30.0f;
  config.mechanical_scan_limits_deg.el_max_deg = 30.0f;
  config.electronic_scan_limits_deg.az_min_deg = -60.0f;
  config.electronic_scan_limits_deg.az_max_deg = 60.0f;
  config.electronic_scan_limits_deg.el_min_deg = -30.0f;
  config.electronic_scan_limits_deg.el_max_deg = 30.0f;

  EulerAnglesDeg platform_attitude;
  platform_attitude.yaw_deg = 90.0f;
  platform_attitude.pitch_deg = 3.0f;
  platform_attitude.roll_deg = 1.0f;
  const EulerAnglesDeg pointing =
      ComputePlatformFrameBeamPointing(platform_attitude, config);

  EXPECT_FLOAT_EQ(pointing.yaw_deg, 135.0f);
  EXPECT_FLOAT_EQ(pointing.pitch_deg, 3.0f);
  EXPECT_FLOAT_EQ(pointing.roll_deg, 3.0f);
}

} // namespace tests
} // namespace airborne_radar
