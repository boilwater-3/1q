// Copyright 2026. All Rights Reserved.
//
// @file radar_orientation_utils_test.cpp
// @brief 验证机载雷达方向配置工具函数的组合与限幅行为。

#include <gtest/gtest.h>

#include <cmath>

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "airborne_radar/common/utils/RadarOrientationUtils.h"
#include "1q/airborne_radar/config/SignalDetectionConfig.h"
#include "common/geometry/GeometryTransform.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"

namespace airborne_radar {
namespace tests {

using model::AzimuthElevationDeg;
using model::AzimuthElevationLimitsDeg;
using model::ComputeBodyFrameBeamPointing;
using model::ComputeMountFrameBeamPointing;
using model::ComputePlatformFrameBeamPointing;
using model::EulerAnglesDeg;
using model::IntersectScanLimits;
using model::IsValidScanLimits;
using model::RadarOrientationConfig;
using model::StabilizationMode;
using config::AntennaConfig;
using signal::detection::BeamControlResolver;
using signal::detection::ResolvedBeamState;
using signal::detection::TargetLookAnglesDeg;

oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(const EulerAnglesDeg& euler_deg) {
  oneq::internal::geometry::EulerAnglesDeg geometry_euler;
  geometry_euler.yaw_deg = euler_deg.yaw_deg;
  geometry_euler.pitch_deg = euler_deg.pitch_deg;
  geometry_euler.roll_deg = euler_deg.roll_deg;
  return geometry_euler;
}

void ExpectMatricesNear(const Eigen::Matrix3f& expected, const Eigen::Matrix3f& actual,
                        float tolerance) {
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_NEAR(expected(row, column), actual(row, column), tolerance);
    }
  }
}

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

  const AzimuthElevationLimitsDeg limits = IntersectScanLimits(mechanical, electronic);

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

  const AzimuthElevationDeg pointing = ComputeMountFrameBeamPointing(config);

  EXPECT_FLOAT_EQ(pointing.az_deg, 45.0f);
  EXPECT_FLOAT_EQ(pointing.el_deg, -15.0f);
}

TEST(RadarOrientationUtilsTest, ComputeBodyFrameBeamPointingUsesRotationComposition) {
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
  const AzimuthElevationDeg mount_frame_pointing = ComputeMountFrameBeamPointing(config);
  EulerAnglesDeg mount_frame_euler;
  mount_frame_euler.yaw_deg = mount_frame_pointing.az_deg;
  mount_frame_euler.pitch_deg = mount_frame_pointing.el_deg;
  const Eigen::Matrix3f expected_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(config.mount_angles_deg)) *
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(mount_frame_euler));
  const Eigen::Matrix3f actual_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(pointing));

  ExpectMatricesNear(expected_rotation, actual_rotation, 1.0e-5f);
  EXPECT_NEAR(pointing.yaw_deg, 45.0361f, 1.0e-3f);
  EXPECT_NEAR(pointing.pitch_deg, 0.2416f, 1.0e-3f);
  EXPECT_NEAR(pointing.roll_deg, -1.2326f, 1.0e-3f);
}

TEST(RadarOrientationUtilsTest, ComputePlatformFrameBeamPointingUsesRotationComposition) {
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
  const EulerAnglesDeg pointing = ComputePlatformFrameBeamPointing(platform_attitude, config);
  const AzimuthElevationDeg mount_frame_pointing = ComputeMountFrameBeamPointing(config);
  EulerAnglesDeg mount_frame_euler;
  mount_frame_euler.yaw_deg = mount_frame_pointing.az_deg;
  mount_frame_euler.pitch_deg = mount_frame_pointing.el_deg;
  const Eigen::Matrix3f expected_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(platform_attitude)) *
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(config.mount_angles_deg)) *
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(mount_frame_euler));
  const Eigen::Matrix3f actual_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(pointing));

  ExpectMatricesNear(expected_rotation, actual_rotation, 1.0e-5f);
  EXPECT_NEAR(pointing.yaw_deg, 135.1033f, 1.0e-3f);
  EXPECT_NEAR(pointing.pitch_deg, 3.0682f, 1.0e-3f);
  EXPECT_NEAR(pointing.roll_deg, -2.6508f, 1.0e-3f);
}

TEST(RadarOrientationUtilsTest, ComputePlatformFrameBeamPointingCapturesLargeAttitudeCoupling) {
  RadarOrientationConfig config;
  config.mount_angles_deg.yaw_deg = 35.0f;
  config.mount_angles_deg.pitch_deg = 20.0f;
  config.mount_angles_deg.roll_deg = 15.0f;
  config.scan_center_deg.az_deg = 15.0f;
  config.scan_center_deg.el_deg = 10.0f;
  config.dwell_center_deg.az_deg = 25.0f;
  config.dwell_center_deg.el_deg = 15.0f;
  config.mechanical_scan_limits_deg.az_min_deg = -60.0f;
  config.mechanical_scan_limits_deg.az_max_deg = 60.0f;
  config.mechanical_scan_limits_deg.el_min_deg = -30.0f;
  config.mechanical_scan_limits_deg.el_max_deg = 30.0f;
  config.electronic_scan_limits_deg = config.mechanical_scan_limits_deg;

  EulerAnglesDeg platform_attitude;
  platform_attitude.yaw_deg = 80.0f;
  platform_attitude.pitch_deg = -30.0f;
  platform_attitude.roll_deg = 25.0f;
  const EulerAnglesDeg pointing = ComputePlatformFrameBeamPointing(platform_attitude, config);
  const AzimuthElevationDeg mount_frame_pointing = ComputeMountFrameBeamPointing(config);
  EulerAnglesDeg mount_frame_euler;
  mount_frame_euler.yaw_deg = mount_frame_pointing.az_deg;
  mount_frame_euler.pitch_deg = mount_frame_pointing.el_deg;
  const Eigen::Matrix3f expected_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(platform_attitude)) *
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(config.mount_angles_deg)) *
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(mount_frame_euler));
  const Eigen::Matrix3f actual_rotation =
      oneq::internal::geometry::BuildRotationMatrix(ToGeometryEuler(pointing));

  ExpectMatricesNear(expected_rotation, actual_rotation, 1.0e-5f);
  EXPECT_NEAR(pointing.yaw_deg, 103.5743f, 1.0e-3f);
  EXPECT_NEAR(pointing.pitch_deg, 50.5794f, 1.0e-3f);
  EXPECT_NEAR(pointing.roll_deg, 58.5730f, 1.0e-3f);
  EXPECT_GT(std::fabs(pointing.roll_deg), 30.0f);
}

TEST(RadarOrientationUtilsTest, BeamControlResolverPreservesInertialStabilizedPointing) {
  RadarOrientationConfig config;
  config.stabilization_mode = StabilizationMode::kInertialStabilized;
  config.mechanical_scan_limits_deg.az_min_deg = -120.0f;
  config.mechanical_scan_limits_deg.az_max_deg = 120.0f;
  config.electronic_scan_limits_deg.az_min_deg = -120.0f;
  config.electronic_scan_limits_deg.az_max_deg = 120.0f;

  EulerAnglesDeg platform_attitude;
  platform_attitude.yaw_deg = 90.0f;
  const ResolvedBeamState beam_state = BeamControlResolver::Resolve(
      AntennaConfig(), config, platform_attitude, TargetLookAnglesDeg());

  EXPECT_NEAR(beam_state.beam_pointing_deg.az_deg, -90.0f, 1.0e-5f);
  EXPECT_NEAR(beam_state.beam_pointing_deg.el_deg, 0.0f, 1.0e-5f);
}

}  // namespace tests
}  // namespace airborne_radar
