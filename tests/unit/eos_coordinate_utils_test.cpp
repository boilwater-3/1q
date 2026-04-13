/**
 * @file eos_coordinate_utils_unit_test.cpp
 * @brief 验证 EOS 外部坐标适配工具（LLA/ECEF -> 平台位姿与目标角距输入）的行为。
 */

#include <gtest/gtest.h>

#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/foundation/coordinate_transform.h"

namespace electro_optical_sensor {
namespace utils {
namespace {

TEST(EosCoordinateUtilsTest, MakePoseFromLlaPopulatesPlatformPose) {
  ::electro_optical_sensor::session::EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::foundation::LlaCoordinateDegM pose_lla;
  pose_lla.latitude_deg = 0.0;
  pose_lla.longitude_deg = 0.001;
  pose_lla.altitude_m = 0.0;
  oneq::foundation::EcefCoordinateM pose_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(pose_lla, &pose_ecef));

  oneq::foundation::Vector3f velocity;
  velocity.x = 1.0f;
  velocity.y = 2.0f;
  velocity.z = 3.0f;
  oneq::foundation::EulerAnglesDeg attitude;
  attitude.yaw_deg = 10.0f;

  session::EosExternalPoseInput input;
  input.platform_position_ecef_m = pose_ecef;
  input.platform_velocity_mps = velocity;
  input.platform_velocity_frame = session::EosVelocityFrame::kEnu;
  input.platform_attitude_deg = attitude;

  oneq::foundation::PoseState pose;
  ASSERT_TRUE(session::TryMakeEosPoseFromExternalKinematics(input, reference, &pose));
  EXPECT_GT(pose.position_m.x, 100.0f);
  EXPECT_FLOAT_EQ(pose.velocity_mps.y, 2.0f);
  EXPECT_FLOAT_EQ(pose.attitude_deg.yaw_deg, 10.0f);
}

TEST(EosCoordinateUtilsTest, MakeTargetFromLlaAndEcefAreConsistent) {
  ::electro_optical_sensor::session::EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::foundation::PoseState platform_pose;
  platform_pose.position_m.x = 0.0f;
  platform_pose.position_m.y = 0.0f;
  platform_pose.position_m.z = 0.0f;

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  ::electro_optical_sensor::session::EosTargetAppearance appearance;
  appearance.apparent_temperature_k = 320.0f;
  appearance.emissivity = 0.85f;
  appearance.reflectance = 0.3f;
  appearance.projected_area_m2 = 2.5f;

  ::electro_optical_sensor::session::EosTargetState target_from_lla;
  ASSERT_TRUE(session::TryMakeEosTargetFromLla(11U, target_lla, reference, platform_pose,
                                               appearance, &target_from_lla));
  EXPECT_EQ(target_from_lla.target_id, 11U);
  EXPECT_GT(target_from_lla.range_m, 100.0f);
  EXPECT_NEAR(target_from_lla.azimuth_deg, 0.0f, 1.0e-2f);
  EXPECT_NEAR(target_from_lla.elevation_deg, 0.0f, 1.0e-2f);

  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));
  ::electro_optical_sensor::session::EosTargetState target_from_ecef;
  ASSERT_TRUE(session::TryMakeEosTargetFromEcef(12U, target_ecef, reference, platform_pose,
                                                appearance, &target_from_ecef));
  EXPECT_NEAR(target_from_ecef.range_m, target_from_lla.range_m, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.azimuth_deg, target_from_lla.azimuth_deg, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.elevation_deg, target_from_lla.elevation_deg, 1.0e-3f);
}

TEST(EosCoordinateUtilsTest, InvalidInputReturnsFalse) {
  ::electro_optical_sensor::session::EosCoordinateReference reference;
  session::EosExternalPoseInput input;
  EXPECT_FALSE(session::TryMakeEosPoseFromExternalKinematics(input, reference, nullptr));
}

TEST(EosCoordinateUtilsTest, ReportsFailureStatusForNullOutputAndDegenerateGeometry) {
  ::electro_optical_sensor::session::EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  session::EosExternalPoseInput input;
  ::electro_optical_sensor::session::EosCoordinateStatus status =
      ::electro_optical_sensor::session::EosCoordinateStatus::kOk;
  EXPECT_FALSE(session::TryMakeEosPoseFromExternalKinematics(input, reference, nullptr, &status));
  EXPECT_EQ(status, ::electro_optical_sensor::session::EosCoordinateStatus::kNullOutput);

  oneq::foundation::PoseState platform_pose;
  platform_pose.position_m.x = 0.0f;
  platform_pose.position_m.y = 0.0f;
  platform_pose.position_m.z = 0.0f;
  ::electro_optical_sensor::session::EosTargetAppearance appearance;
  ::electro_optical_sensor::session::EosTargetState target;
  oneq::foundation::LlaCoordinateDegM target_lla = reference.origin_lla;
  EXPECT_FALSE(session::TryMakeEosTargetFromLla(1U, target_lla, reference, platform_pose,
                                                appearance, &target, &status));
  EXPECT_EQ(status, ::electro_optical_sensor::session::EosCoordinateStatus::kDegenerateGeometry);
}

}  // namespace
}  // namespace utils
}  // namespace electro_optical_sensor
