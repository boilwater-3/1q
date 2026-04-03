/**
 * @file eos_coordinate_utils_unit_test.cpp
 * @brief 验证 EOS 外部坐标适配工具（LLA/ECEF -> 平台位姿与目标角距输入）的行为。
 */

#include <gtest/gtest.h>

#include "1q/common/coordinate_transform.h"
#include "1q/electro_optical_sensor/core/context/EosCoordinateUtils.h"

namespace electro_optical_sensor {
namespace core {
namespace context {
namespace {

TEST(EosCoordinateUtilsTest, MakePoseFromLlaPopulatesPlatformPose) {
  EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::common::LlaCoordinateDegM pose_lla;
  pose_lla.latitude_deg = 0.0;
  pose_lla.longitude_deg = 0.001;
  pose_lla.altitude_m = 0.0;

  oneq::common::Vector3f velocity;
  velocity.x = 1.0f;
  velocity.y = 2.0f;
  velocity.z = 3.0f;
  oneq::common::EulerAnglesDeg attitude;
  attitude.yaw_deg = 10.0f;

  oneq::common::PoseState pose;
  ASSERT_TRUE(TryMakeEosPoseFromLla(pose_lla, reference, velocity, attitude, &pose));
  EXPECT_GT(pose.position_m.x, 100.0f);
  EXPECT_FLOAT_EQ(pose.velocity_mps.y, 2.0f);
  EXPECT_FLOAT_EQ(pose.attitude_deg.yaw_deg, 10.0f);
}

TEST(EosCoordinateUtilsTest, MakeTargetFromLlaAndEcefAreConsistent) {
  EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::common::PoseState platform_pose;
  platform_pose.position_m.x = 0.0f;
  platform_pose.position_m.y = 0.0f;
  platform_pose.position_m.z = 0.0f;

  oneq::common::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  EosTargetAppearance appearance;
  appearance.apparent_temperature_k = 320.0f;
  appearance.emissivity = 0.85f;
  appearance.reflectance = 0.3f;
  appearance.projected_area_m2 = 2.5f;

  EosTargetState target_from_lla;
  ASSERT_TRUE(TryMakeEosTargetFromLla(11U, target_lla, reference, platform_pose, appearance,
                                      &target_from_lla));
  EXPECT_EQ(target_from_lla.target_id, 11U);
  EXPECT_GT(target_from_lla.range_m, 100.0f);
  EXPECT_NEAR(target_from_lla.azimuth_deg, 0.0f, 1.0e-2f);
  EXPECT_NEAR(target_from_lla.elevation_deg, 0.0f, 1.0e-2f);

  oneq::common::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(target_lla, &target_ecef));
  EosTargetState target_from_ecef;
  ASSERT_TRUE(TryMakeEosTargetFromEcef(12U, target_ecef, reference, platform_pose, appearance,
                                       &target_from_ecef));
  EXPECT_NEAR(target_from_ecef.range_m, target_from_lla.range_m, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.azimuth_deg, target_from_lla.azimuth_deg, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.elevation_deg, target_from_lla.elevation_deg, 1.0e-3f);
}

TEST(EosCoordinateUtilsTest, InvalidInputReturnsFalse) {
  EosCoordinateReference reference;
  oneq::common::PoseState platform_pose;
  EosTargetAppearance appearance;

  oneq::common::EcefCoordinateM ecef;
  EXPECT_FALSE(TryConvertEcefToEosLocal(ecef, reference, nullptr));
  EXPECT_FALSE(TryMakeEosPoseFromEcef(ecef, reference, oneq::common::Vector3f(),
                                      oneq::common::EulerAnglesDeg(), nullptr));
  EXPECT_FALSE(TryMakeEosTargetFromEcef(1U, ecef, reference, platform_pose, appearance, nullptr));
}

}  // namespace
}  // namespace context
}  // namespace core
}  // namespace electro_optical_sensor

