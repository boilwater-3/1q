/**
 * @file esr_coordinate_utils_test.cpp
 * @brief 验证 ESR 外部坐标适配工具（LLA/ECEF -> ESR 局部位姿）的行为。
 */

#include <gtest/gtest.h>

#include "1q/common/coordinate_transform.h"
#include "1q/electronic_surveillance_radar/common/EsrCoordinateUtils.h"

namespace electronic_surveillance_radar {
namespace common {
namespace {

TEST(EsrCoordinateUtilsTest, LlaAndEcefConversionAreConsistent) {
  EsrCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::common::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  EsrVector3f local_from_lla;
  ASSERT_TRUE(TryConvertLlaToEsrLocal(target_lla, reference, &local_from_lla));
  EXPECT_GT(local_from_lla.x, 100.0f);
  EXPECT_NEAR(local_from_lla.y, 0.0f, 1.0e-2f);
  EXPECT_NEAR(local_from_lla.z, 0.0f, 1.0e-2f);

  oneq::common::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(target_lla, &target_ecef));
  EsrVector3f local_from_ecef;
  ASSERT_TRUE(TryConvertEcefToEsrLocal(target_ecef, reference, &local_from_ecef));
  EXPECT_NEAR(local_from_ecef.x, local_from_lla.x, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.y, local_from_lla.y, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.z, local_from_lla.z, 1.0e-3f);
}

TEST(EsrCoordinateUtilsTest, MakePoseFromLlaPopulatesPoseState) {
  EsrCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::common::LlaCoordinateDegM pose_lla;
  pose_lla.latitude_deg = 0.0;
  pose_lla.longitude_deg = 0.001;
  pose_lla.altitude_m = 0.0;

  EsrVector3f velocity;
  velocity.x = 11.0f;
  velocity.y = 12.0f;
  velocity.z = 13.0f;
  EsrEulerAngleDeg attitude;
  attitude.yaw_deg = 1.0f;
  attitude.pitch_deg = 2.0f;
  attitude.roll_deg = 3.0f;

  EsrPoseState pose;
  ASSERT_TRUE(TryMakeEsrPoseFromLla(pose_lla, reference, velocity, attitude, &pose));
  EXPECT_GT(pose.position_m.x, 100.0f);
  EXPECT_FLOAT_EQ(pose.velocity_mps.x, 11.0f);
  EXPECT_FLOAT_EQ(pose.velocity_mps.y, 12.0f);
  EXPECT_FLOAT_EQ(pose.velocity_mps.z, 13.0f);
  EXPECT_FLOAT_EQ(pose.attitude_deg.yaw_deg, 1.0f);
  EXPECT_FLOAT_EQ(pose.attitude_deg.pitch_deg, 2.0f);
  EXPECT_FLOAT_EQ(pose.attitude_deg.roll_deg, 3.0f);
}

TEST(EsrCoordinateUtilsTest, InvalidInputReturnsFalse) {
  EsrCoordinateReference reference;
  oneq::common::EcefCoordinateM ecef;
  EXPECT_FALSE(TryConvertEcefToEsrLocal(ecef, reference, nullptr));
  EXPECT_FALSE(TryMakeEsrPoseFromEcef(ecef, reference, EsrVector3f(), EsrEulerAngleDeg(),
                                      nullptr));

  oneq::common::LlaCoordinateDegM invalid_lla;
  invalid_lla.latitude_deg = 95.0;
  invalid_lla.longitude_deg = 0.0;
  invalid_lla.altitude_m = 0.0;

  EsrVector3f local;
  EXPECT_FALSE(TryConvertLlaToEsrLocal(invalid_lla, reference, &local));
}

}  // namespace
}  // namespace common
}  // namespace electronic_surveillance_radar

