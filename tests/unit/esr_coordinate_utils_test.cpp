/**
 * @file esr_coordinate_utils_test.cpp
 * @brief 验证 ESR 外部坐标适配工具（LLA/ECEF -> ESR 局部位姿）的行为。
 */

#include <gtest/gtest.h>

#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/foundation/coordinate_transform.h"

namespace electronic_surveillance_radar {
namespace model {
namespace {

TEST(EsrCoordinateUtilsTest, ExternalKinematicsSupportsEnuAndEcefVelocity) {
  session::EsrCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));
  EsrVector3f velocity;
  velocity.x = 11.0f;
  velocity.y = 12.0f;
  velocity.z = 13.0f;
  EsrEulerAngleDeg attitude;
  attitude.yaw_deg = 1.0f;
  attitude.pitch_deg = 2.0f;
  attitude.roll_deg = 3.0f;

  session::EsrExternalPoseInput enu_input;
  enu_input.platform_position_ecef_m = target_ecef;
  enu_input.platform_velocity_mps = velocity;
  enu_input.platform_velocity_frame = session::EsrVelocityFrame::kEnu;
  enu_input.platform_attitude_deg = attitude;

  session::EsrExternalPoseInput ecef_input;
  ecef_input.platform_position_ecef_m = target_ecef;
  ecef_input.platform_velocity_frame = session::EsrVelocityFrame::kEcef;
  ecef_input.platform_attitude_deg = attitude;
  ecef_input.platform_velocity_mps.x = 13.0f;  // up -> +X at lat=0, lon=0
  ecef_input.platform_velocity_mps.y = 11.0f;  // east -> +Y at lat=0, lon=0
  ecef_input.platform_velocity_mps.z = 12.0f;  // north -> +Z at lat=0, lon=0

  EsrPoseState pose_from_enu;
  ASSERT_TRUE(session::TryMakeEsrPoseFromExternalKinematics(enu_input, reference, &pose_from_enu));
  EXPECT_GT(pose_from_enu.position_m.x, 100.0f);
  EXPECT_FLOAT_EQ(pose_from_enu.velocity_mps.x, 11.0f);
  EXPECT_FLOAT_EQ(pose_from_enu.velocity_mps.y, 12.0f);
  EXPECT_FLOAT_EQ(pose_from_enu.velocity_mps.z, 13.0f);
  EXPECT_FLOAT_EQ(pose_from_enu.attitude_deg.yaw_deg, 1.0f);
  EXPECT_FLOAT_EQ(pose_from_enu.attitude_deg.pitch_deg, 2.0f);
  EXPECT_FLOAT_EQ(pose_from_enu.attitude_deg.roll_deg, 3.0f);

  EsrPoseState pose_from_ecef;
  ASSERT_TRUE(
      session::TryMakeEsrPoseFromExternalKinematics(ecef_input, reference, &pose_from_ecef));
  EXPECT_NEAR(pose_from_ecef.position_m.x, pose_from_enu.position_m.x, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.position_m.y, pose_from_enu.position_m.y, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.position_m.z, pose_from_enu.position_m.z, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.velocity_mps.x, pose_from_enu.velocity_mps.x, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.velocity_mps.y, pose_from_enu.velocity_mps.y, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.velocity_mps.z, pose_from_enu.velocity_mps.z, 1.0e-3f);
}

TEST(EsrCoordinateUtilsTest, InvalidInputReturnsFalse) {
  session::EsrCoordinateReference reference;
  session::EsrExternalPoseInput input;
  EXPECT_FALSE(session::TryMakeEsrPoseFromExternalKinematics(input, reference, nullptr));
}

TEST(EsrCoordinateUtilsTest, ExternalEmitterInputBuildsSceneEmitter) {
  session::EsrCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::foundation::LlaCoordinateDegM emitter_lla;
  emitter_lla.latitude_deg = 0.0;
  emitter_lla.longitude_deg = 0.001;
  emitter_lla.altitude_m = 0.0;

  oneq::foundation::EcefCoordinateM emitter_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(emitter_lla, &emitter_ecef));

  session::EsrExternalEmitterInput input;
  input.emitter_id = "e-001";
  input.emitter_position_ecef_m = emitter_ecef;
  input.emitter_velocity_frame = session::EsrVelocityFrame::kEnu;
  input.emitter_velocity_mps.x = 1.0f;
  input.emitter_velocity_mps.y = 2.0f;
  input.emitter_velocity_mps.z = 3.0f;
  input.carrier_hz = 10.0e9;
  input.bandwidth_hz = 2.0e6;
  input.tx_power_w = 5.0e7;
  input.pulse_width_s = 1.0e-6;
  input.pri_s = 1.0e-4;

  session::EsrSceneEmitter emitter;
  session::EsrCoordinateStatus status = session::EsrCoordinateStatus::kCoordinateTransformFail;
  ASSERT_TRUE(
      session::TryMakeEsrSceneEmitterFromExternalInput(input, reference, &emitter, &status));
  EXPECT_EQ(status, session::EsrCoordinateStatus::kOk);
  EXPECT_EQ(emitter.emitter_id, "e-001");
  EXPECT_GT(emitter.pose.position_m.x, 100.0f);
  EXPECT_NEAR(emitter.pose.velocity_mps.x, 1.0f, 1.0e-5f);
  EXPECT_NEAR(emitter.pose.velocity_mps.y, 2.0f, 1.0e-5f);
  EXPECT_NEAR(emitter.pose.velocity_mps.z, 3.0f, 1.0e-5f);
}

}  // namespace
}  // namespace model
}  // namespace electronic_surveillance_radar
