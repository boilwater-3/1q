/**
 * @file ar_external_input_adapter_test.cpp
 * @brief 验证 ArExternalInputAdapter 的坐标转换与校验分支（此前 32% 覆盖）。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/airborne_radar/session/ArExternalInputAdapter.h"
#include "1q/coordinate/types.h"

namespace airborne_radar {
namespace session {
namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EcefVelocityMps;
using oneq::coordinate::EulerAnglesDeg;
using oneq::coordinate::LlaPositionDegM;
using oneq::coordinate::LocalFrameReference;

// 有效 ECEF 位置（赤道附近）
EcefPositionM MakeValidEcefPosition() {
  LlaPositionDegM lla;
  lla.latitude_deg = 31.0;
  lla.longitude_deg = 121.0;
  lla.altitude_m = 5000.0;
  EcefPositionM ecef;
  // 通过坐标转换得到有效 ECEF（避免手写大数值出错）
  // 直接用已知值（上海附近约略）
  ecef.x_m = -2868000.0;
  ecef.y_m = 4898000.0;
  ecef.z_m = 3275000.0;
  return ecef;
}

EcefVelocityMps MakeValidEcefVelocity() {
  EcefVelocityMps v;
  v.x_mps = 100.0;
  v.y_mps = 50.0;
  v.z_mps = 10.0;
  return v;
}

ArExternalPoseInput MakeValidPoseInput() {
  ArExternalPoseInput input;
  input.platform_position_ecef_m = MakeValidEcefPosition();
  input.platform_velocity_mps = MakeValidEcefVelocity();
  input.platform_attitude_deg = EulerAnglesDeg{10.0, -5.0, 2.0};
  return input;
}

constexpr EulerAnglesDeg kZeroMount{0.0, 0.0, 0.0};

// =============================================================================
// TryMakeArPoseFromExternalKinematics
// =============================================================================

TEST(ArExternalInputAdapterTest, ValidPoseConvertsToFrameAndVelocity) {
  ArExternalPoseInput input = MakeValidPoseInput();
  LocalFrameReference reference;
  oneq::foundation::Vector3f velocity;
  ArCoordinateStatus status;

  ASSERT_TRUE(TryMakeArPoseFromExternalKinematics(input, kZeroMount, &reference, &velocity, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kOk);
  // 验证 LLA 转换成功且纬度在合理范围
  EXPECT_GT(reference.origin_lla.latitude_deg, 20.0);
  EXPECT_LT(reference.origin_lla.latitude_deg, 40.0);
}

TEST(ArExternalInputAdapterTest, NullOutputsReturnFalseWithStatus) {
  ArExternalPoseInput input = MakeValidPoseInput();
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArPoseFromExternalKinematics(input, kZeroMount, nullptr, nullptr, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kNullOutput);
}

TEST(ArExternalInputAdapterTest, NullStatusDoesNotCrash) {
  ArExternalPoseInput input = MakeValidPoseInput();
  LocalFrameReference reference;
  oneq::foundation::Vector3f velocity;
  EXPECT_TRUE(TryMakeArPoseFromExternalKinematics(input, kZeroMount, &reference, &velocity, nullptr));
}

TEST(ArExternalInputAdapterTest, InvalidEcefPositionReturnsTransformFail) {
  ArExternalPoseInput input = MakeValidPoseInput();
  input.platform_position_ecef_m = EcefPositionM{0.0, 0.0, 0.0};  // 原点 → norm=0
  LocalFrameReference reference;
  oneq::foundation::Vector3f velocity;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArPoseFromExternalKinematics(input, kZeroMount, &reference, &velocity, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kCoordinateTransformFail);
}

TEST(ArExternalInputAdapterTest, NonFiniteVelocityUsesZeroVelocity) {
  ArExternalPoseInput input = MakeValidPoseInput();
  input.platform_velocity_mps.x_mps = std::numeric_limits<double>::quiet_NaN();
  LocalFrameReference reference;
  oneq::foundation::Vector3f velocity;
  ArCoordinateStatus status;
  // 非有限速度不导致失败，而是退化为零速度
  ASSERT_TRUE(TryMakeArPoseFromExternalKinematics(input, kZeroMount, &reference, &velocity, &status));
  EXPECT_EQ(velocity.x, 0.0f);
  EXPECT_EQ(velocity.y, 0.0f);
  EXPECT_EQ(velocity.z, 0.0f);
}

// =============================================================================
// TryMakeArTargetFromExternalKinematics
// =============================================================================

TEST(ArExternalInputAdapterTest, ValidTargetWithEcefPositionConverts) {
  ArExternalPoseInput pose_input = MakeValidPoseInput();
  LocalFrameReference reference;
  oneq::foundation::Vector3f velocity;
  ASSERT_TRUE(TryMakeArPoseFromExternalKinematics(pose_input, kZeroMount, &reference, &velocity, nullptr));

  ArExternalTargetInput target_input;
  target_input.target_id = 42U;
  target_input.target_name = "test";
  target_input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  // 目标位置略偏移平台
  target_input.kinematics.position_ecef_m.x_m = pose_input.platform_position_ecef_m.x_m + 1000.0;
  target_input.kinematics.position_ecef_m.y_m = pose_input.platform_position_ecef_m.y_m;
  target_input.kinematics.position_ecef_m.z_m = pose_input.platform_position_ecef_m.z_m;
  target_input.kinematics.velocity_mps = MakeValidEcefVelocity();
  target_input.rcs = 5.0f;

  ArSceneTarget target;
  ArCoordinateStatus status;
  ASSERT_TRUE(TryMakeArTargetFromExternalKinematics(
      target_input, reference, velocity, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kOk);
  EXPECT_EQ(target.external_target_id, 42U);
  EXPECT_GT(target.range_m, 0.0f);
}

TEST(ArExternalInputAdapterTest, ValidTargetWithLlaPositionConverts) {
  ArExternalPoseInput pose_input = MakeValidPoseInput();
  LocalFrameReference reference;
  oneq::foundation::Vector3f velocity;
  ASSERT_TRUE(TryMakeArPoseFromExternalKinematics(pose_input, kZeroMount, &reference, &velocity, nullptr));

  ArExternalTargetInput target_input;
  target_input.target_id = 99U;
  target_input.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  target_input.kinematics.position_lla_deg_m.latitude_deg = 31.001;
  target_input.kinematics.position_lla_deg_m.longitude_deg = 121.0;
  target_input.kinematics.position_lla_deg_m.altitude_m = 5000.0;
  target_input.kinematics.velocity_mps = MakeValidEcefVelocity();

  ArSceneTarget target;
  ArCoordinateStatus status;
  ASSERT_TRUE(TryMakeArTargetFromExternalKinematics(
      target_input, reference, velocity, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kOk);
}

TEST(ArExternalInputAdapterTest, NullTargetReturnsFalse) {
  ArExternalTargetInput target_input;
  target_input.kinematics.velocity_mps = MakeValidEcefVelocity();
  LocalFrameReference reference;
  oneq::foundation::Vector3f radar_vel;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArTargetFromExternalKinematics(
      target_input, reference, radar_vel, nullptr, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kNullOutput);
}

TEST(ArExternalInputAdapterTest, NonFiniteTargetVelocityReturnsTransformFail) {
  ArExternalTargetInput target_input;
  target_input.kinematics.velocity_mps.x_mps = std::numeric_limits<double>::quiet_NaN();
  LocalFrameReference reference;
  oneq::foundation::Vector3f radar_vel;
  ArSceneTarget target;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArTargetFromExternalKinematics(
      target_input, reference, radar_vel, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kCoordinateTransformFail);
}

TEST(ArExternalInputAdapterTest, NonFiniteRadarVelocityReturnsTransformFail) {
  ArExternalTargetInput target_input;
  target_input.kinematics.velocity_mps = MakeValidEcefVelocity();
  LocalFrameReference reference;
  oneq::foundation::Vector3f radar_vel;
  radar_vel.x = std::numeric_limits<float>::quiet_NaN();
  ArSceneTarget target;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArTargetFromExternalKinematics(
      target_input, reference, radar_vel, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kCoordinateTransformFail);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
