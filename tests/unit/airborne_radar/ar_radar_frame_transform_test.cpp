/**
 * @file ar_radar_frame_transform_test.cpp
 * @brief 验证 ArRadarFrameTransform 的坐标转换与校验分支（此前 32% 覆盖）。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/airborne_radar/session/ArRadarFrameTransform.h"
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

ArPlatformInput MakeValidPoseInput() {
  ArPlatformInput input;
  input.platform_position_ecef_m = MakeValidEcefPosition();
  input.platform_velocity_mps = MakeValidEcefVelocity();
  input.platform_attitude_deg = EulerAnglesDeg{10.0, -5.0, 2.0};
  return input;
}

// be501c5d（VS2015 兼容）为 EulerAnglesDeg 引入非 constexpr 构造函数，
// 聚合初始化不再可 constexpr——此处按普通常量处理。
const EulerAnglesDeg kZeroMount{0.0, 0.0, 0.0};

// =============================================================================
// TryMakeArPoseFromPlatform
// =============================================================================

TEST(ArRadarFrameTransformTest, ValidPoseConvertsToFrameAndVelocity) {
  ArPlatformInput input = MakeValidPoseInput();
  LocalFrameReference reference;
  oneq::coordinate::Vector3d velocity;
  ArCoordinateStatus status;

  ASSERT_TRUE(TryMakeArPoseFromPlatform(input, kZeroMount, &reference, &velocity, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kOk);
  // 验证 LLA 转换成功且纬度在合理范围
  EXPECT_GT(reference.origin_lla.latitude_deg, 20.0);
  EXPECT_LT(reference.origin_lla.latitude_deg, 40.0);
}

TEST(ArRadarFrameTransformTest, NullOutputsReturnFalseWithStatus) {
  ArPlatformInput input = MakeValidPoseInput();
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArPoseFromPlatform(input, kZeroMount, nullptr, nullptr, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kNullOutput);
}

TEST(ArRadarFrameTransformTest, NullStatusDoesNotCrash) {
  ArPlatformInput input = MakeValidPoseInput();
  LocalFrameReference reference;
  oneq::coordinate::Vector3d velocity;
  EXPECT_TRUE(TryMakeArPoseFromPlatform(input, kZeroMount, &reference, &velocity, nullptr));
}

TEST(ArRadarFrameTransformTest, InvalidEcefPositionReturnsTransformFail) {
  ArPlatformInput input = MakeValidPoseInput();
  input.platform_position_ecef_m = EcefPositionM{0.0, 0.0, 0.0};  // 原点 → norm=0
  LocalFrameReference reference;
  oneq::coordinate::Vector3d velocity;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArPoseFromPlatform(input, kZeroMount, &reference, &velocity, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kCoordinateTransformFail);
}

TEST(ArRadarFrameTransformTest, NonFiniteVelocityUsesZeroVelocity) {
  ArPlatformInput input = MakeValidPoseInput();
  input.platform_velocity_mps.x_mps = std::numeric_limits<double>::quiet_NaN();
  LocalFrameReference reference;
  oneq::coordinate::Vector3d velocity;
  ArCoordinateStatus status;
  // 非有限速度不导致失败，而是退化为零速度
  ASSERT_TRUE(TryMakeArPoseFromPlatform(input, kZeroMount, &reference, &velocity, &status));
  EXPECT_EQ(velocity.x, 0.0f);
  EXPECT_EQ(velocity.y, 0.0f);
  EXPECT_EQ(velocity.z, 0.0f);
}

// =============================================================================
// TryMakeArTargetFromEnu
// =============================================================================

TEST(ArRadarFrameTransformTest, ValidEnuTargetWithZeroAttitudePreservesGeometry) {
  ArPlatformInput pose_input = MakeValidPoseInput();
  pose_input.platform_attitude_deg = EulerAnglesDeg{0.0, 0.0, 0.0};
  LocalFrameReference reference;
  oneq::coordinate::Vector3d velocity;
  ASSERT_TRUE(TryMakeArPoseFromPlatform(pose_input, kZeroMount, &reference, &velocity,
                                                  nullptr));

  ArTargetInput target_input;
  target_input.target_id = 42U;
  target_input.target_name = "test";
  target_input.position_x = 1000.0f;
  target_input.position_y = 2000.0f;
  target_input.position_z = 3000.0f;
  target_input.velocity_x = 10.0f;
  target_input.velocity_y = 20.0f;
  target_input.velocity_z = 30.0f;
  target_input.rcs = 5.0f;

  ArSceneTarget target;
  ArCoordinateStatus status;
  ASSERT_TRUE(TryMakeArTargetFromEnu(target_input, reference, velocity, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kOk);
  EXPECT_EQ(target.external_target_id, 42U);
  EXPECT_EQ(target.target_name, "test");
  EXPECT_NEAR(target.position_x, 1000.0f, 1.0e-3f);
  EXPECT_NEAR(target.position_y, 2000.0f, 1.0e-3f);
  EXPECT_NEAR(target.position_z, 3000.0f, 1.0e-3f);
  // 零姿态下雷达系轴 = ENU 轴；相对速度 = 目标 ENU 速度 - 平台雷达系速度
  //（velocity 为平台 ECEF 速度旋入 ENU 轴的结果，非 ECEF 原值）。
  EXPECT_NEAR(target.velocity_x, 10.0f - velocity.x, 0.2f);
  EXPECT_NEAR(target.velocity_y, 20.0f - velocity.y, 0.2f);
  EXPECT_NEAR(target.velocity_z, 30.0f - velocity.z, 0.2f);
  EXPECT_NEAR(target.range_m, std::sqrt(1000.0f * 1000.0f + 2000.0f * 2000.0f +
                                            3000.0f * 3000.0f),
              1.0e-2f);
  EXPECT_FLOAT_EQ(target.rcs, 5.0f);
}

TEST(ArRadarFrameTransformTest, EnuTargetRotationPreservesNormAndSubtractsPlatformVelocity) {
  ArPlatformInput pose_input = MakeValidPoseInput();
  // 平台零速度：相对速度 = 目标 ENU 速度旋入雷达体系，模长保持
  pose_input.platform_velocity_mps = EcefVelocityMps{};
  LocalFrameReference reference;
  oneq::coordinate::Vector3d velocity;
  ASSERT_TRUE(TryMakeArPoseFromPlatform(pose_input, kZeroMount, &reference, &velocity,
                                                  nullptr));
  EXPECT_EQ(velocity.x, 0.0f);
  EXPECT_EQ(velocity.y, 0.0f);
  EXPECT_EQ(velocity.z, 0.0f);

  ArTargetInput target_input;
  target_input.position_x = 1500.0f;
  target_input.position_y = -800.0f;
  target_input.position_z = 600.0f;
  target_input.velocity_x = 40.0f;
  target_input.velocity_y = -25.0f;
  target_input.velocity_z = 10.0f;

  ArSceneTarget target;
  ArCoordinateStatus status;
  ASSERT_TRUE(TryMakeArTargetFromEnu(target_input, reference, velocity, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kOk);
  // 旋转不改范数：斜距与 ENU 位置模长一致
  const float enu_norm =
      std::sqrt(1500.0f * 1500.0f + 800.0f * 800.0f + 600.0f * 600.0f);
  EXPECT_NEAR(target.range_m, enu_norm, 1.0e-2f);
  const float relative_norm = std::sqrt(target.velocity_x * target.velocity_x +
                                        target.velocity_y * target.velocity_y +
                                        target.velocity_z * target.velocity_z);
  const float target_norm =
      std::sqrt(40.0f * 40.0f + 25.0f * 25.0f + 10.0f * 10.0f);
  EXPECT_NEAR(relative_norm, target_norm, 1.0e-2f);
}

TEST(ArRadarFrameTransformTest, NullEnuTargetReturnsFalse) {
  ArTargetInput target_input;
  LocalFrameReference reference;
  oneq::coordinate::Vector3d radar_vel;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArTargetFromEnu(target_input, reference, radar_vel, nullptr, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kNullOutput);
}

TEST(ArRadarFrameTransformTest, NonFiniteEnuPositionReturnsTransformFail) {
  ArTargetInput target_input;
  target_input.position_x = std::numeric_limits<float>::quiet_NaN();
  target_input.velocity_x = 10.0f;
  LocalFrameReference reference;
  oneq::coordinate::Vector3d radar_vel;
  ArSceneTarget target;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArTargetFromEnu(target_input, reference, radar_vel, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kCoordinateTransformFail);
}

TEST(ArRadarFrameTransformTest, NonFiniteEnuVelocityReturnsTransformFail) {
  ArTargetInput target_input;
  target_input.velocity_z = std::numeric_limits<float>::infinity();
  LocalFrameReference reference;
  oneq::coordinate::Vector3d radar_vel;
  ArSceneTarget target;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArTargetFromEnu(target_input, reference, radar_vel, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kCoordinateTransformFail);
}

TEST(ArRadarFrameTransformTest, NonFiniteRadarVelocityReturnsTransformFail) {
  ArTargetInput target_input;
  target_input.velocity_x = 10.0f;
  LocalFrameReference reference;
  oneq::coordinate::Vector3d radar_vel;
  radar_vel.x = std::numeric_limits<float>::quiet_NaN();
  ArSceneTarget target;
  ArCoordinateStatus status;
  EXPECT_FALSE(TryMakeArTargetFromEnu(target_input, reference, radar_vel, &target, &status));
  EXPECT_EQ(status, ArCoordinateStatus::kCoordinateTransformFail);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
