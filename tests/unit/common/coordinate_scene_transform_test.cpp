/**
 * @file coordinate_scene_transform_test.cpp
 * @brief 验证 oneq::coordinate 平台锚点 ENU 场景一站式转换的分帧正确性与失败契约。
 */

#include <gtest/gtest.h>

#include <limits>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace oneq {
namespace coordinate {
namespace {

LlaPositionDegM MakeAnchor() {
  LlaPositionDegM anchor;
  anchor.latitude_deg = 30.0;
  anchor.longitude_deg = 120.0;
  anchor.altitude_m = 5000.0;
  return anchor;
}

EcefPositionM MakeAnchorEcef() {
  EcefPositionM ecef;
  EXPECT_TRUE(TryLlaToEcef(MakeAnchor(), &ecef));
  return ecef;
}

ExternalKinematics MakeEcefKinematics() {
  ExternalKinematics kinematics;
  kinematics.position_frame = PositionFrame::kEcef;
  const EcefPositionM anchor_ecef = MakeAnchorEcef();
  kinematics.position_ecef_m.x_m = anchor_ecef.x_m + 1000.0;
  kinematics.position_ecef_m.y_m = anchor_ecef.y_m + 2000.0;
  kinematics.position_ecef_m.z_m = anchor_ecef.z_m + 3000.0;
  kinematics.velocity_mps.x_mps = 10.0;
  kinematics.velocity_mps.y_mps = 0.0;
  kinematics.velocity_mps.z_mps = 0.0;
  return kinematics;
}

// ECEF 分支与底层 TryEcefToEnu / TryEcefToEnuVelocity 逐步组合结果一致。
TEST(SceneTransformTest, EcefBranchMatchesComposedTransforms) {
  const ExternalKinematics kinematics = MakeEcefKinematics();
  const LlaPositionDegM anchor = MakeAnchor();

  EnuPositionM expect_position;
  EnuVelocityMps expect_velocity;
  ASSERT_TRUE(TryEcefToEnu(kinematics.position_ecef_m, anchor, &expect_position));
  ASSERT_TRUE(TryEcefToEnuVelocity(kinematics.velocity_mps, anchor, &expect_velocity));

  EnuSceneState state;
  ASSERT_TRUE(TryMakeEnuSceneState(kinematics, anchor, &state));
  EXPECT_NEAR(state.position_enu_m.east_m, expect_position.east_m, 1.0e-6);
  EXPECT_NEAR(state.position_enu_m.north_m, expect_position.north_m, 1.0e-6);
  EXPECT_NEAR(state.position_enu_m.up_m, expect_position.up_m, 1.0e-6);
  EXPECT_NEAR(state.velocity_enu_mps.east_mps, expect_velocity.east_mps, 1.0e-9);
  EXPECT_NEAR(state.velocity_enu_mps.north_mps, expect_velocity.north_mps, 1.0e-9);
  EXPECT_NEAR(state.velocity_enu_mps.up_mps, expect_velocity.up_mps, 1.0e-9);
}

// LLA 分支与等价 ECEF 输入落点一致（位置字段互斥语义）。
TEST(SceneTransformTest, LlaBranchMatchesEcefEquivalent) {
  const LlaPositionDegM anchor = MakeAnchor();

  LlaPositionDegM target_lla;
  target_lla.latitude_deg = anchor.latitude_deg + 0.01;  // ~1.11 km 北向偏移
  target_lla.longitude_deg = anchor.longitude_deg;
  target_lla.altitude_m = anchor.altitude_m + 100.0;
  EcefPositionM target_ecef;
  ASSERT_TRUE(TryLlaToEcef(target_lla, &target_ecef));

  ExternalKinematics lla_input;
  lla_input.position_frame = PositionFrame::kLla;
  lla_input.position_lla_deg_m = target_lla;
  lla_input.velocity_mps.x_mps = 5.0;
  lla_input.velocity_mps.y_mps = 5.0;
  lla_input.velocity_mps.z_mps = 5.0;

  ExternalKinematics ecef_input;
  ecef_input.position_frame = PositionFrame::kEcef;
  ecef_input.position_ecef_m = target_ecef;
  ecef_input.velocity_mps = lla_input.velocity_mps;

  EnuSceneState lla_state;
  EnuSceneState ecef_state;
  ASSERT_TRUE(TryMakeEnuSceneState(lla_input, anchor, &lla_state));
  ASSERT_TRUE(TryMakeEnuSceneState(ecef_input, anchor, &ecef_state));
  EXPECT_NEAR(lla_state.position_enu_m.east_m, ecef_state.position_enu_m.east_m, 1.0e-3);
  EXPECT_NEAR(lla_state.position_enu_m.north_m, ecef_state.position_enu_m.north_m, 1.0e-3);
  EXPECT_NEAR(lla_state.position_enu_m.up_m, ecef_state.position_enu_m.up_m, 1.0e-3);
  EXPECT_EQ(lla_state.velocity_enu_mps.east_mps, ecef_state.velocity_enu_mps.east_mps);
}

// 非法输入契约：空输出指针、非有限速度、非法锚点均失败且不写输出。
TEST(SceneTransformTest, RejectsInvalidInputs) {
  const LlaPositionDegM anchor = MakeAnchor();

  EnuSceneState state;
  EXPECT_FALSE(TryMakeEnuSceneState(MakeEcefKinematics(), anchor, nullptr));

  ExternalKinematics nan_velocity = MakeEcefKinematics();
  nan_velocity.velocity_mps.x_mps = std::numeric_limits<double>::quiet_NaN();
  state = EnuSceneState{};
  EXPECT_FALSE(TryMakeEnuSceneState(nan_velocity, anchor, &state));
  EXPECT_DOUBLE_EQ(state.position_enu_m.east_m, 0.0);

  LlaPositionDegM bad_anchor;
  bad_anchor.latitude_deg = 120.0;  // 纬度越界
  bad_anchor.longitude_deg = 0.0;
  bad_anchor.altitude_m = 0.0;
  EXPECT_FALSE(TryMakeEnuSceneState(MakeEcefKinematics(), bad_anchor, &state));
}

}  // namespace
}  // namespace coordinate
}  // namespace oneq
