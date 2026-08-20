/**
 * @file eos_coordinate_utils_unit_test.cpp
 * @brief 验证 TryMakeEnuSceneState → EosSceneTarget 直填（世界 ECEF/LLA → 平台锚点 ENU）。
 */

#include <gtest/gtest.h>

#include <limits>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "support/eos_enu_scene_helpers.h"

namespace electro_optical_sensor {
namespace utils {
namespace {

using oneq::test_support::TryFillEosSceneTargetFromKinematics;

oneq::coordinate::LlaPositionDegM MakeAnchor() {
  oneq::coordinate::LlaPositionDegM anchor;
  anchor.latitude_deg = 0.0;
  anchor.longitude_deg = 0.0;
  anchor.altitude_m = 0.0;
  return anchor;
}

TEST(EosCoordinateUtilsTest, MakeTargetFromLlaAndEcefAreConsistent) {
  const oneq::coordinate::LlaPositionDegM anchor = MakeAnchor();

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  ::electro_optical_sensor::session::EosTargetAppearance appearance;
  appearance.apparent_temperature_k = 320.0f;
  appearance.emissivity = 0.85f;
  appearance.reflectance = 0.3f;
  appearance.projected_area_m2 = 2.5f;

  oneq::coordinate::ExternalKinematics lla_kinematics;
  lla_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  lla_kinematics.position_lla_deg_m = target_lla;
  ::electro_optical_sensor::session::EosSceneTarget target_from_lla;
  ASSERT_TRUE(TryFillEosSceneTargetFromKinematics(11U, lla_kinematics, anchor, appearance,
                                                  &target_from_lla));
  EXPECT_EQ(target_from_lla.target_id, 11U);
  // 目标在锚点正东（经度 +0.001°）：ENU 东向分量主导，北/天近零。
  EXPECT_GT(target_from_lla.position_x, 100.0f);
  EXPECT_NEAR(target_from_lla.position_y, 0.0f, 1.0f);
  EXPECT_NEAR(target_from_lla.position_z, 0.0f, 1.0f);
  EXPECT_FLOAT_EQ(target_from_lla.appearance.apparent_temperature_k, 320.0f);

  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));
  oneq::coordinate::ExternalKinematics ecef_kinematics;
  ecef_kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  ecef_kinematics.position_ecef_m = target_ecef;
  ::electro_optical_sensor::session::EosSceneTarget target_from_ecef;
  ASSERT_TRUE(TryFillEosSceneTargetFromKinematics(12U, ecef_kinematics, anchor, appearance,
                                                  &target_from_ecef));
  EXPECT_NEAR(target_from_ecef.position_x, target_from_lla.position_x, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.position_y, target_from_lla.position_y, 1.0e-3f);
  EXPECT_NEAR(target_from_ecef.position_z, target_from_lla.position_z, 1.0e-3f);
}

// 速度经公共一站式入口旋转到锚点 ENU 轴（零速度直通）。
TEST(EosCoordinateUtilsTest, VelocityIsRotatedIntoAnchorEnuAxes) {
  const oneq::coordinate::LlaPositionDegM anchor = MakeAnchor();

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;
  oneq::coordinate::ExternalKinematics kinematics;
  kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  kinematics.position_lla_deg_m = target_lla;
  kinematics.velocity_mps.x_mps = 12.0;
  kinematics.velocity_mps.y_mps = -7.0;
  kinematics.velocity_mps.z_mps = 3.5;

  ::electro_optical_sensor::session::EosTargetAppearance appearance;
  ::electro_optical_sensor::session::EosSceneTarget target;
  ASSERT_TRUE(TryFillEosSceneTargetFromKinematics(21U, kinematics, anchor, appearance, &target));
  // 锚点 (lat=0,lon=0)：ECEF x→ENU 上、y→东、z→北，速度按同映射旋转。
  EXPECT_NEAR(target.velocity_x, -7.0f, 0.2f);
  EXPECT_NEAR(target.velocity_y, 3.5f, 0.2f);
  EXPECT_NEAR(target.velocity_z, 12.0f, 0.2f);
}

TEST(EosCoordinateUtilsTest, ReportsFailureForNullOutputAndBadVelocity) {
  const oneq::coordinate::LlaPositionDegM anchor = MakeAnchor();
  oneq::coordinate::ExternalKinematics kinematics;
  ::electro_optical_sensor::session::EosTargetAppearance appearance;
  EXPECT_FALSE(TryFillEosSceneTargetFromKinematics(1U, kinematics, anchor, appearance, nullptr));

  // 锚点原点目标 + 非有限速度：一站式转换数值失败。
  kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  kinematics.position_lla_deg_m = anchor;
  kinematics.velocity_mps.x_mps = std::numeric_limits<double>::quiet_NaN();
  ::electro_optical_sensor::session::EosSceneTarget target;
  EXPECT_FALSE(TryFillEosSceneTargetFromKinematics(1U, kinematics, anchor, appearance, &target));
}

}  // namespace
}  // namespace utils
}  // namespace electro_optical_sensor
