/**
 * @file eos_cycle_input_builder_test.cpp
 * @brief 验证 EosCycleInputAdapter 一步构建与原始两步适配器的等价一致性。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"

namespace electro_optical_sensor {
namespace session {
namespace {

/// @brief Builder 生成结果与单目标适配器对同一输入产生相同输出。
TEST(EosCycleInputBuilderTest, BuilderMatchesSingleTargetAdapter) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.2304;
  origin_lla.longitude_deg = 121.4737;
  origin_lla.altitude_m = 200.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  oneq::coordinate::LlaPositionDegM target_lla = origin_lla;
  target_lla.longitude_deg += 0.001;
  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  oneq::coordinate::LocalFrameReference reference;
  reference.origin_lla = origin_lla;
  reference.frame_attitude_deg.yaw_deg = 5.0f;

  EosExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;
  pose_input.platform_velocity_mps.x_mps = 120.0f;
  pose_input.platform_velocity_mps.y_mps = -70.0f;
  pose_input.platform_velocity_mps.z_mps = 30.0f;
  pose_input.platform_attitude_deg.yaw_deg = 5.0f;

  // 单目标适配器（参考基准）
  EosExternalTargetInput ext_target;
  ext_target.target_name = "eos-builder-target";
  ext_target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  ext_target.kinematics.position_ecef_m = target_ecef;
  ext_target.appearance.apparent_temperature_k = 300.0f;
  ext_target.appearance.emissivity = 0.8f;
  ext_target.appearance.reflectance = 0.3f;
  ext_target.appearance.projected_area_m2 = 2.5f;

  EosSceneTarget target_single_step;
  ASSERT_TRUE(TryMakeEosSceneTargetFromExternalInput(100U, ext_target, reference,
                                                     &target_single_step));

  // Builder（一步构建）
  EosCycleInput builder_input;
  ASSERT_TRUE(EosCycleInputAdapter::Build(pose_input, {ext_target}, 1.0f, &builder_input));

  ASSERT_EQ(builder_input.scene.size(), 1U);

  const auto& builder_target = builder_input.scene[0];
  EXPECT_EQ(builder_target.target_id, 0U);  // Builder 使用索引作为 ID
  EXPECT_EQ(builder_target.target_name, "eos-builder-target");
  EXPECT_NEAR(builder_target.position_x, target_single_step.position_x, 1.0e-4f);
  EXPECT_NEAR(builder_target.position_y, target_single_step.position_y, 1.0e-4f);
  EXPECT_NEAR(builder_target.position_z, target_single_step.position_z, 1.0e-4f);
  EXPECT_NEAR(builder_target.velocity_x, target_single_step.velocity_x, 1.0e-4f);
  EXPECT_NEAR(builder_target.velocity_y, target_single_step.velocity_y, 1.0e-4f);
  EXPECT_NEAR(builder_target.velocity_z, target_single_step.velocity_z, 1.0e-4f);
  EXPECT_FLOAT_EQ(builder_target.appearance.apparent_temperature_k,
                  ext_target.appearance.apparent_temperature_k);
  EXPECT_FLOAT_EQ(builder_target.appearance.emissivity, ext_target.appearance.emissivity);
  EXPECT_FLOAT_EQ(builder_target.appearance.reflectance, ext_target.appearance.reflectance);
  EXPECT_FLOAT_EQ(builder_target.appearance.projected_area_m2,
                  ext_target.appearance.projected_area_m2);
}

/// @brief Builder 接受空目标列表。
TEST(EosCycleInputBuilderTest, EmptyTargetsProducesValidCycleInput) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.0;
  origin_lla.longitude_deg = 121.0;
  origin_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  EosExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;
  pose_input.platform_attitude_deg.yaw_deg = 10.0f;

  EosCycleInput input;
  ASSERT_TRUE(EosCycleInputAdapter::Build(pose_input, {}, 2.0f, &input));

  EXPECT_EQ(input.cycle_index, 0U);
  EXPECT_FLOAT_EQ(input.dt_sec, 2.0f);
  EXPECT_FLOAT_EQ(input.platform_altitude_m, 1000.0f);
  EXPECT_NEAR(input.platform_attitude_deg.yaw_deg, 10.0, 1.0e-5);
  EXPECT_TRUE(input.scene.empty());
}

/// @brief Builder 显式环境重载写入完整环境快照。
/// @brief Builder 在 nullptr 输出时返回 false 并设置 status。
TEST(EosCycleInputBuilderTest, NullOutputReturnsFalse) {
  EosExternalPoseInput pose_input;
  EosCoordinateStatus status = EosCoordinateStatus::kOk;
  EXPECT_FALSE(EosCycleInputAdapter::Build(pose_input, {}, 1.0f, nullptr, &status));
  EXPECT_EQ(status, EosCoordinateStatus::kNullOutput);
}

/// @brief Builder 等价的 LLA 目标位置。
TEST(EosCycleInputBuilderTest, LlaTargetPosition) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.0;
  origin_lla.longitude_deg = 121.0;
  origin_lla.altitude_m = 500.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  EosExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;
  pose_input.platform_attitude_deg.yaw_deg = 0.0f;

  EosExternalTargetInput ext_target;
  ext_target.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  ext_target.kinematics.position_lla_deg_m = origin_lla;
  ext_target.kinematics.position_lla_deg_m.latitude_deg += 0.001;

  EosCycleInput input;
  ASSERT_TRUE(EosCycleInputAdapter::Build(pose_input, {ext_target}, 1.0f, &input));

  ASSERT_EQ(input.scene.size(), 1U);
  EXPECT_GT(input.scene[0].position_x + std::fabs(input.scene[0].position_y) +
                std::fabs(input.scene[0].position_z),
            0.0f);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
