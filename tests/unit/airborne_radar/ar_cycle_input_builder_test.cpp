/**
 * @file ar_cycle_input_builder_test.cpp
 * @brief 验证 ArCycleInputAdapter 一步构建与原始两步适配器的等价一致性。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "1q/airborne_radar/session/ArCycleInputAdapter.h"
#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/coordinate/position_transform.h"

namespace airborne_radar {
namespace session {
namespace {

/// @brief Builder 生成结果与两步式适配器对同一输入产生相同输出。
TEST(RadarCycleInputBuilderTest, BuilderMatchesTwoStepAdapter) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 31.2304;
  radar_lla.longitude_deg = 121.4737;
  radar_lla.altitude_m = 200.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  oneq::coordinate::LlaPositionDegM target_lla = radar_lla;
  target_lla.longitude_deg += 0.001;
  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  // 准备外部输入
  ArExternalPoseInput pose_input;
  pose_input.platform_entity_id = 42U;
  pose_input.platform_position_ecef_m = radar_ecef;
  pose_input.platform_velocity_mps.x_mps = 120.0f;
  pose_input.platform_velocity_mps.y_mps = -70.0f;
  pose_input.platform_velocity_mps.z_mps = 30.0f;
  pose_input.platform_attitude_deg.yaw_deg = 5.0f;
  pose_input.radar_mount_angles_deg.pitch_deg = 2.0f;

  ArExternalTargetInput target_input;
  target_input.target_id = 600U;
  target_input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target_input.kinematics.position_ecef_m = target_ecef;
  target_input.kinematics.velocity_mps.x_mps = 120.0f;
  target_input.kinematics.velocity_mps.y_mps = -70.0f;
  target_input.kinematics.velocity_mps.z_mps = 30.0f;
  target_input.rcs = 1.2f;
  target_input.swerling_type = 0;

  // 两步式适配器（参考基准）
  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::PoseState pose_2step;
  ASSERT_TRUE(TryMakeArPoseFromExternalKinematics(pose_input, &reference, &pose_2step));

  ArSceneTarget target_2step;
  ASSERT_TRUE(TryMakeArTargetFromExternalKinematics(target_input, reference,
                                                    pose_2step.velocity_mps, &target_2step));

  // Builder（一步构建）
  ArCycleInput builder_input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(pose_input, {target_input}, 1.0f, &builder_input));

  EXPECT_FALSE(builder_input.has_environment);
  EXPECT_EQ(builder_input.platform_entity_id, 42U);
  EXPECT_TRUE(builder_input.has_platform_ecef_kinematics);
  EXPECT_DOUBLE_EQ(builder_input.platform_position_ecef_m.x_m, radar_ecef.x_m);
  EXPECT_DOUBLE_EQ(builder_input.platform_position_ecef_m.y_m, radar_ecef.y_m);
  EXPECT_DOUBLE_EQ(builder_input.platform_position_ecef_m.z_m, radar_ecef.z_m);
  EXPECT_DOUBLE_EQ(builder_input.platform_velocity_ecef_mps.x_mps,
                   pose_input.platform_velocity_mps.x_mps);
  ASSERT_EQ(builder_input.scene.size(), 1U);

  const auto& builder_target = builder_input.scene[0];
  EXPECT_EQ(builder_target.external_target_id, 600U);  // Builder 从字段读取调用方设置的 ID
  EXPECT_NEAR(builder_target.position_x, target_2step.position_x, 1.0e-6f);
  EXPECT_NEAR(builder_target.position_y, target_2step.position_y, 1.0e-6f);
  EXPECT_NEAR(builder_target.position_z, target_2step.position_z, 1.0e-6f);
  EXPECT_NEAR(builder_target.velocity_x, target_2step.velocity_x, 1.0e-6f);
  EXPECT_NEAR(builder_target.velocity_y, target_2step.velocity_y, 1.0e-6f);
  EXPECT_NEAR(builder_target.velocity_z, target_2step.velocity_z, 1.0e-6f);
  EXPECT_NEAR(builder_target.range_m, target_2step.range_m, 1.0e-6f);
  EXPECT_NEAR(builder_target.rcs, target_2step.rcs, 1.0e-6f);
  EXPECT_EQ(builder_target.target_swerling_type, target_2step.target_swerling_type);
}

/// @brief Builder 接受空目标列表，platform_pose 依然正确。
TEST(RadarCycleInputBuilderTest, EmptyTargetsProducesValidCycleInput) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 31.0;
  radar_lla.longitude_deg = 121.0;
  radar_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  ArExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = radar_ecef;
  pose_input.platform_attitude_deg.yaw_deg = 10.0f;
  pose_input.platform_attitude_deg.pitch_deg = -3.0f;
  pose_input.platform_attitude_deg.roll_deg = 1.0f;

  ArCycleInput input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(pose_input, {}, 2.0f, &input));
  EXPECT_EQ(input.cycle_index, 0U);
  EXPECT_FLOAT_EQ(input.dt_sec, 2.0f);
  EXPECT_FLOAT_EQ(input.platform_altitude_m, 1000.0f);
  EXPECT_FLOAT_EQ(input.platform_pose.position_m.x, 0.0f);
  EXPECT_FLOAT_EQ(input.platform_pose.position_m.y, 0.0f);
  EXPECT_FLOAT_EQ(input.platform_pose.position_m.z, 0.0f);
  EXPECT_NEAR(input.platform_pose.attitude_deg.yaw_deg, 10.0f, 1.0e-5f);
  EXPECT_NEAR(input.platform_pose.attitude_deg.pitch_deg, -3.0f, 1.0e-5f);
  EXPECT_NEAR(input.platform_pose.attitude_deg.roll_deg, 1.0f, 1.0e-5f);
  EXPECT_TRUE(input.scene.empty());
}

/// @brief Builder 显式环境重载写入完整环境快照。
TEST(RadarCycleInputBuilderTest, ExplicitEnvironmentSnapshotIsCopiedToCycleInput) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 31.0;
  radar_lla.longitude_deg = 121.0;
  radar_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  ArExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = radar_ecef;

  ArEnvironmentInput environment;
  environment.atmospheric_observation.enable_physical_model = true;
  environment.atmospheric_observation.temperature_k = 301.0f;
  environment.atmospheric_context.solar_flux_f107 = 180.0f;
  environment.surface_observation.cover_profile = config::VegetationCoverProfile::kSparseWoodland;
  environment.jammer_sources.push_back(config::JammerEmitterState{});
  environment.jammer_sources[0].power_db = 12.0f;

  ArCycleInput input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(pose_input, {}, 1.0f, environment, &input));

  EXPECT_FLOAT_EQ(input.platform_altitude_m, 1000.0f);
  EXPECT_TRUE(input.has_environment);
  EXPECT_TRUE(input.environment.atmospheric_observation.enable_physical_model);
  EXPECT_FLOAT_EQ(input.environment.atmospheric_observation.temperature_k, 301.0f);
  EXPECT_FLOAT_EQ(input.environment.atmospheric_context.solar_flux_f107, 180.0f);
  EXPECT_EQ(input.environment.surface_observation.cover_profile,
            config::VegetationCoverProfile::kSparseWoodland);
  ASSERT_EQ(input.environment.jammer_sources.size(), 1U);
  EXPECT_FLOAT_EQ(input.environment.jammer_sources[0].power_db, 12.0f);
}

/// @brief 环境输入状态只更新 patch 标记过的字段。
TEST(RadarCycleInputBuilderTest, EnvironmentInputStateAppliesOnlyFlaggedFields) {
  ArEnvironmentInput initial;
  initial.atmospheric_observation.temperature_k = 288.0f;
  initial.atmospheric_context.solar_flux_f107 = 150.0f;

  ArEnvironmentInputState state(initial);

  ArEnvironmentInputPatch patch;
  patch.has_atmospheric_context = true;
  patch.atmospheric_context.solar_flux_f107 = 210.0f;
  patch.atmospheric_observation.temperature_k = 310.0f;
  state.Update(patch);

  const ArEnvironmentInput snapshot = state.Snapshot();
  EXPECT_FLOAT_EQ(snapshot.atmospheric_observation.temperature_k, 288.0f);
  EXPECT_FLOAT_EQ(snapshot.atmospheric_context.solar_flux_f107, 210.0f);
}

/// @brief Builder 在 nullptr 输出时返回 false。
TEST(RadarCycleInputBuilderTest, NullOutputReturnsFalse) {
  ArExternalPoseInput pose_input;
  EXPECT_FALSE(ArCycleInputAdapter::Build(pose_input, {}, 1.0f, nullptr));
}

/// @brief Builder 多个目标。
TEST(RadarCycleInputBuilderTest, MultipleTargets) {
  oneq::coordinate::LlaPositionDegM radar_lla;
  radar_lla.latitude_deg = 31.0;
  radar_lla.longitude_deg = 121.0;
  radar_lla.altitude_m = 500.0;
  oneq::coordinate::EcefPositionM radar_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(radar_lla, &radar_ecef));

  ArExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = radar_ecef;

  std::vector<ArExternalTargetInput> targets(3);
  for (int i = 0; i < 3; ++i) {
    auto& t = targets[i];
    t.target_id = static_cast<std::uint64_t>(i) + 1U;
    t.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    t.kinematics.position_ecef_m = radar_ecef;
    t.kinematics.position_ecef_m.x_m += static_cast<double>(i + 1) * 1000.0;
    t.rcs = static_cast<float>(i + 1);
  }

  ArCycleInput input;
  ASSERT_TRUE(ArCycleInputAdapter::Build(pose_input, targets, 0.5f, &input));
  ASSERT_EQ(input.scene.size(), 3U);
  EXPECT_FLOAT_EQ(input.dt_sec, 0.5f);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(input.scene[i].external_target_id, static_cast<std::uint64_t>(i) + 1U);
    EXPECT_GT(input.scene[i].range_m, 0.0f);
  }
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
