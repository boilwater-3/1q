/**
 * @file eos_cycle_input_builder_test.cpp
 * @brief 验证 EosCycleInputAdapter 一步构建与原始两步适配器的等价一致性。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"

namespace electro_optical_sensor {
namespace session {
namespace {

/// @brief Builder 生成结果与两步式适配器对同一输入产生相同输出。
TEST(EosCycleInputBuilderTest, BuilderMatchesTwoStepAdapter) {
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

  // 两步式适配器（参考基准）
  oneq::foundation::PoseState pose_2step;
  ASSERT_TRUE(TryMakeEosPoseFromExternalKinematics(pose_input, reference, &pose_2step));

  EosExternalTargetInput ext_target;
  ext_target.target_name = "eos-builder-target";
  ext_target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  ext_target.kinematics.position_ecef_m = target_ecef;
  ext_target.appearance.apparent_temperature_k = 300.0f;
  ext_target.appearance.emissivity = 0.8f;
  ext_target.appearance.reflectance = 0.3f;
  ext_target.appearance.projected_area_m2 = 2.5f;

  EosSceneTarget target_2step;
  ASSERT_TRUE(TryMakeEosSceneTargetFromExternalInput(100U, ext_target, reference, pose_2step,
                                                     &target_2step));

  // Builder（一步构建）
  EosCycleInput builder_input;
  ASSERT_TRUE(EosCycleInputAdapter::Build(pose_input, {ext_target}, 1.0f, &builder_input));

  ASSERT_EQ(builder_input.scene.size(), 1U);

  const auto& builder_target = builder_input.scene[0];
  EXPECT_EQ(builder_target.target_id, 0U);  // Builder 使用索引作为 ID
  EXPECT_EQ(builder_target.target_name, "eos-builder-target");
  EXPECT_NEAR(builder_target.range_m, target_2step.range_m, 1.0e-4f);
  EXPECT_NEAR(builder_target.azimuth_deg, target_2step.azimuth_deg, 1.0e-4f);
  EXPECT_NEAR(builder_target.elevation_deg, target_2step.elevation_deg, 1.0e-4f);
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
  EXPECT_NEAR(input.platform_pose.attitude_deg.yaw_deg, 10.0f, 1.0e-5f);
  EXPECT_TRUE(input.scene.empty());
}

/// @brief Builder 显式环境重载写入完整环境快照。
TEST(EosCycleInputBuilderTest, ExplicitEnvironmentSnapshotIsCopiedToCycleInput) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.0;
  origin_lla.longitude_deg = 121.0;
  origin_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  EosExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;

  EosEnvironmentInput environment;
  environment.solar_altitude_deg = 12.0f;
  environment.cloud_coverage_ratio = 0.7f;
  environment.day_night_type = DayNightType::kTwilight;

  EosCycleInput input;
  ASSERT_TRUE(EosCycleInputAdapter::Build(pose_input, {}, 1.0f, environment, &input));

  EXPECT_FLOAT_EQ(input.environment.solar_altitude_deg, 12.0f);
  EXPECT_FLOAT_EQ(input.platform_altitude_m, 1000.0f);
  EXPECT_FLOAT_EQ(input.environment.cloud_coverage_ratio, 0.7f);
  EXPECT_EQ(input.environment.day_night_type, DayNightType::kTwilight);
}

/// @brief 环境输入状态只更新 patch 标记过的字段。
TEST(EosCycleInputBuilderTest, EnvironmentInputStateAppliesOnlyFlaggedFields) {
  EosEnvironmentInput initial;
  initial.solar_altitude_deg = 45.0f;
  initial.cloud_coverage_ratio = 0.1f;
  initial.background_temperature_k = 280.0f;

  EosEnvironmentInputState state(initial);

  EosEnvironmentInputPatch patch;
  patch.has_cloud_coverage_ratio = true;
  patch.cloud_coverage_ratio = 0.8f;
  patch.background_temperature_k = 310.0f;
  state.Update(patch);

  const EosEnvironmentInput snapshot = state.Snapshot();
  EXPECT_FLOAT_EQ(snapshot.solar_altitude_deg, 45.0f);
  EXPECT_FLOAT_EQ(snapshot.cloud_coverage_ratio, 0.8f);
  EXPECT_FLOAT_EQ(snapshot.background_temperature_k, 280.0f);
}

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
  EXPECT_GT(input.scene[0].range_m, 0.0f);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
