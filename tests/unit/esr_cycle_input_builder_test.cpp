/**
 * @file esr_cycle_input_builder_test.cpp
 * @brief 验证 EsrCycleInputBuilder 一步构建与原始两步适配器的等价一致性。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "1q/coordinate/position_transform.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInputBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInputState.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

/// @brief Builder 生成结果与两步式适配器对同一输入产生相同输出。
TEST(EsrCycleInputBuilderTest, BuilderMatchesTwoStepAdapter) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.2304;
  origin_lla.longitude_deg = 121.4737;
  origin_lla.altitude_m = 200.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  oneq::coordinate::LlaPositionDegM emitter_lla = origin_lla;
  emitter_lla.longitude_deg += 0.002;
  oneq::coordinate::EcefPositionM emitter_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(emitter_lla, &emitter_ecef));

  EsrCoordinateReference reference;
  reference.origin_lla = origin_lla;
  reference.frame_attitude_deg.yaw_deg = 5.0f;

  EsrExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;
  pose_input.platform_velocity_mps.x_mps = 100.0f;
  pose_input.platform_velocity_mps.y_mps = 50.0f;
  pose_input.platform_velocity_mps.z_mps = 20.0f;
  pose_input.platform_attitude_deg.yaw_deg = 5.0f;

  // 两步式适配器（参考基准）
  EsrPoseState pose_2step;
  ASSERT_TRUE(TryMakeEsrPoseFromExternalKinematics(pose_input, reference, &pose_2step));

  EsrExternalEmitterInput ext_emitter;
  ext_emitter.emitter_id = "emi_1";
  ext_emitter.emitter_position_ecef_m = emitter_ecef;
  ext_emitter.emitter_velocity_mps.x_mps = 100.0f;
  ext_emitter.emitter_velocity_mps.y_mps = 50.0f;
  ext_emitter.emitter_velocity_mps.z_mps = 20.0f;
  ext_emitter.carrier_hz = 10.0e9;
  ext_emitter.bandwidth_hz = 1.0e6;
  ext_emitter.tx_power_w = 1000.0;
  ext_emitter.pulse_width_s = 1.0e-6;
  ext_emitter.pri_s = 1.0e-3;
  ext_emitter.is_emitting = true;

  EsrSceneEmitter emitter_2step;
  ASSERT_TRUE(TryMakeEsrSceneEmitterFromExternalInput(ext_emitter, reference, &emitter_2step));

  // Builder（一步构建）
  EsrCycleInput builder_input;
  ASSERT_TRUE(EsrCycleInputBuilder::Build(pose_input, {ext_emitter}, 1.0f, &builder_input));

  ASSERT_EQ(builder_input.scene.size(), 1U);

  const auto& builder_emitter = builder_input.scene[0];
  EXPECT_EQ(builder_emitter.emitter_id, "emi_1");
  EXPECT_NEAR(builder_emitter.pose.position_m.x, emitter_2step.pose.position_m.x, 1.0e-5f);
  EXPECT_NEAR(builder_emitter.pose.position_m.y, emitter_2step.pose.position_m.y, 1.0e-5f);
  EXPECT_NEAR(builder_emitter.pose.position_m.z, emitter_2step.pose.position_m.z, 1.0e-5f);
  EXPECT_NEAR(builder_emitter.pose.velocity_mps.x, emitter_2step.pose.velocity_mps.x, 1.0e-5f);
  EXPECT_NEAR(builder_emitter.pose.velocity_mps.y, emitter_2step.pose.velocity_mps.y, 1.0e-5f);
  EXPECT_NEAR(builder_emitter.pose.velocity_mps.z, emitter_2step.pose.velocity_mps.z, 1.0e-5f);
  EXPECT_DOUBLE_EQ(builder_emitter.carrier_hz, ext_emitter.carrier_hz);
  EXPECT_DOUBLE_EQ(builder_emitter.bandwidth_hz, ext_emitter.bandwidth_hz);
  EXPECT_DOUBLE_EQ(builder_emitter.tx_power_w, ext_emitter.tx_power_w);
  EXPECT_DOUBLE_EQ(builder_emitter.pulse_width_s, ext_emitter.pulse_width_s);
  EXPECT_DOUBLE_EQ(builder_emitter.pri_s, ext_emitter.pri_s);
  EXPECT_TRUE(builder_emitter.is_emitting);
}

/// @brief Builder 接受空辐射源列表。
TEST(EsrCycleInputBuilderTest, EmptyEmittersProducesValidCycleInput) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.0;
  origin_lla.longitude_deg = 121.0;
  origin_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  EsrExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;
  pose_input.platform_attitude_deg.yaw_deg = 10.0f;

  EsrCycleInput input;
  ASSERT_TRUE(EsrCycleInputBuilder::Build(pose_input, {}, 2.0f, &input));

  EXPECT_EQ(input.cycle_index, 0U);
  EXPECT_FLOAT_EQ(input.dt_sec, 2.0f);
  EXPECT_NEAR(input.platform_pose.attitude_deg.yaw_deg, 10.0f, 1.0e-5f);
  EXPECT_TRUE(input.scene.empty());
}

/// @brief Builder 显式环境重载写入完整环境快照。
TEST(EsrCycleInputBuilderTest, ExplicitEnvironmentSnapshotIsCopiedToCycleInput) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.0;
  origin_lla.longitude_deg = 121.0;
  origin_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  EsrExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;

  EsrEnvironmentInput environment;
  environment.propagation_profile = environment::EsrPropagationEnvironmentProfile::kComplex;
  environment.spectrum_occupancy_ratio = 0.6f;
  environment.atmospheric_observation.visibility_km = 8.0f;

  EsrCycleInput input;
  ASSERT_TRUE(EsrCycleInputBuilder::Build(pose_input, {}, 1.0f, environment, &input));

  EXPECT_EQ(input.environment.propagation_profile,
            environment::EsrPropagationEnvironmentProfile::kComplex);
  EXPECT_FLOAT_EQ(input.environment.spectrum_occupancy_ratio, 0.6f);
  EXPECT_FLOAT_EQ(input.environment.atmospheric_observation.visibility_km, 8.0f);
}

/// @brief 环境输入状态只更新 patch 标记过的字段。
TEST(EsrCycleInputBuilderTest, EnvironmentInputStateAppliesOnlyFlaggedFields) {
  EsrEnvironmentInput initial;
  initial.propagation_profile = environment::EsrPropagationEnvironmentProfile::kOpen;
  initial.spectrum_occupancy_ratio = 0.1f;
  initial.atmospheric_observation.visibility_km = 30.0f;

  EsrEnvironmentInputState state(initial);

  EsrEnvironmentInputPatch patch;
  patch.has_spectrum_occupancy_ratio = true;
  patch.spectrum_occupancy_ratio = 0.9f;
  patch.atmospheric_observation.visibility_km = 5.0f;
  state.Update(patch);

  const EsrEnvironmentInput snapshot = state.Snapshot();
  EXPECT_EQ(snapshot.propagation_profile, environment::EsrPropagationEnvironmentProfile::kOpen);
  EXPECT_FLOAT_EQ(snapshot.spectrum_occupancy_ratio, 0.9f);
  EXPECT_FLOAT_EQ(snapshot.atmospheric_observation.visibility_km, 30.0f);
}

/// @brief Builder 在 nullptr 输出时返回 false 并设置 status。
TEST(EsrCycleInputBuilderTest, NullOutputReturnsFalse) {
  EsrExternalPoseInput pose_input;
  EsrCoordinateStatus status = EsrCoordinateStatus::kOk;
  EXPECT_FALSE(EsrCycleInputBuilder::Build(pose_input, {}, 1.0f, nullptr, &status));
  EXPECT_EQ(status, EsrCoordinateStatus::kNullOutput);
}

/// @brief Builder 多个辐射源。
TEST(EsrCycleInputBuilderTest, MultipleEmitters) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 31.0;
  origin_lla.longitude_deg = 121.0;
  origin_lla.altitude_m = 500.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  EsrExternalPoseInput pose_input;
  pose_input.platform_position_ecef_m = origin_ecef;

  std::vector<EsrExternalEmitterInput> emitters(3);
  for (int i = 0; i < 3; ++i) {
    auto& e = emitters[i];
    e.emitter_id = std::string("emi_") + std::to_string(i);
    e.emitter_position_ecef_m = origin_ecef;
    e.emitter_position_ecef_m.x_m += static_cast<double>(i + 1) * 1000.0;
    e.carrier_hz = 10.0e9 + static_cast<double>(i) * 1.0e6;
    e.is_emitting = true;
  }

  EsrCycleInput input;
  ASSERT_TRUE(EsrCycleInputBuilder::Build(pose_input, emitters, 0.5f, &input));

  ASSERT_EQ(input.scene.size(), 3U);
  EXPECT_FLOAT_EQ(input.dt_sec, 0.5f);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(input.scene[i].emitter_id, std::string("emi_") + std::to_string(i));
    EXPECT_TRUE(input.scene[i].is_emitting);
  }
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
