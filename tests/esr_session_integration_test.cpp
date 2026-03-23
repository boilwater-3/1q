/**
 * @file esr_session_integration_test.cpp
 * @brief 验证 ESR Session 单周期闭环与干扰退化方向。
 */

#include <gtest/gtest.h>

#include <algorithm>

#include "1q/electronic_surveillance_radar/common/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {
namespace {

/**
 * @brief 构造可被 X 波段接收窗口截获的最小场景输入。
 * @return 最小可运行输入。
 */
context::EsrCycleInput MakeBaseInput() {
  context::EsrCycleInput input;
  input.cycle_index = 4U;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.x = 0.0f;
  input.platform_pose.position_m.y = 0.0f;
  input.platform_pose.position_m.z = 5000.0f;
  input.environment_scene_state.base_propagation_loss_db = 3.0f;
  input.environment_scene_state.atmospheric_attenuation_db = 1.0f;
  input.environment_scene_state.terrain_reflection_db = 0.0f;
  input.environment_scene_state.clutter_noise_w = 1.0e-12f;

  common::EmitterTruthState emitter;
  emitter.emitter_id = "target-emitter";
  emitter.pose.position_m.x = 1200.0f;
  emitter.pose.position_m.y = 0.0f;
  emitter.pose.position_m.z = 5200.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.2e-6;
  emitter.pri_s = 1.0e-4;
  emitter.is_emitting = true;
  input.scene_emitters.push_back(emitter);
  return input;
}

/**
 * @brief 构造可保证波束覆盖的会话配置。
 * @return 会话配置。
 */
EsrSessionConfig MakeSessionConfig() {
  EsrSessionConfig config;
  config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  config.pipeline_config.scan.scan_start_el_deg = -20.0f;
  config.pipeline_config.scan.scan_end_el_deg = 20.0f;
  config.pipeline_config.scan.az_step_deg = 120.0f;
  config.pipeline_config.scan.el_step_deg = 40.0f;
  config.pipeline_config.scan.scan_start_pos = 0;
  config.pipeline_config.scan.scan_sequence = 0;
  config.pipeline_config.detection.min_detect_snr_db = 6.0f;
  config.pipeline_config.detection.max_detect_range_m = 400000.0f;
  config.pipeline_config.detection.boundary_resolution_m = 25.0f;
  config.pipeline_config.detection.boundary_max_iterations = 32;
  config.pipeline_config.algorithm.random_seed = 20260323U;
  return config;
}

TEST(EsrSessionIntegrationTest, StepWithResultProducesThreeChannelOutput) {
  EsrSession session(MakeSessionConfig());
  const context::EsrCycleInput input = MakeBaseInput();

  const EsrCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_FALSE(result.output_frame.observation_output.observations.empty());
  EXPECT_EQ(result.output_frame.observation_output.cycle_index,
            input.cycle_index);
  EXPECT_EQ(result.output_frame.emitter_output.cycle_index, input.cycle_index);
  EXPECT_EQ(result.output_frame.truth_evaluation_output.cycle_index,
            input.cycle_index);
  EXPECT_FALSE(result.output_frame.emitter_output.hypotheses.empty());
  EXPECT_LE(result.output_frame.emitter_output.hypotheses.size(),
            result.output_frame.observation_output.observations.size());
  EXPECT_GE(result.output_frame.truth_evaluation_output.associations.size(),
            result.output_frame.observation_output.observations.size());
}

TEST(EsrSessionIntegrationTest, JammingDegradesObservationQualityAndConfidence) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput no_jam_input = MakeBaseInput();
  context::EsrCycleInput jam_input = no_jam_input;
  jam_input.cycle_index = no_jam_input.cycle_index;

  environment::EsrJammerSource jammer;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 3.0e9;
  jammer.power_w = 1.0e-4f;
  jammer.confidence = 1.0f;
  jammer.deception_risk = 0.3f;
  jam_input.environment_scene_state.jammer_sources.push_back(jammer);

  const EsrCycleResult no_jam_result = session.StepWithResult(no_jam_input);
  const EsrCycleResult jam_result = session.StepWithResult(jam_input);

  ASSERT_FALSE(no_jam_result.output_frame.observation_output.observations.empty());
  ASSERT_FALSE(jam_result.output_frame.observation_output.observations.empty());
  ASSERT_FALSE(no_jam_result.output_frame.emitter_output.hypotheses.empty());
  ASSERT_FALSE(jam_result.output_frame.emitter_output.hypotheses.empty());

  const float no_jam_snr =
      no_jam_result.output_frame.observation_output.observations.front().snr_db;
  const float jam_snr =
      jam_result.output_frame.observation_output.observations.front().snr_db;
  const float no_jam_confidence =
      no_jam_result.output_frame.emitter_output.hypotheses.front().confidence;
  const float jam_confidence =
      jam_result.output_frame.emitter_output.hypotheses.front().confidence;

  EXPECT_LE(jam_snr, no_jam_snr);
  EXPECT_LE(jam_confidence, no_jam_confidence);
}

TEST(EsrSessionIntegrationTest, EmptyEmitterSceneReturnsEmptyObservation) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput input = MakeBaseInput();
  input.scene_emitters.clear();

  const EsrCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.output_frame.observation_output.observations.empty());
  EXPECT_TRUE(result.output_frame.emitter_output.hypotheses.empty());
  EXPECT_TRUE(result.output_frame.truth_evaluation_output.associations.empty());
}

TEST(EsrSessionIntegrationTest, MultiEmitterSceneProducesClusteredHypotheses) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput input = MakeBaseInput();

  common::EmitterTruthState emitter_2 = input.scene_emitters.front();
  emitter_2.emitter_id = "target-emitter-2";
  emitter_2.pose.position_m.y = 200.0f;
  emitter_2.carrier_hz = 10.2e9;
  emitter_2.bandwidth_hz = 1.5e6;
  emitter_2.tx_power_w = 3.0e7;
  input.scene_emitters.push_back(emitter_2);

  const EsrCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_GE(result.output_frame.observation_output.observations.size(), 2U);
  EXPECT_GE(result.output_frame.emitter_output.hypotheses.size(), 1U);
  EXPECT_LE(result.output_frame.emitter_output.hypotheses.size(),
            result.output_frame.observation_output.observations.size());
}

}  // namespace
}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar
