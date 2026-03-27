/**
 * @file esr_session_integration_test.cpp
 * @brief 验证 ESR Session 单周期闭环与干扰退化方向。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

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
  config.pipeline_config.statistical_detection.enable_statistical_detection = true;
  config.pipeline_config.statistical_detection.pfa = 1.0e-6f;
  config.pipeline_config.statistical_detection.min_snr_db = 6.0f;
  config.pipeline_config.statistical_detection.pulse_count = 8U;
  config.pipeline_config.statistical_detection.integration_mode =
      pipeline::InterceptIntegrationMode::kNonCoherent;
  config.pipeline_config.statistical_detection.threshold_scale = 1.0f;
  config.pipeline_config.algorithm.random_seed = 20260323U;
  config.pipeline_config.deception_model.false_alarm_probability_scale = 1.0f;
  config.pipeline_config.deception_model.max_false_observations_per_emitter = 2U;
  return config;
}

/**
 * @brief 统计评估通道中的伪观测数量。
 * @param[in] result 周期结果。
 * @return 伪观测数量。
 */
std::size_t CountFalseAlarms(const EsrCycleResult& result) {
  std::size_t false_alarm_count = 0U;
  for (std::size_t i = 0; i < result.output_frame.truth_evaluation_output.associations.size();
       ++i) {
    const common::TruthAssociationRecord& association =
        result.output_frame.truth_evaluation_output.associations[i];
    if (!association.matched && association.truth_emitter_id == "UNASSOCIATED") {
      ++false_alarm_count;
    }
  }
  return false_alarm_count;
}

/**
 * @brief 统计指定真值在评估通道中的匹配次数。
 * @param[in] result 周期结果。
 * @param[in] truth_id 真值 ID。
 * @return 匹配次数。
 */
std::size_t CountMatchedTruthObservations(const EsrCycleResult& result,
                                          const std::string& truth_id) {
  std::size_t matched_count = 0U;
  for (std::size_t i = 0; i < result.output_frame.truth_evaluation_output.associations.size();
       ++i) {
    const common::TruthAssociationRecord& association =
        result.output_frame.truth_evaluation_output.associations[i];
    if (association.matched && association.truth_emitter_id == truth_id) {
      ++matched_count;
    }
  }
  return matched_count;
}

/**
 * @brief 获取与指定真值 ID 匹配的首条观测 SNR。
 * @param[in] result 周期结果。
 * @param[in] truth_id 真值 ID。
 * @return 匹配观测 SNR；未找到时返回 NaN。
 */
float FindMatchedTruthSnr(const EsrCycleResult& result, const std::string& truth_id) {
  std::uint64_t observation_id = 0U;
  for (std::size_t i = 0; i < result.output_frame.truth_evaluation_output.associations.size();
       ++i) {
    const common::TruthAssociationRecord& association =
        result.output_frame.truth_evaluation_output.associations[i];
    if (association.matched && association.truth_emitter_id == truth_id) {
      observation_id = association.observation_id;
      break;
    }
  }
  if (observation_id == 0U) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  for (std::size_t i = 0; i < result.output_frame.observation_output.observations.size(); ++i) {
    const common::EmitterObservation& observation =
        result.output_frame.observation_output.observations[i];
    if (observation.observation_id == observation_id) {
      return observation.snr_db;
    }
  }
  return std::numeric_limits<float>::quiet_NaN();
}

/**
 * @brief 计算假设平均置信度。
 * @param[in] result 周期结果。
 * @return 平均置信度。
 */
float AverageHypothesisConfidence(const EsrCycleResult& result) {
  const std::size_t hypothesis_count = result.output_frame.emitter_output.hypotheses.size();
  if (hypothesis_count == 0U) {
    return 0.0f;
  }
  float confidence_sum = 0.0f;
  for (std::size_t i = 0; i < hypothesis_count; ++i) {
    confidence_sum += result.output_frame.emitter_output.hypotheses[i].confidence;
  }
  return confidence_sum / static_cast<float>(hypothesis_count);
}

/**
 * @brief 统计含歧义类别的假设数量。
 * @param[in] result 周期结果。
 * @return 含 `AMBIGUOUS_CLASS` 的假设数量。
 */
std::size_t CountAmbiguousHypotheses(const EsrCycleResult& result) {
  std::size_t ambiguous_count = 0U;
  for (std::size_t i = 0; i < result.output_frame.emitter_output.hypotheses.size(); ++i) {
    const common::EmitterHypothesis& hypothesis = result.output_frame.emitter_output.hypotheses[i];
    if (std::find(hypothesis.candidate_classes.begin(), hypothesis.candidate_classes.end(),
                  "AMBIGUOUS_CLASS") != hypothesis.candidate_classes.end()) {
      ++ambiguous_count;
    }
  }
  return ambiguous_count;
}

/**
 * @brief 断言两个周期结果的观测/评估输出一致。
 * @param[in] lhs 左结果。
 * @param[in] rhs 右结果。
 */
void ExpectDeterministicResult(const EsrCycleResult& lhs, const EsrCycleResult& rhs) {
  ASSERT_EQ(lhs.output_frame.observation_output.observations.size(),
            rhs.output_frame.observation_output.observations.size());
  for (std::size_t i = 0; i < lhs.output_frame.observation_output.observations.size(); ++i) {
    const common::EmitterObservation& lhs_obs = lhs.output_frame.observation_output.observations[i];
    const common::EmitterObservation& rhs_obs = rhs.output_frame.observation_output.observations[i];
    EXPECT_EQ(lhs_obs.observation_id, rhs_obs.observation_id);
    EXPECT_DOUBLE_EQ(lhs_obs.timestamp_s, rhs_obs.timestamp_s);
    EXPECT_DOUBLE_EQ(lhs_obs.rf_hz, rhs_obs.rf_hz);
    EXPECT_DOUBLE_EQ(lhs_obs.pulse_width_s, rhs_obs.pulse_width_s);
    EXPECT_FLOAT_EQ(lhs_obs.aoa_az_deg, rhs_obs.aoa_az_deg);
    EXPECT_FLOAT_EQ(lhs_obs.aoa_el_deg, rhs_obs.aoa_el_deg);
    EXPECT_FLOAT_EQ(lhs_obs.snr_db, rhs_obs.snr_db);
    EXPECT_EQ(lhs_obs.quality, rhs_obs.quality);
    EXPECT_EQ(lhs_obs.is_jammed, rhs_obs.is_jammed);
  }

  ASSERT_EQ(lhs.output_frame.truth_evaluation_output.associations.size(),
            rhs.output_frame.truth_evaluation_output.associations.size());
  for (std::size_t i = 0; i < lhs.output_frame.truth_evaluation_output.associations.size(); ++i) {
    const common::TruthAssociationRecord& lhs_assoc =
        lhs.output_frame.truth_evaluation_output.associations[i];
    const common::TruthAssociationRecord& rhs_assoc =
        rhs.output_frame.truth_evaluation_output.associations[i];
    EXPECT_EQ(lhs_assoc.observation_id, rhs_assoc.observation_id);
    EXPECT_EQ(lhs_assoc.truth_emitter_id, rhs_assoc.truth_emitter_id);
    EXPECT_EQ(lhs_assoc.matched, rhs_assoc.matched);
    EXPECT_FLOAT_EQ(lhs_assoc.confidence, rhs_assoc.confidence);
  }
}

/**
 * @brief 判断指定真值是否在当前周期被漏检。
 * @param[in] result 周期结果。
 * @param[in] truth_id 真值 ID。
 * @return 漏检时返回 `true`。
 */
bool HasMissedTruth(const EsrCycleResult& result, const std::string& truth_id) {
  for (std::size_t i = 0; i < result.output_frame.truth_evaluation_output.associations.size();
       ++i) {
    const common::TruthAssociationRecord& association =
        result.output_frame.truth_evaluation_output.associations[i];
    if (!association.matched && association.truth_emitter_id == truth_id) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 统计多周期中指定真值的匹配总次数。
 * @param[in] session 会话实例。
 * @param[in] base_input 基础输入模板。
 * @param[in] truth_id 真值 ID。
 * @param[in] cycle_count 统计周期数。
 * @return 累计匹配次数。
 */
std::size_t CountMatchedAcrossCycles(EsrSession* session, const context::EsrCycleInput& base_input,
                                     const std::string& truth_id, std::size_t cycle_count) {
  if (session == nullptr) {
    return 0U;
  }
  std::size_t matched_total = 0U;
  for (std::size_t i = 0; i < cycle_count; ++i) {
    context::EsrCycleInput input = base_input;
    input.cycle_index = base_input.cycle_index + static_cast<std::uint32_t>(i);
    const EsrCycleResult result = session->StepWithResult(input);
    matched_total += CountMatchedTruthObservations(result, truth_id);
  }
  return matched_total;
}

TEST(EsrSessionIntegrationTest, StepWithResultProducesThreeChannelOutput) {
  EsrSession session(MakeSessionConfig());
  const context::EsrCycleInput input = MakeBaseInput();

  const EsrCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_FALSE(result.output_frame.observation_output.observations.empty());
  EXPECT_EQ(result.output_frame.observation_output.cycle_index, input.cycle_index);
  EXPECT_EQ(result.output_frame.emitter_output.cycle_index, input.cycle_index);
  EXPECT_EQ(result.output_frame.truth_evaluation_output.cycle_index, input.cycle_index);
  EXPECT_FALSE(result.output_frame.emitter_output.hypotheses.empty());
  EXPECT_LE(result.output_frame.emitter_output.hypotheses.size(),
            result.output_frame.observation_output.observations.size());
  EXPECT_GE(result.output_frame.truth_evaluation_output.associations.size(),
            result.output_frame.observation_output.observations.size());
}

TEST(EsrSessionIntegrationTest, LayeredDisabledKeepsLegacyExecutionPath) {
  EsrSessionConfig baseline_config = MakeSessionConfig();
  EsrSessionConfig layered_config = baseline_config;
  layered_config.enable_layered_config = false;
  layered_config.layered_config.hardware.receiver_band_lower_hz = 30.0e9;
  layered_config.layered_config.hardware.receiver_band_upper_hz = 31.0e9;
  layered_config.layered_config.hardware.integrated_receive_loss_db = 40.0f;
  layered_config.layered_config.mission.power_on = false;
  layered_config.layered_config.mission.work_mode = EsrWorkMode::kRwr;
  layered_config.layered_config.mission.scan_rate_hz = 8.0f;

  const context::EsrCycleInput input = MakeBaseInput();
  EsrSession baseline_session(baseline_config);
  EsrSession layered_session(layered_config);
  const EsrCycleResult baseline_result = baseline_session.StepWithResult(input);
  const EsrCycleResult layered_result = layered_session.StepWithResult(input);

  ExpectDeterministicResult(baseline_result, layered_result);
}

TEST(EsrSessionIntegrationTest, LayeredFixedBandFiltersOutOfBandEmitters) {
  EsrSessionConfig config = MakeSessionConfig();
  config.enable_layered_config = true;
  config.layered_config.hardware.receiver_band_lower_hz = 8.0e9;
  config.layered_config.hardware.receiver_band_upper_hz = 9.0e9;
  config.layered_config.mission.power_on = true;

  context::EsrCycleInput input = MakeBaseInput();
  input.scene_emitters.front().carrier_hz = 10.0e9;

  EsrSession session(config);
  const EsrCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.output_frame.observation_output.observations.empty());
  EXPECT_EQ(CountMatchedTruthObservations(result, "target-emitter"), 0U);
}

TEST(EsrSessionIntegrationTest, LayeredReceiveLossReducesMatchedObservationSnr) {
  EsrSessionConfig baseline_config = MakeSessionConfig();
  EsrSessionConfig high_loss_config = baseline_config;
  high_loss_config.enable_layered_config = true;
  high_loss_config.layered_config.hardware.receiver_band_lower_hz = 9.0e9;
  high_loss_config.layered_config.hardware.receiver_band_upper_hz = 11.0e9;
  high_loss_config.layered_config.hardware.integrated_receive_loss_db = 20.0f;
  high_loss_config.layered_config.mission.power_on = true;

  context::EsrCycleInput input = MakeBaseInput();
  EsrSession baseline_session(baseline_config);
  EsrSession high_loss_session(high_loss_config);

  const EsrCycleResult baseline_result = baseline_session.StepWithResult(input);
  const EsrCycleResult high_loss_result = high_loss_session.StepWithResult(input);

  ASSERT_GT(CountMatchedTruthObservations(baseline_result, "target-emitter"), 0U);
  const std::size_t high_loss_matched =
      CountMatchedTruthObservations(high_loss_result, "target-emitter");
  ASSERT_TRUE(high_loss_matched > 0U || HasMissedTruth(high_loss_result, "target-emitter"));
  const float baseline_snr = FindMatchedTruthSnr(baseline_result, "target-emitter");
  const float high_loss_snr = FindMatchedTruthSnr(high_loss_result, "target-emitter");
  ASSERT_TRUE(std::isfinite(baseline_snr));
  if (high_loss_matched > 0U) {
    ASSERT_TRUE(std::isfinite(high_loss_snr));
    EXPECT_LT(high_loss_snr, baseline_snr);
  } else {
    EXPECT_TRUE(HasMissedTruth(high_loss_result, "target-emitter"));
  }
}

TEST(EsrSessionIntegrationTest, LayeredModeMappingMakesHgesmMoreDetectableThanRwr) {
  EsrSessionConfig hgesm_config = MakeSessionConfig();
  hgesm_config.enable_layered_config = true;
  hgesm_config.layered_config.hardware.receiver_band_lower_hz = 9.0e9;
  hgesm_config.layered_config.hardware.receiver_band_upper_hz = 11.0e9;
  hgesm_config.layered_config.mission.power_on = true;
  hgesm_config.layered_config.mission.work_mode = EsrWorkMode::kHgesm;

  EsrSessionConfig rwr_config = hgesm_config;
  rwr_config.layered_config.mission.work_mode = EsrWorkMode::kRwr;

  const double kPowerSweepW[] = {20.0, 30.0, 40.0, 60.0};
  bool has_strict_advantage = false;
  for (std::size_t i = 0; i < sizeof(kPowerSweepW) / sizeof(kPowerSweepW[0]); ++i) {
    context::EsrCycleInput input = MakeBaseInput();
    input.scene_emitters.front().tx_power_w = kPowerSweepW[i];

    EsrSession hgesm_session(hgesm_config);
    EsrSession rwr_session(rwr_config);
    const std::size_t hgesm_matched =
        CountMatchedAcrossCycles(&hgesm_session, input, "target-emitter", 32U);
    const std::size_t rwr_matched =
        CountMatchedAcrossCycles(&rwr_session, input, "target-emitter", 32U);
    EXPECT_GE(hgesm_matched, rwr_matched);
    if (hgesm_matched > rwr_matched) {
      has_strict_advantage = true;
    }
  }
  EXPECT_TRUE(has_strict_advantage);
}

TEST(EsrSessionIntegrationTest, LayeredPowerOffReturnsEmptyChannels) {
  EsrSessionConfig config = MakeSessionConfig();
  config.enable_layered_config = true;
  config.layered_config.mission.power_on = false;

  EsrSession session(config);
  const EsrCycleResult result = session.StepWithResult(MakeBaseInput());

  EXPECT_TRUE(result.output_frame.observation_output.observations.empty());
  EXPECT_TRUE(result.output_frame.emitter_output.hypotheses.empty());
  EXPECT_TRUE(result.output_frame.truth_evaluation_output.associations.empty());
}

TEST(EsrSessionIntegrationTest, SuppressionJammingDegradesSnrWithoutRaisingFalseAlarms) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput no_jam_input = MakeBaseInput();
  context::EsrCycleInput jam_input = no_jam_input;
  jam_input.cycle_index = no_jam_input.cycle_index;

  environment::EsrJammerSource jammer;
  jammer.technique = environment::EsrJammingTechnique::kNoiseSuppression;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 3.0e9;
  jammer.power_w = 1.0e-4f;
  jammer.confidence = 1.0f;
  jammer.deception_risk = 1.0f;
  jam_input.environment_scene_state.jammer_sources.push_back(jammer);

  const EsrCycleResult no_jam_result = session.StepWithResult(no_jam_input);
  const EsrCycleResult jam_result = session.StepWithResult(jam_input);

  ASSERT_FALSE(no_jam_result.output_frame.observation_output.observations.empty());
  const std::size_t no_jam_match_count =
      CountMatchedTruthObservations(no_jam_result, "target-emitter");
  const std::size_t jam_match_count = CountMatchedTruthObservations(jam_result, "target-emitter");
  const std::size_t no_jam_false_alarm_count = CountFalseAlarms(no_jam_result);
  const std::size_t jam_false_alarm_count = CountFalseAlarms(jam_result);

  EXPECT_LE(jam_match_count, no_jam_match_count);
  if (jam_match_count > 0U && no_jam_match_count > 0U) {
    const float no_jam_snr = FindMatchedTruthSnr(no_jam_result, "target-emitter");
    const float jam_snr = FindMatchedTruthSnr(jam_result, "target-emitter");
    ASSERT_TRUE(std::isfinite(no_jam_snr));
    ASSERT_TRUE(std::isfinite(jam_snr));
    EXPECT_LE(jam_snr, no_jam_snr);
  } else {
    EXPECT_TRUE(HasMissedTruth(jam_result, "target-emitter"));
  }
  EXPECT_LE(jam_false_alarm_count, no_jam_false_alarm_count + 1U);
}

TEST(EsrSessionIntegrationTest, DeceptionJammingRaisesFalseAlarmsWithoutSnrDrop) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput no_jam_input = MakeBaseInput();
  context::EsrCycleInput deception_input = no_jam_input;

  environment::EsrJammerSource jammer;
  jammer.technique = environment::EsrJammingTechnique::kDeception;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 2.0e6;
  jammer.power_w = 1.0e-3f;
  jammer.confidence = 1.0f;
  jammer.deception_risk = 1.0f;
  deception_input.environment_scene_state.jammer_sources.push_back(jammer);

  const EsrCycleResult no_jam_result = session.StepWithResult(no_jam_input);
  const EsrCycleResult deception_result = session.StepWithResult(deception_input);

  ASSERT_FALSE(no_jam_result.output_frame.emitter_output.hypotheses.empty());
  ASSERT_FALSE(deception_result.output_frame.emitter_output.hypotheses.empty());
  ASSERT_FALSE(no_jam_result.output_frame.observation_output.observations.empty());
  ASSERT_FALSE(deception_result.output_frame.observation_output.observations.empty());

  const float no_jam_snr = FindMatchedTruthSnr(no_jam_result, "target-emitter");
  const float deception_snr = FindMatchedTruthSnr(deception_result, "target-emitter");
  const std::size_t no_jam_false_alarm_count = CountFalseAlarms(no_jam_result);
  const std::size_t deception_false_alarm_count = CountFalseAlarms(deception_result);
  const float no_jam_mean_confidence = AverageHypothesisConfidence(no_jam_result);
  const float deception_mean_confidence = AverageHypothesisConfidence(deception_result);
  const std::size_t no_jam_ambiguous_count = CountAmbiguousHypotheses(no_jam_result);
  const std::size_t deception_ambiguous_count = CountAmbiguousHypotheses(deception_result);

  ASSERT_TRUE(std::isfinite(no_jam_snr));
  ASSERT_TRUE(std::isfinite(deception_snr));
  EXPECT_NEAR(deception_snr, no_jam_snr, 1.0e-4f);
  EXPECT_GT(deception_false_alarm_count, no_jam_false_alarm_count);
  EXPECT_LE(deception_mean_confidence, no_jam_mean_confidence);
  EXPECT_GE(deception_ambiguous_count, no_jam_ambiguous_count);
}

TEST(EsrSessionIntegrationTest, MixedJammingCombinesSuppressionAndDeceptionEffects) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput base_input = MakeBaseInput();
  context::EsrCycleInput mixed_input = base_input;

  environment::EsrJammerSource jammer;
  jammer.technique = environment::EsrJammingTechnique::kMixed;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 2.0e6;
  jammer.power_w = 1.0e-4f;
  jammer.confidence = 1.0f;
  jammer.deception_risk = 1.0f;
  mixed_input.environment_scene_state.jammer_sources.push_back(jammer);

  const EsrCycleResult base_result = session.StepWithResult(base_input);
  const EsrCycleResult mixed_result = session.StepWithResult(mixed_input);

  const float base_snr = FindMatchedTruthSnr(base_result, "target-emitter");
  const float mixed_snr = FindMatchedTruthSnr(mixed_result, "target-emitter");
  const std::size_t base_false_alarm_count = CountFalseAlarms(base_result);
  const std::size_t mixed_false_alarm_count = CountFalseAlarms(mixed_result);

  ASSERT_TRUE(std::isfinite(base_snr));
  if (std::isfinite(mixed_snr)) {
    EXPECT_LE(mixed_snr, base_snr);
  } else {
    EXPECT_TRUE(HasMissedTruth(mixed_result, "target-emitter"));
  }
  EXPECT_GT(mixed_false_alarm_count, base_false_alarm_count);
}

TEST(EsrSessionIntegrationTest, DeceptionCanInjectFalseAlarmsWhenGateFails) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput input = MakeBaseInput();
  input.scene_emitters.front().pose.position_m.x = -200.0f;
  input.scene_emitters.front().pose.position_m.y = 1200.0f;

  environment::EsrJammerSource jammer;
  jammer.technique = environment::EsrJammingTechnique::kDeception;
  jammer.active = true;
  jammer.center_hz = 10.0e9;
  jammer.bandwidth_hz = 2.0e6;
  jammer.power_w = 1.0e-3f;
  jammer.confidence = 1.0f;
  jammer.deception_risk = 1.0f;
  input.environment_scene_state.jammer_sources.push_back(jammer);

  const EsrCycleResult result = session.StepWithResult(input);

  EXPECT_EQ(CountMatchedTruthObservations(result, "target-emitter"), 0U);
  EXPECT_GT(CountFalseAlarms(result), 0U);
}

TEST(EsrSessionIntegrationTest, DeceptionRiskAndSeedProduceDeterministicOutput) {
  EsrSessionConfig config = MakeSessionConfig();
  config.pipeline_config.algorithm.random_seed = 424242U;
  config.pipeline_config.deception_model.max_false_observations_per_emitter = 3U;

  context::EsrCycleInput risk_zero_input = MakeBaseInput();
  environment::EsrJammerSource risk_zero_jammer;
  risk_zero_jammer.technique = environment::EsrJammingTechnique::kDeception;
  risk_zero_jammer.active = true;
  risk_zero_jammer.center_hz = 10.0e9;
  risk_zero_jammer.bandwidth_hz = 2.0e6;
  risk_zero_jammer.power_w = 1.0e-3f;
  risk_zero_jammer.confidence = 1.0f;
  risk_zero_jammer.deception_risk = 0.0f;
  risk_zero_input.environment_scene_state.jammer_sources.push_back(risk_zero_jammer);

  context::EsrCycleInput risk_one_input = MakeBaseInput();
  environment::EsrJammerSource risk_one_jammer = risk_zero_jammer;
  risk_one_jammer.deception_risk = 1.0f;
  risk_one_input.environment_scene_state.jammer_sources.push_back(risk_one_jammer);

  EsrSession risk_zero_session(config);
  const EsrCycleResult risk_zero_result = risk_zero_session.StepWithResult(risk_zero_input);
  EXPECT_EQ(CountFalseAlarms(risk_zero_result), 0U);

  EsrSession risk_one_session_a(config);
  EsrSession risk_one_session_b(config);
  const EsrCycleResult risk_one_result_a = risk_one_session_a.StepWithResult(risk_one_input);
  const EsrCycleResult risk_one_result_b = risk_one_session_b.StepWithResult(risk_one_input);

  EXPECT_GT(CountFalseAlarms(risk_one_result_a), 0U);
  EXPECT_EQ(CountFalseAlarms(risk_one_result_a), CountFalseAlarms(risk_one_result_b));
  ExpectDeterministicResult(risk_one_result_a, risk_one_result_b);
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

TEST(EsrSessionIntegrationTest, PlatformAttitudeChangesInterceptGateGeometry) {
  EsrSessionConfig config = MakeSessionConfig();
  config.pipeline_config.scan.scan_start_az_deg = 0.0f;
  config.pipeline_config.scan.scan_end_az_deg = 0.0f;
  config.pipeline_config.scan.scan_start_el_deg = 0.0f;
  config.pipeline_config.scan.scan_end_el_deg = 0.0f;
  config.pipeline_config.scan.az_step_deg = 10.0f;
  config.pipeline_config.scan.el_step_deg = 10.0f;

  context::EsrCycleInput input = MakeBaseInput();
  input.scene_emitters.front().pose.position_m.x = 0.0f;
  input.scene_emitters.front().pose.position_m.y = 1200.0f;
  input.scene_emitters.front().pose.position_m.z = 5000.0f;

  EsrSession zero_attitude_session(config);
  const EsrCycleResult zero_attitude_result = zero_attitude_session.StepWithResult(input);
  EXPECT_EQ(CountMatchedTruthObservations(zero_attitude_result, "target-emitter"), 0U);

  input.platform_pose.attitude_deg.yaw_deg = 90.0f;
  EsrSession yawed_session(config);
  const EsrCycleResult yawed_result = yawed_session.StepWithResult(input);

  EXPECT_EQ(CountMatchedTruthObservations(yawed_result, "target-emitter"), 1U);
  ASSERT_FALSE(yawed_result.output_frame.observation_output.observations.empty());
  EXPECT_NEAR(yawed_result.output_frame.observation_output.observations.front().aoa_az_deg, 0.0f,
              3.0f);
}

TEST(EsrSessionIntegrationTest, EmitterBeamStateControlsWhetherTrueObservationCanBeIntercepted) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput blocked_input = MakeBaseInput();
  blocked_input.scene_emitters.front().beam_state.beam_state_valid = true;
  blocked_input.scene_emitters.front().beam_state.center_az_deg = 0.0f;
  blocked_input.scene_emitters.front().beam_state.center_el_deg = 0.0f;
  blocked_input.scene_emitters.front().beam_state.az_beamwidth_deg = 8.0f;
  blocked_input.scene_emitters.front().beam_state.el_beamwidth_deg = 8.0f;

  context::EsrCycleInput covered_input = blocked_input;
  covered_input.scene_emitters.front().beam_state.center_az_deg = 180.0f;
  covered_input.scene_emitters.front().beam_state.center_el_deg = -10.0f;

  const EsrCycleResult blocked_result = session.StepWithResult(blocked_input);
  const EsrCycleResult covered_result = session.StepWithResult(covered_input);

  EXPECT_EQ(CountMatchedTruthObservations(blocked_result, "target-emitter"), 0U);
  EXPECT_GT(CountMatchedTruthObservations(covered_result, "target-emitter"), 0U);
}

TEST(EsrSessionIntegrationTest, PriLongerThanCycleSuppressesTrueDetection) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput low_pri_input = MakeBaseInput();
  context::EsrCycleInput high_pri_input = MakeBaseInput();
  high_pri_input.scene_emitters.front().pri_s = 10.0;

  const EsrCycleResult low_pri_result = session.StepWithResult(low_pri_input);
  const EsrCycleResult high_pri_result = session.StepWithResult(high_pri_input);

  EXPECT_GT(CountMatchedTruthObservations(low_pri_result, "target-emitter"), 0U);
  EXPECT_EQ(CountMatchedTruthObservations(high_pri_result, "target-emitter"), 0U);
}

TEST(EsrSessionIntegrationTest, PriWindowCapsEffectivePulseCountAcrossStatisticalConfigs) {
  EsrSessionConfig low_pulse_config = MakeSessionConfig();
  low_pulse_config.pipeline_config.statistical_detection.pulse_count = 8U;
  EsrSessionConfig high_pulse_config = low_pulse_config;
  high_pulse_config.pipeline_config.statistical_detection.pulse_count = 32U;

  context::EsrCycleInput input = MakeBaseInput();
  input.scene_emitters.front().pri_s = 0.25;

  EsrSession low_pulse_session(low_pulse_config);
  EsrSession high_pulse_session(high_pulse_config);
  const EsrCycleResult low_pulse_result = low_pulse_session.StepWithResult(input);
  const EsrCycleResult high_pulse_result = high_pulse_session.StepWithResult(input);

  EXPECT_EQ(CountMatchedTruthObservations(low_pulse_result, "target-emitter"),
            CountMatchedTruthObservations(high_pulse_result, "target-emitter"));
  const float low_pulse_snr = FindMatchedTruthSnr(low_pulse_result, "target-emitter");
  const float high_pulse_snr = FindMatchedTruthSnr(high_pulse_result, "target-emitter");
  if (std::isfinite(low_pulse_snr) && std::isfinite(high_pulse_snr)) {
    EXPECT_FLOAT_EQ(low_pulse_snr, high_pulse_snr);
  }
}

TEST(EsrSessionIntegrationTest, ValidationFailureReturnsEmptyFrameAndStillAdvancesBatchId) {
  EsrSession session(MakeSessionConfig());
  context::EsrCycleInput invalid_input = MakeBaseInput();
  invalid_input.dt_sec = 0.0f;

  const EsrCycleResult invalid_result = session.StepWithResult(invalid_input);
  EXPECT_TRUE(invalid_result.has_validation_error);
  EXPECT_TRUE(invalid_result.output_frame.observation_output.observations.empty());
  EXPECT_TRUE(invalid_result.output_frame.emitter_output.hypotheses.empty());
  EXPECT_TRUE(invalid_result.output_frame.truth_evaluation_output.associations.empty());
  EXPECT_EQ(invalid_result.output_frame.observation_output.batch_id, 1U);

  context::EsrCycleInput valid_input = MakeBaseInput();
  valid_input.cycle_index = invalid_input.cycle_index + 1U;
  const EsrCycleResult valid_result = session.StepWithResult(valid_input);
  EXPECT_FALSE(valid_result.has_validation_error);
  EXPECT_EQ(valid_result.output_frame.observation_output.batch_id, 2U);
}

}  // namespace
}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar
