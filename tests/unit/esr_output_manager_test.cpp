/**
 * @file esr_output_manager_test.cpp
 * @brief 验证 ESR 输出管理器的空帧构造与流水线结果装配行为。
 */

#include <gtest/gtest.h>

#include <string>

#include "electronic_surveillance_radar/output/EsrOutputManager.h"

namespace electronic_surveillance_radar {
namespace output {
namespace {

model::EmitterObservation MakeObservation(std::uint64_t observation_id, float snr_db) {
  model::EmitterObservation observation;
  observation.observation_id = observation_id;
  observation.timestamp_s = 12.0;
  observation.aoa_az_deg = 5.0f;
  observation.aoa_el_deg = -2.0f;
  observation.rf_hz = 9.4e9f;
  observation.pulse_width_s = 2.0e-6f;
  observation.amplitude_db = 18.0f;
  observation.snr_db = snr_db;
  observation.quality = model::EsrObservationQuality::kHigh;
  return observation;
}

model::EmitterHypothesis MakeHypothesis(std::uint64_t hypothesis_id, float confidence) {
  model::EmitterHypothesis hypothesis;
  hypothesis.hypothesis_id = hypothesis_id;
  hypothesis.candidate_classes.push_back("SAM");
  hypothesis.mode = model::EsrEmitterMode::kTracking;
  hypothesis.threat_level = model::EsrThreatLevel::kHigh;
  hypothesis.bearing_az_deg = 11.0f;
  hypothesis.bearing_el_deg = 2.5f;
  hypothesis.bearing_std_deg = 0.8f;
  hypothesis.confidence = confidence;
  hypothesis.last_seen_cycle = 6U;
  return hypothesis;
}

output::TruthAssociationRecord MakeAssociation(std::uint64_t observation_id,
                                               const std::string& truth_emitter_id, bool matched) {
  output::TruthAssociationRecord association;
  association.observation_id = observation_id;
  association.truth_emitter_id = truth_emitter_id;
  association.matched = matched;
  association.confidence = matched ? 0.9f : 0.25f;
  return association;
}

TEST(EsrOutputManagerTest, BuildEmptyFrameSetsTopLevelHeader) {
  EsrOutputManager manager;

  const output::EsrOutputFrame frame = manager.BuildEmptyFrame(7U, 103U);

  EXPECT_EQ(frame.cycle_index, 7U);
  EXPECT_EQ(frame.batch_id, 103U);
  EXPECT_TRUE(frame.observation_output.observations.empty());
  EXPECT_TRUE(frame.emitter_output.hypotheses.empty());
  EXPECT_TRUE(frame.truth_evaluation_output.associations.empty());
}

TEST(EsrOutputManagerTest, BuildOutputFrameMovesCycleResults) {
  EsrOutputManager manager;
  extension::InterceptCycleResult cycle_result;
  cycle_result.observations.push_back(MakeObservation(1001U, 18.5f));
  cycle_result.observations.push_back(MakeObservation(1002U, 7.5f));
  cycle_result.emitter_hypotheses.push_back(MakeHypothesis(3001U, 0.82f));
  cycle_result.truth_associations.push_back(MakeAssociation(1001U, "truth-a", true));
  cycle_result.truth_associations.push_back(MakeAssociation(1002U, "UNASSOCIATED", false));

  const output::EsrOutputFrame frame = manager.BuildOutputFrame(8U, 104U, cycle_result);

  EXPECT_EQ(frame.cycle_index, 8U);
  EXPECT_EQ(frame.batch_id, 104U);
  ASSERT_EQ(frame.observation_output.observations.size(), 2U);
  EXPECT_EQ(frame.observation_output.observations[0].observation_id, 1001U);
  EXPECT_EQ(frame.observation_output.observations[1].observation_id, 1002U);
  ASSERT_EQ(frame.emitter_output.hypotheses.size(), 1U);
  EXPECT_EQ(frame.emitter_output.hypotheses[0].hypothesis_id, 3001U);
  ASSERT_EQ(frame.truth_evaluation_output.associations.size(), 2U);
  EXPECT_TRUE(frame.truth_evaluation_output.associations[0].matched);
  EXPECT_FALSE(frame.truth_evaluation_output.associations[1].matched);
}

}  // namespace
}  // namespace output

}  // namespace electronic_surveillance_radar
