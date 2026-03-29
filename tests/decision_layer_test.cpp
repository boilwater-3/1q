// Copyright 2026. All Rights Reserved.
//
// @file decision_layer_test.cpp
// @brief 验证默认战术协调器路径的核心行为。

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "airborne_radar/decision/evaluators/ThreatAssessmentEvaluator.h"
#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"
#include "airborne_radar/decision/pipeline/TacticalEvaluation.h"
#include "airborne_radar/environment/database/FeatureRepository.h"

namespace dp = airborne_radar::decision::pipeline;
namespace de = airborne_radar::decision::evaluators;
namespace ac = airborne_radar::common;
namespace edb = airborne_radar::environment::database;

namespace {

ac::DecisionInputFrame BuildSingleTrackFrame(float speed, float rcs, bool jamming) {
  ac::DecisionInputFrame frame;
  frame.tracks.push_back(ac::DecisionTrackSnapshot(speed, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f,
                                                    jamming));
  frame.tracks.back().state.status = ac::DecisionTrackStatus::kConfirmed;
  frame.tracks.back().evidence.has_measurement_evidence = true;
  frame.tracks.back().evidence.updated_this_cycle = true;
  return frame;
}

std::string PrimaryCategory(const dp::TacticalDecisionResult& result) {
  if (result.target_classification_result.empty()) {
    return "UNKNOWN";
  }
  return result.target_classification_result.front().target_type;
}

bool ContainsDirectiveType(const std::vector<dp::TacticalProposal>& proposals,
                           ac::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const dp::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

}  // namespace

TEST(TacticalCoordinatorTest, HighThreatAndJamming) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  ac::DecisionInputFrame frame = BuildSingleTrackFrame(800.0f, 2.5f, true);
  frame.environment_jamming_detected = true;
  frame.eccm_source_info.has_jamming_signal = true;
  frame.eccm_source_info.jammer_power_db = 10.0f;
  frame.eccm_source_info.frequency_overlap_ratio = 0.9f;
  frame.eccm_source_info.prf_lock_risk = 0.9f;
  frame.eccm_source_info.jammer_in_sidelobe = true;

  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.selected_mode, dp::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    ac::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, ac::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    ac::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(
      ContainsDirectiveType(result.proposals, ac::ControlDirectiveType::REQUEST_ECCM_REJITTER));
}

TEST(TacticalCoordinatorTest, LowThreatClearEnvironment) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  const ac::DecisionInputFrame frame = BuildSingleTrackFrame(150.0f, 1.0f, false);
  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "LOW_THREAT_TARGET");
  EXPECT_EQ(result.selected_mode, dp::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, HighThreatClearEnvironment) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  const ac::DecisionInputFrame frame = BuildSingleTrackFrame(900.0f, 5.0f, false);
  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.selected_mode, dp::TacticalMode::kThreatResponse);
  ASSERT_EQ(result.proposals.size(), 1u);
  EXPECT_EQ(result.proposals[0].directive.type,
            ac::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
}

TEST(TacticalCoordinatorTest, HighRcsCanPromoteThreatWithModerateSpeed) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  const ac::DecisionInputFrame frame = BuildSingleTrackFrame(180.0f, 4.2f, false);
  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
}

TEST(TacticalCoordinatorTest, ClassifiesAllTracksInFrame) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  ac::DecisionInputFrame frame;
  frame.tracks.push_back(ac::DecisionTrackSnapshot(900.0f, 0.0f, 0.0f, 4.5f));
  frame.tracks.push_back(ac::DecisionTrackSnapshot(160.0f, 0.0f, 0.0f, 0.8f));
  frame.tracks[0].state.status = ac::DecisionTrackStatus::kConfirmed;
  frame.tracks[1].state.status = ac::DecisionTrackStatus::kConfirmed;
  frame.tracks[0].evidence.has_measurement_evidence = true;
  frame.tracks[1].evidence.has_measurement_evidence = true;

  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  ASSERT_EQ(result.target_classification_result.size(), frame.tracks.size());
  EXPECT_EQ(result.target_classification_result[0].target_type, "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.target_classification_result[1].target_type, "LOW_THREAT_TARGET");
}

TEST(ThreatAssessmentEvaluatorTest, RepositoryMatchProvidesProbability) {
  edb::FeatureRepository repository;
  de::ThreatAssessmentEvaluator evaluator(&repository);
  dp::TacticalStateStore state_store;
  dp::TacticalEvaluationState evaluation_state;

  const ac::DecisionInputFrame frame = BuildSingleTrackFrame(780.0f, 2.8f, true);

  evaluator.Evaluate(frame, state_store, evaluation_state);

  ASSERT_FALSE(evaluation_state.target_classification_result.empty());
  EXPECT_EQ(evaluation_state.target_classification_result.front().target_type,
            "HIGH_THREAT_FIGHTER");
  const float prob = evaluation_state.target_classification_result.front().probability;
  EXPECT_GT(prob, 0.0f);
  EXPECT_LE(prob, 1.0f);
  EXPECT_FALSE(std::isnan(prob));
}
