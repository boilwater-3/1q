// Copyright 2026. All Rights Reserved.
//
// Description: GTest based verification for the tactical coordinator path.

#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/decision/TacticalCoordinator.h"
#include "1q/airborne_radar/decision/classifier/ThreatAssessmentEvaluator.h"
#include "1q/airborne_radar/environment/database/FeatureRepository.h"

using namespace airborne_radar;

namespace {

common::DecisionInputFrame BuildSingleTrackFrame(float speed, float rcs,
                                                 bool jamming) {
  common::DecisionInputFrame frame;
  frame.tracks.push_back(
      common::DecisionTrackSnapshot(speed, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f,
                                    jamming));
  frame.tracks.back().state.status = common::DecisionTrackStatus::kConfirmed;
  frame.tracks.back().evidence.has_measurement_evidence = true;
  frame.tracks.back().evidence.updated_this_cycle = true;
  return frame;
}

std::string PrimaryCategory(
    const decision::TacticalDecisionResult& result) {
  if (result.target_classification_result.empty()) {
    return "UNKNOWN";
  }
  return result.target_classification_result.front().target_type;
}

bool ContainsDirectiveType(const std::vector<decision::TacticalProposal>& proposals,
                           common::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const decision::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

} // namespace

TEST(TacticalCoordinatorTest, HighThreatAndJamming) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  common::DecisionInputFrame frame =
      BuildSingleTrackFrame(800.0f, 2.5f, true);
  frame.environment_jamming_detected = true;
  frame.eccm_source_info.has_jamming_signal = true;
  frame.eccm_source_info.jammer_power_db = 10.0f;
  frame.eccm_source_info.frequency_overlap_ratio = 0.9f;
  frame.eccm_source_info.prf_lock_risk = 0.9f;
  frame.eccm_source_info.jammer_in_sidelobe = true;

  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_TARGET");
  EXPECT_EQ(result.selected_mode, decision::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
}

TEST(TacticalCoordinatorTest, LowThreatClearEnvironment) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  const common::DecisionInputFrame frame =
      BuildSingleTrackFrame(150.0f, 1.0f, false);
  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "LOW_THREAT_TARGET");
  EXPECT_EQ(result.selected_mode, decision::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, HighThreatClearEnvironment) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  const common::DecisionInputFrame frame =
      BuildSingleTrackFrame(900.0f, 5.0f, false);
  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_TARGET");
  EXPECT_EQ(result.selected_mode, decision::TacticalMode::kThreatResponse);
  ASSERT_EQ(result.proposals.size(), 1u);
  EXPECT_EQ(result.proposals[0].directive.type,
            common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
}

TEST(TacticalCoordinatorTest, HighRcsCanPromoteThreatWithModerateSpeed) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  const common::DecisionInputFrame frame =
      BuildSingleTrackFrame(180.0f, 4.2f, false);
  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_TARGET");
}

TEST(TacticalCoordinatorTest, ClassifiesAllTracksInFrame) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.tracks.push_back(common::DecisionTrackSnapshot(900.0f, 0.0f, 0.0f, 4.5f));
  frame.tracks.push_back(common::DecisionTrackSnapshot(160.0f, 0.0f, 0.0f, 0.8f));
  frame.tracks[0].state.status = common::DecisionTrackStatus::kConfirmed;
  frame.tracks[1].state.status = common::DecisionTrackStatus::kConfirmed;
  frame.tracks[0].evidence.has_measurement_evidence = true;
  frame.tracks[1].evidence.has_measurement_evidence = true;

  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  ASSERT_EQ(result.target_classification_result.size(), frame.tracks.size());
  EXPECT_EQ(result.target_classification_result[0].target_type,
            "HIGH_THREAT_TARGET");
  EXPECT_EQ(result.target_classification_result[1].target_type,
            "LOW_THREAT_TARGET");
}

TEST(ThreatAssessmentEvaluatorTest, RepositoryMatchProvidesProbability) {
  environment::database::FeatureRepository repository;
  decision::classifier::ThreatAssessmentEvaluator evaluator(&repository);
  decision::TacticalStateStore state_store;
  decision::TacticalEvaluationState evaluation_state;

  const common::DecisionInputFrame frame =
      BuildSingleTrackFrame(780.0f, 2.8f, true);

  evaluator.Evaluate(frame, state_store, evaluation_state);

  ASSERT_FALSE(evaluation_state.target_classification_result.empty());
  EXPECT_EQ(evaluation_state.target_classification_result.front().target_type,
            "HIGH_THREAT_FIGHTER");
}
