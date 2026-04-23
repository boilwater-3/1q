// Copyright 2026. All Rights Reserved.
//
// @file decision_layer_test.cpp
// @brief 验证默认战术协调器路径的核心行为。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "1q/airborne_radar/extension/control/ControlDirective.h"
#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "airborne_radar/decision/evaluators/ThreatAssessmentEvaluator.h"
#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"
#include "airborne_radar/decision/pipeline/TacticalEvaluation.h"
#include "airborne_radar/environment/FeatureRepository.h"

namespace dp = airborne_radar::decision::pipeline;
namespace de = airborne_radar::decision::evaluators;
namespace acc = airborne_radar::extension::control;
namespace acm = airborne_radar::model;
namespace edb = airborne_radar::environment;

namespace {

acm::TrackStateSnapshot MakeTrack(float vx, float vy, float vz, float rcs, bool jamming = false) {
  acm::TrackStateSnapshot track;
  track.velocity_x = vx;
  track.velocity_y = vy;
  track.velocity_z = vz;
  track.speed = std::sqrt(vx * vx + vy * vy + vz * vz);
  track.rcs = rcs;
  track.jamming_detected = jamming;
  return track;
}

acm::DecisionInputFrame BuildSingleTrackFrame(float speed, float rcs, bool jamming) {
  acm::DecisionInputFrame frame;
  frame.tracks.push_back(MakeTrack(speed, 0.0f, 0.0f, rcs, jamming));
  frame.tracks.back().status = acm::TrackStatus::kConfirmed;
  return frame;
}

std::string PrimaryCategory(const dp::TacticalDecisionResult& result) {
  if (result.target_classification_result.empty()) {
    return "UNKNOWN";
  }
  return result.target_classification_result.front().target_type;
}

bool ContainsDirectiveType(const std::vector<dp::TacticalProposal>& proposals,
                           acc::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const dp::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

}  // namespace

TEST(TacticalCoordinatorTest, HighThreatAndJamming) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  acm::DecisionInputFrame frame = BuildSingleTrackFrame(800.0f, 2.5f, true);
  frame.environment_jamming_detected = true;
  frame.eccm_source_info.has_jamming_signal = true;
  acm::EccmJammerSourceInfo source;
  source.technique = acm::JammingTechnique::kDeception;
  source.jammer_power_db = 10.0f;
  source.jammer_to_signal_db = 8.0f;
  source.frequency_overlap_ratio = 0.9f;
  source.prf_lock_risk = 0.9f;
  source.jammer_in_sidelobe = true;
  source.confidence = 1.0f;
  frame.eccm_source_info.jammer_sources.push_back(source);

  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.selected_mode, dp::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    acc::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, acc::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    acc::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(
      ContainsDirectiveType(result.proposals, acc::ControlDirectiveType::REQUEST_ECCM_REJITTER));
}

TEST(TacticalCoordinatorTest, LowThreatClearEnvironment) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(150.0f, 1.0f, false);
  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "LOW_THREAT_TARGET");
  EXPECT_EQ(result.selected_mode, dp::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, HighThreatClearEnvironment) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(900.0f, 5.0f, false);
  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.selected_mode, dp::TacticalMode::kThreatResponse);
  ASSERT_EQ(result.proposals.size(), 1u);
  EXPECT_EQ(result.proposals[0].directive.type,
            acc::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
}

TEST(TacticalCoordinatorTest, HighRcsCanPromoteThreatWithModerateSpeed) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(180.0f, 4.2f, false);
  const dp::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
}

TEST(TacticalCoordinatorTest, ClassifiesAllTracksInFrame) {
  dp::TacticalCoordinator coordinator;
  dp::TacticalStateStore state_store;

  acm::DecisionInputFrame frame;
  frame.tracks.push_back(MakeTrack(900.0f, 0.0f, 0.0f, 4.5f));
  frame.tracks.push_back(MakeTrack(160.0f, 0.0f, 0.0f, 0.8f));
  frame.tracks[0].status = acm::TrackStatus::kConfirmed;
  frame.tracks[1].status = acm::TrackStatus::kConfirmed;

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

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(780.0f, 2.8f, true);

  evaluator.Evaluate(frame, state_store, evaluation_state);

  ASSERT_FALSE(evaluation_state.target_classification_result.empty());
  EXPECT_EQ(evaluation_state.target_classification_result.front().target_type,
            "HIGH_THREAT_FIGHTER");
  const float prob = evaluation_state.target_classification_result.front().probability;
  EXPECT_GT(prob, 0.0f);
  EXPECT_LE(prob, 1.0f);
  EXPECT_FALSE(std::isnan(prob));
}

TEST(FeatureRepositoryTest, ExtremeDistanceReturnsFalseInsteadOfInvalidProbability) {
  edb::FeatureRepository repository;
  edb::FeatureVector input;
  input.Set("speed", 1.0e12f);
  input.Set("rcs", 1.0e12f);
  input.Set("jamming", 1.0f);

  edb::MatchResult result;
  result.target_type = "SHOULD_BE_RESET";
  result.probability = 1.0f;
  result.distance = 1.0f;

  const bool matched = repository.QueryBestMatch(input, result);
  EXPECT_FALSE(matched);
  EXPECT_EQ(result.target_type, "UNKNOWN");
  EXPECT_FLOAT_EQ(result.probability, 0.0f);
  EXPECT_FLOAT_EQ(result.distance, 0.0f);
}
