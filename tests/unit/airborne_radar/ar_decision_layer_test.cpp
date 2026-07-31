// Copyright 2026. All Rights Reserved.
//
// @file decision_layer_test.cpp
// @brief 验证默认战术协调器路径的核心行为。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "airborne_radar/decision/ControlReducerTypes.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/decision/ThreatAssessmentEvaluator.h"
#include "airborne_radar/decision/TacticalCoordinator.h"
#include "airborne_radar/environment/FeatureRepository.h"

namespace dp = airborne_radar::decision;
namespace de = airborne_radar::decision;
namespace acc = airborne_radar::session;
namespace acm = airborne_radar::session;
namespace edb = airborne_radar::environment;
namespace ext = airborne_radar::session;

namespace {

acm::TrackStateSnapshot MakeTrack(float vx, float vy, float vz, float rcs) {
  acm::TrackStateSnapshot track;
  track.velocity_x = vx;
  track.velocity_y = vy;
  track.velocity_z = vz;
  track.speed = std::sqrt(vx * vx + vy * vy + vz * vz);
  track.rcs = rcs;
  return track;
}

acm::DecisionInputFrame BuildSingleTrackFrame(float speed, float rcs) {
  acm::DecisionInputFrame frame;
  frame.tracks.push_back(MakeTrack(speed, 0.0f, 0.0f, rcs));
  frame.tracks.back().status = acm::TrackStatus::kConfirmed;
  return frame;
}

std::string PrimaryCategory(const ext::TacticalDecisionResult& result) {
  if (result.target_classification_result.empty()) {
    return "UNKNOWN";
  }
  return result.target_classification_result.front().target_type;
}

bool ContainsDirectiveType(const std::vector<ext::TacticalProposal>& proposals,
                           acc::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const ext::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

}  // namespace

TEST(TacticalCoordinatorTest, HighThreatAndReceiverRfObservation) {
  dp::TacticalCoordinator coordinator;
  ext::TacticalStateStore state_store;

  acm::DecisionInputFrame frame = BuildSingleTrackFrame(800.0f, 2.5f);
  acm::ArInterferenceObservation observation;
  observation.observation_id = 1U;
  observation.estimated_off_boresight_deg = 8.0;
  observation.estimated_center_frequency_hz = 3.0e9;
  observation.estimated_bandwidth_hz = 2.0e6;
  observation.estimated_waveform_kind =
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  observation.jammer_to_noise_db = 12.0;
  frame.interference_observations.push_back(observation);

  const ext::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.selected_mode, ext::TacticalMode::kProtectedEmission);
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
  ext::TacticalStateStore state_store;

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(150.0f, 1.0f);
  const ext::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "LOW_THREAT_TARGET");
  EXPECT_EQ(result.selected_mode, ext::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, HighThreatClearEnvironment) {
  dp::TacticalCoordinator coordinator;
  ext::TacticalStateStore state_store;

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(900.0f, 5.0f);
  const ext::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.selected_mode, ext::TacticalMode::kThreatResponse);
  ASSERT_EQ(result.proposals.size(), 2u);
  EXPECT_EQ(result.proposals[0].directive.type,
            acc::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
  EXPECT_TRUE(result.proposals[0].directive.has_requested_value);
  EXPECT_FLOAT_EQ(result.proposals[0].directive.requested_value, 0.5f);
  EXPECT_EQ(result.proposals[1].directive.type,
            acc::ControlDirectiveType::REQUEST_LPI_DWELL);
  EXPECT_FLOAT_EQ(result.proposals[1].directive.requested_value, 0.75f);
}

TEST(TacticalCoordinatorTest, HighRcsCanPromoteThreatWithModerateSpeed) {
  dp::TacticalCoordinator coordinator;
  ext::TacticalStateStore state_store;

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(180.0f, 4.2f);
  const ext::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(PrimaryCategory(result), "HIGH_THREAT_FIGHTER");
}

TEST(TacticalCoordinatorTest, ClassifiesAllTracksInFrame) {
  dp::TacticalCoordinator coordinator;
  ext::TacticalStateStore state_store;

  acm::DecisionInputFrame frame;
  frame.tracks.push_back(MakeTrack(900.0f, 0.0f, 0.0f, 4.5f));
  frame.tracks.push_back(MakeTrack(160.0f, 0.0f, 0.0f, 0.8f));
  frame.tracks[0].status = acm::TrackStatus::kConfirmed;
  frame.tracks[1].status = acm::TrackStatus::kConfirmed;

  const ext::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  ASSERT_EQ(result.target_classification_result.size(), frame.tracks.size());
  EXPECT_EQ(result.target_classification_result[0].target_type, "HIGH_THREAT_FIGHTER");
  EXPECT_EQ(result.target_classification_result[1].target_type, "LOW_THREAT_TARGET");
}

TEST(ThreatAssessmentEvaluatorTest, RepositoryMatchProvidesProbability) {
  edb::FeatureRepository repository;
  de::ThreatAssessmentEvaluator evaluator(&repository);
  ext::TacticalStateStore state_store;

  const acm::DecisionInputFrame frame = BuildSingleTrackFrame(780.0f, 2.8f);

  const de::ThreatAssessmentEvaluator::Result threat_result =
      evaluator.Evaluate(frame, state_store);

  ASSERT_FALSE(threat_result.target_classification_result.empty());
  EXPECT_EQ(threat_result.target_classification_result.front().target_type,
            "HIGH_THREAT_FIGHTER");
  const float prob = threat_result.target_classification_result.front().probability;
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
