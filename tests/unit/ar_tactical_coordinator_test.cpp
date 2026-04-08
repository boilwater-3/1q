// Copyright 2026. All Rights Reserved.
//
// @file tactical_coordinator_test.cpp
// @brief 验证新决策协调器与控制归并器的基础行为。

#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <string>

#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/DecisionTrackSnapshot.h"
#include "airborne_radar/decision/pipeline/ControlReducer.h"
#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"

namespace airborne_radar {
namespace tests {

namespace {

model::DecisionTrackSnapshot BuildTrack(float speed, float rcs, model::DecisionTrackStatus status,
                                         bool has_measurement_evidence,
                                         bool jamming_detected = false) {
  model::DecisionTrackSnapshot snapshot(speed, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f,
                                         jamming_detected);
  snapshot.state.status = status;
  snapshot.evidence.has_measurement_evidence = has_measurement_evidence;
  snapshot.evidence.updated_this_cycle = has_measurement_evidence;
  snapshot.evidence.predicted_only_this_cycle = !has_measurement_evidence;
  snapshot.state.association_key = static_cast<std::uint64_t>(speed * 10.0f + rcs * 10.0f);
  return snapshot;
}

bool ContainsDirectiveType(const std::vector<extension::TacticalProposal>& proposals,
                           extension::control::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const extension::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

int FindDirectivePriority(const std::vector<extension::TacticalProposal>& proposals,
                          extension::control::ControlDirectiveType type) {
  const std::vector<extension::TacticalProposal>::const_iterator found =
      std::find_if(proposals.begin(), proposals.end(),
                   [type](const extension::TacticalProposal& proposal) {
                     return proposal.directive.type == type;
                   });
  return found == proposals.end() ? -1 : found->priority;
}

std::string FindDirectiveRationale(
    const std::vector<extension::TacticalProposal>& proposals,
    extension::control::ControlDirectiveType type) {
  const std::vector<extension::TacticalProposal>::const_iterator found =
      std::find_if(proposals.begin(), proposals.end(),
                   [type](const extension::TacticalProposal& proposal) {
                     return proposal.directive.type == type;
                   });
  return found == proposals.end() ? std::string() : found->rationale;
}

}  // namespace

TEST(TacticalCoordinatorTest, HighThreatTrackGeneratesLpiProposal) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(800.0f, 4.0f, model::DecisionTrackStatus::kConfirmed, true));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kThreatResponse);
  ASSERT_FALSE(result.proposals.empty());
  EXPECT_EQ(result.proposals[0].directive.type,
            extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
}

TEST(TacticalCoordinatorTest, LowEvidenceTrackDoesNotTriggerAggressiveControl) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(900.0f, 5.0f, model::DecisionTrackStatus::kTentative, false));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, JammingEnvironmentGeneratesEccmProposals) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.environment_jamming_detected = true;
  frame.eccm_source_info.has_jamming_signal = true;
  model::EccmJammerSourceInfo noise_source;
  noise_source.technique = model::JammingTechnique::kNoiseSuppression;
  noise_source.jammer_power_db = 11.0f;
  noise_source.jammer_to_signal_db = 8.0f;
  noise_source.frequency_overlap_ratio = 0.2f;
  noise_source.prf_lock_risk = 0.1f;
  noise_source.jammer_in_sidelobe = true;
  noise_source.confidence = 1.0f;

  model::EccmJammerSourceInfo deception_source;
  deception_source.technique = model::JammingTechnique::kDeception;
  deception_source.jammer_power_db = 5.0f;
  deception_source.jammer_to_signal_db = 7.0f;
  deception_source.frequency_overlap_ratio = 0.85f;
  deception_source.prf_lock_risk = 0.9f;
  deception_source.jammer_in_sidelobe = false;
  deception_source.confidence = 1.0f;
  frame.eccm_source_info.jammer_sources.push_back(noise_source);
  frame.eccm_source_info.jammer_sources.push_back(deception_source);
  frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, true));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kProtectedEmission);
  ASSERT_GE(result.proposals.size(), 5u);

  std::set<extension::control::ControlDirectiveType> directive_types;
  for (std::size_t i = 0; i < result.proposals.size(); ++i) {
    directive_types.insert(result.proposals[i].directive.type);
  }

  EXPECT_NE(directive_types.find(extension::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER),
            directive_types.end());
  EXPECT_NE(directive_types.find(extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING),
            directive_types.end());
  EXPECT_NE(directive_types.find(extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY),
            directive_types.end());
  EXPECT_NE(directive_types.find(extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER),
            directive_types.end());
  EXPECT_NE(directive_types.find(extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN),
            directive_types.end());
}

TEST(TacticalCoordinatorTest, DetailedEccmFactsSelectOnlyRelevantProposals) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.eccm_source_info.has_jamming_signal = true;
  model::EccmJammerSourceInfo source;
  source.technique = model::JammingTechnique::kUnknown;
  source.jammer_power_db = 5.0f;
  source.jammer_to_signal_db = 3.0f;
  source.frequency_overlap_ratio = 0.8f;
  source.prf_lock_risk = 0.2f;
  source.jammer_in_sidelobe = false;
  source.confidence = 1.0f;
  frame.eccm_source_info.jammer_sources.push_back(source);
  frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, true));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals, extension::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_FALSE(
      ContainsDirectiveType(result.proposals, extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_FALSE(ContainsDirectiveType(result.proposals,
                                     extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
}

TEST(TacticalCoordinatorTest, LowConfidenceEccmSourceDoesNotGetArtificialWeightBoost) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.eccm_source_info.has_jamming_signal = true;

  model::EccmJammerSourceInfo source;
  source.technique = model::JammingTechnique::kDeception;
  source.jammer_power_db = 4.0f;
  source.jammer_to_signal_db = 3.0f;
  source.frequency_overlap_ratio = 1.0f;
  source.prf_lock_risk = 0.1f;
  source.jammer_in_sidelobe = false;
  source.confidence = 0.36f;
  frame.eccm_source_info.jammer_sources.push_back(source);
  frame.tracks.push_back(
      BuildTrack(220.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, true));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kProtectedEmission);
  EXPECT_FALSE(ContainsDirectiveType(result.proposals,
                                     extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
}

TEST(TacticalCoordinatorTest, MultiSourceEccmFactsCombineTypeSpecificCountermeasures) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.eccm_source_info.has_jamming_signal = true;

  model::EccmJammerSourceInfo noise_source;
  noise_source.technique = model::JammingTechnique::kNoiseSuppression;
  noise_source.jammer_power_db = 11.0f;
  noise_source.jammer_to_signal_db = 8.0f;
  noise_source.frequency_overlap_ratio = 0.2f;
  noise_source.prf_lock_risk = 0.1f;
  noise_source.jammer_in_sidelobe = true;

  model::EccmJammerSourceInfo deception_source;
  deception_source.technique = model::JammingTechnique::kDeception;
  deception_source.jammer_power_db = 4.0f;
  deception_source.jammer_to_signal_db = 5.0f;
  deception_source.frequency_overlap_ratio = 0.85f;
  deception_source.prf_lock_risk = 0.9f;
  deception_source.jammer_in_sidelobe = false;

  frame.eccm_source_info.jammer_sources.push_back(noise_source);
  frame.eccm_source_info.jammer_sources.push_back(deception_source);
  frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, true));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, extension::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(
      ContainsDirectiveType(result.proposals, extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
  EXPECT_GT(
      FindDirectivePriority(result.proposals,
                            extension::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER),
      FindDirectivePriority(result.proposals,
                            extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
}

TEST(TacticalCoordinatorTest,
     LowConfidenceMultiSourceFactsDoNotTriggerAggressiveTypeSpecificActions) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.eccm_source_info.has_jamming_signal = true;

  model::EccmJammerSourceInfo low_confidence_deception;
  low_confidence_deception.technique = model::JammingTechnique::kDeception;
  low_confidence_deception.jammer_power_db = 9.0f;
  low_confidence_deception.jammer_to_signal_db = 8.0f;
  low_confidence_deception.frequency_overlap_ratio = 0.95f;
  low_confidence_deception.prf_lock_risk = 0.95f;
  low_confidence_deception.confidence = 0.2f;

  frame.eccm_source_info.jammer_sources.push_back(low_confidence_deception);
  frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, true));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_FALSE(ContainsDirectiveType(result.proposals,
                                     extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_FALSE(
      ContainsDirectiveType(result.proposals, extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_FALSE(ContainsDirectiveType(result.proposals,
                                     extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
}

TEST(TacticalCoordinatorTest, AssociationStressCanBackfillDeceptionDrivenEccmTrigger) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.association_quality_info.dominant_jamming_semantic = model::JammingSemantic::kDeception;
  frame.association_quality_info.jamming_severity = 0.7f;
  frame.association_quality_info.association_stress = 0.35f;
  frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, false));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(result.proposals,
                                    extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(
      ContainsDirectiveType(result.proposals, extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals, extension::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_FALSE(ContainsDirectiveType(result.proposals,
                                     extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
  EXPECT_EQ(state_store.last_decision_summary, "protected-emission(association-pressure)");
}

TEST(TacticalCoordinatorTest, AssociationStressRaisesTypeSpecificEccmPriorityWithoutMutatingFacts) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore baseline_state_store;
  extension::TacticalStateStore stressed_state_store;

  model::DecisionInputFrame baseline_frame;
  baseline_frame.cycle_index = 1u;
  baseline_frame.batch_id = 1u;
  baseline_frame.eccm_source_info.has_jamming_signal = true;
  model::EccmJammerSourceInfo baseline_source;
  baseline_source.technique = model::JammingTechnique::kDeception;
  baseline_source.jammer_power_db = 6.0f;
  baseline_source.jammer_to_signal_db = 7.0f;
  baseline_source.frequency_overlap_ratio = 0.7f;
  baseline_source.prf_lock_risk = 0.7f;
  baseline_source.confidence = 1.0f;
  baseline_frame.eccm_source_info.jammer_sources.push_back(baseline_source);
  baseline_frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, false));

  model::DecisionInputFrame stressed_frame = baseline_frame;
  stressed_frame.association_quality_info.dominant_jamming_semantic =
      model::JammingSemantic::kDeception;
  stressed_frame.association_quality_info.jamming_severity = 0.8f;
  stressed_frame.association_quality_info.association_stress = 0.5f;
  stressed_frame.association_quality_info.mean_match_cost = 1.2f;
  stressed_frame.association_quality_info.p95_match_cost = 2.4f;

  const extension::TacticalDecisionResult baseline_result =
      coordinator.Evaluate(baseline_frame, baseline_state_store);
  const extension::TacticalDecisionResult stressed_result =
      coordinator.Evaluate(stressed_frame, stressed_state_store);

  EXPECT_GT(FindDirectivePriority(stressed_result.proposals,
                                  extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY),
            FindDirectivePriority(baseline_result.proposals,
                                  extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_GT(FindDirectivePriority(stressed_result.proposals,
                                  extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER),
            FindDirectivePriority(baseline_result.proposals,
                                  extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_NE(FindDirectiveRationale(stressed_result.proposals,
                                   extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY)
                .find("association stress"),
            std::string::npos);
}

TEST(TacticalCoordinatorTest, PoorAssociationQualityWithoutJammingSemanticDoesNotTriggerEccm) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.association_quality_info.match_rate = 0.1f;
  frame.association_quality_info.missed_track_rate = 0.8f;
  frame.association_quality_info.mean_match_cost = 2.5f;
  frame.association_quality_info.p95_match_cost = 3.5f;
  frame.association_quality_info.association_stress = 0.45f;
  frame.association_quality_info.jamming_severity = 0.0f;
  frame.association_quality_info.dominant_jamming_semantic = model::JammingSemantic::kNone;
  frame.perception_quality_info.input_target_count = 4u;
  frame.perception_quality_info.detection_count = 1u;
  frame.perception_quality_info.detection_rate = 0.25f;
  frame.perception_quality_info.detection_stress = 0.75f;
  frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, false));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
  EXPECT_EQ(state_store.last_decision_summary, "baseline(detection-pressure)");
}

TEST(TacticalCoordinatorTest, EnvironmentJammingAndAssociationPressureAreBothReflectedInSummary) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.environment_jamming_detected = true;
  frame.eccm_source_info.has_jamming_signal = true;
  frame.association_quality_info.dominant_jamming_semantic = model::JammingSemantic::kMixed;
  frame.association_quality_info.jamming_severity = 0.8f;
  frame.association_quality_info.association_stress = 0.5f;
  frame.perception_quality_info.input_target_count = 4u;
  frame.perception_quality_info.detection_count = 2u;
  frame.perception_quality_info.detection_rate = 0.5f;
  frame.perception_quality_info.detection_stress = 0.5f;
  frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, model::DecisionTrackStatus::kConfirmed, true, true));

  const extension::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, extension::TacticalMode::kProtectedEmission);
  EXPECT_EQ(state_store.last_decision_summary,
            "protected-emission(environment-jamming+association-pressure+detection-pressure)");
}

TEST(TacticalCoordinatorTest, LpiProposalStopsWithoutFreshThreatEvidence) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame trigger_frame;
  trigger_frame.cycle_index = 1u;
  trigger_frame.batch_id = 1u;
  trigger_frame.tracks.push_back(
      BuildTrack(820.0f, 4.5f, model::DecisionTrackStatus::kConfirmed, true));
  const extension::TacticalDecisionResult trigger_result =
      coordinator.Evaluate(trigger_frame, state_store);
  EXPECT_EQ(trigger_result.selected_mode, extension::TacticalMode::kThreatResponse);
  EXPECT_TRUE(ContainsDirectiveType(
      trigger_result.proposals, extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));
  EXPECT_EQ(state_store.lpi_hold_cycles_remaining, 0u);

  model::DecisionInputFrame next_frame;
  next_frame.cycle_index = 2u;
  next_frame.batch_id = 2u;
  const extension::TacticalDecisionResult next_result =
      coordinator.Evaluate(next_frame, state_store);
  EXPECT_EQ(next_result.selected_mode, extension::TacticalMode::kBaseline);
  EXPECT_FALSE(ContainsDirectiveType(
      next_result.proposals, extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));
  EXPECT_EQ(state_store.lpi_hold_cycles_remaining, 0u);
}

TEST(TacticalCoordinatorTest, EccmProposalStopsWithoutFreshJammingEvidence) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame trigger_frame;
  trigger_frame.cycle_index = 1u;
  trigger_frame.batch_id = 1u;
  trigger_frame.environment_jamming_detected = true;
  trigger_frame.tracks.push_back(
      BuildTrack(260.0f, 2.2f, model::DecisionTrackStatus::kConfirmed, true, true));
  const extension::TacticalDecisionResult trigger_result =
      coordinator.Evaluate(trigger_frame, state_store);
  EXPECT_EQ(trigger_result.selected_mode, extension::TacticalMode::kProtectedEmission);
  EXPECT_EQ(state_store.eccm_hold_cycles_remaining, 0u);

  model::DecisionInputFrame next_frame;
  next_frame.cycle_index = 2u;
  next_frame.batch_id = 2u;
  const extension::TacticalDecisionResult next_result =
      coordinator.Evaluate(next_frame, state_store);
  EXPECT_EQ(next_result.selected_mode, extension::TacticalMode::kBaseline);
  EXPECT_EQ(state_store.eccm_hold_cycles_remaining, 0u);
}

TEST(TacticalCoordinatorTest, ControlHoldIsOwnedByReducerAfterProposalStops) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::ControlReducerConfig reducer_config;
  reducer_config.lpi_hold_cycles_after_request = 2u;
  decision::pipeline::ControlReducer reducer(reducer_config);
  extension::TacticalStateStore state_store;
  extension::control::RadarControlProfile profile;

  model::DecisionInputFrame trigger_frame;
  trigger_frame.cycle_index = 1u;
  trigger_frame.batch_id = 1u;
  trigger_frame.tracks.push_back(
      BuildTrack(820.0f, 4.5f, model::DecisionTrackStatus::kConfirmed, true));

  const extension::TacticalDecisionResult trigger_result =
      coordinator.Evaluate(trigger_frame, state_store);
  ASSERT_TRUE(ContainsDirectiveType(
      trigger_result.proposals, extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));

  const extension::ControlReductionResult activated =
      reducer.Reduce(profile, trigger_result.proposals);
  profile = activated.profile;
  ASSERT_TRUE(profile.enable_lpi_power_control);
  EXPECT_EQ(reducer.GetRuntimeState().lpi_hold_cycles_remaining, 2u);

  model::DecisionInputFrame next_frame;
  next_frame.cycle_index = 2u;
  next_frame.batch_id = 2u;
  const extension::TacticalDecisionResult next_result =
      coordinator.Evaluate(next_frame, state_store);
  EXPECT_EQ(next_result.selected_mode, extension::TacticalMode::kBaseline);
  EXPECT_FALSE(ContainsDirectiveType(
      next_result.proposals, extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));

  const extension::ControlReductionResult held =
      reducer.Reduce(profile, next_result.proposals);
  EXPECT_TRUE(held.profile.enable_lpi_power_control);
  EXPECT_EQ(reducer.GetRuntimeState().lpi_hold_cycles_remaining, 1u);
}

TEST(TacticalCoordinatorTest, PrunesInactiveTrackStateMemoryByActiveTrackKeys) {
  decision::pipeline::TacticalCoordinator coordinator;
  extension::TacticalStateStore state_store;

  model::DecisionInputFrame frame_a;
  frame_a.cycle_index = 1u;
  frame_a.batch_id = 1u;
  frame_a.tracks.push_back(
      BuildTrack(780.0f, 3.8f, model::DecisionTrackStatus::kConfirmed, true));
  frame_a.tracks.back().state.association_key = 1001u;
  coordinator.Evaluate(frame_a, state_store);
  ASSERT_EQ(state_store.confidence_memory.size(), 1u);
  ASSERT_EQ(state_store.threat_memory.size(), 1u);
  EXPECT_EQ(state_store.confidence_memory.count(1001u), 1u);
  EXPECT_EQ(state_store.threat_memory.count(1001u), 1u);

  model::DecisionInputFrame frame_b;
  frame_b.cycle_index = 2u;
  frame_b.batch_id = 2u;
  frame_b.tracks.push_back(
      BuildTrack(260.0f, 1.8f, model::DecisionTrackStatus::kConfirmed, true));
  frame_b.tracks.back().state.association_key = 2002u;
  coordinator.Evaluate(frame_b, state_store);

  EXPECT_EQ(state_store.confidence_memory.size(), 1u);
  EXPECT_EQ(state_store.threat_memory.size(), 1u);
  EXPECT_EQ(state_store.confidence_memory.count(1001u), 0u);
  EXPECT_EQ(state_store.threat_memory.count(1001u), 0u);
  EXPECT_EQ(state_store.confidence_memory.count(2002u), 1u);
  EXPECT_EQ(state_store.threat_memory.count(2002u), 1u);
}

TEST(ControlReducerTest, ReducerBuildsNextControlProfileAndRejectsDuplicates) {
  decision::pipeline::ControlReducer reducer;
  extension::control::RadarControlProfile previous_profile;

  std::vector<extension::TacticalProposal> proposals;
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               extension::control::ControlDirectiveSource::EMISSION_CONTROL),
      60, "reduce power"});
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               extension::control::ControlDirectiveSource::UNKNOWN),
      10, "duplicate reduce power"});
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                               extension::control::ControlDirectiveSource::SURVIVABILITY),
      84, "enable agility"});

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_EQ(result.profile.version, 1u);
  EXPECT_TRUE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.5f);
  EXPECT_TRUE(result.profile.enable_agility_frequency);
  EXPECT_EQ(result.applied_directives.size(), 2u);
  EXPECT_EQ(result.rejected_directives.size(), 1u);
}

TEST(ControlReducerTest, AgilityFrequencyHopPhaseAlternatesAcrossConsecutiveEnabledCycles) {
  decision::pipeline::ControlReducer reducer;
  extension::control::RadarControlProfile previous_profile;

  const std::vector<extension::TacticalProposal> proposals{
      extension::TacticalProposal{
          extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                            extension::control::ControlDirectiveSource::SURVIVABILITY),
          84, "enable agility"}};

  const extension::ControlReductionResult first = reducer.Reduce(previous_profile, proposals);
  EXPECT_TRUE(first.profile.enable_agility_frequency);
  EXPECT_EQ(first.profile.agility_frequency_hop_phase, 0u);

  const extension::ControlReductionResult second = reducer.Reduce(first.profile, proposals);
  EXPECT_TRUE(second.profile.enable_agility_frequency);
  EXPECT_EQ(second.profile.agility_frequency_hop_phase, 1u);
  EXPECT_EQ(second.profile.version, first.profile.version);

  const extension::ControlReductionResult third = reducer.Reduce(second.profile, proposals);
  EXPECT_TRUE(third.profile.enable_agility_frequency);
  EXPECT_EQ(third.profile.agility_frequency_hop_phase, 0u);
  EXPECT_EQ(third.profile.version, first.profile.version);

  const extension::ControlReductionResult released =
      reducer.Reduce(third.profile, std::vector<extension::TacticalProposal>());
  EXPECT_FALSE(released.profile.enable_agility_frequency);
  EXPECT_EQ(released.profile.agility_frequency_hop_phase, 0u);
}

TEST(ControlReducerTest, HopPhaseTogglingDuringHoldDoesNotIncreaseProfileVersion) {
  extension::ControlReducerConfig config;
  config.eccm_hold_cycles_after_request = 2u;
  decision::pipeline::ControlReducer reducer(config);

  extension::control::RadarControlProfile previous_profile;
  const std::vector<extension::TacticalProposal> proposals{
      extension::TacticalProposal{
          extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                            extension::control::ControlDirectiveSource::SURVIVABILITY),
          84, "enable agility"}};

  const extension::ControlReductionResult activated = reducer.Reduce(previous_profile, proposals);
  ASSERT_TRUE(activated.profile.enable_agility_frequency);
  EXPECT_EQ(activated.profile.agility_frequency_hop_phase, 0u);

  const extension::ControlReductionResult held_once =
      reducer.Reduce(activated.profile, std::vector<extension::TacticalProposal>());
  EXPECT_TRUE(held_once.profile.enable_agility_frequency);
  EXPECT_EQ(held_once.profile.agility_frequency_hop_phase, 1u);
  EXPECT_EQ(held_once.profile.version, activated.profile.version);

  const extension::ControlReductionResult held_twice =
      reducer.Reduce(held_once.profile, std::vector<extension::TacticalProposal>());
  EXPECT_TRUE(held_twice.profile.enable_agility_frequency);
  EXPECT_EQ(held_twice.profile.agility_frequency_hop_phase, 0u);
  EXPECT_EQ(held_twice.profile.version, activated.profile.version);
}

TEST(ControlReducerTest, BurnthroughGainFloorsLpiPowerReductionForSurvivability) {
  decision::pipeline::ControlReducer reducer;
  extension::control::RadarControlProfile previous_profile;

  std::vector<extension::TacticalProposal> proposals;
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               extension::control::ControlDirectiveSource::EMISSION_CONTROL),
      60, "reduce power"});
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               extension::control::ControlDirectiveSource::SURVIVABILITY),
      82, "increase burnthrough"});

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_TRUE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.85f);
}

TEST(ControlReducerTest, ReducerSupportsCustomConfigPolicyTable) {
  extension::ControlReducerConfig config;
  config.lpi_power_scale_on_reduction = 0.65f;
  config.lpi_dwell_scale = 0.60f;
  config.eccm_burnthrough_gain = 1.8f;
  config.burnthrough_lpi_power_floor = 0.92f;

  decision::pipeline::ControlReducer reducer(config);
  extension::control::RadarControlProfile previous_profile;

  std::vector<extension::TacticalProposal> proposals;
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               extension::control::ControlDirectiveSource::EMISSION_CONTROL),
      60, "reduce power"});
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_LPI_DWELL,
                               extension::control::ControlDirectiveSource::EMISSION_CONTROL),
      55, "reduce dwell"});
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               extension::control::ControlDirectiveSource::SURVIVABILITY),
      82, "increase burnthrough"});

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.8f);
  EXPECT_FLOAT_EQ(result.profile.lpi_dwell_scale, 0.60f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.92f);
}

TEST(ControlReducerTest, ReducerClearsExpiredDomainWhenNoProposalArrives) {
  decision::pipeline::ControlReducer reducer;
  extension::control::RadarControlProfile previous_profile;
  previous_profile.version = 3u;
  previous_profile.enable_lpi_power_control = true;
  previous_profile.lpi_power_scale = 0.5f;

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, std::vector<extension::TacticalProposal>());

  EXPECT_EQ(result.profile.version, 4u);
  EXPECT_FALSE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 1.0f);
}

TEST(ControlReducerTest, ReducerPreservesDomainDuringConfiguredHoldWindow) {
  extension::ControlReducerConfig config;
  config.eccm_hold_cycles_after_request = 1u;
  decision::pipeline::ControlReducer reducer(config);
  extension::control::RadarControlProfile previous_profile;

  std::vector<extension::TacticalProposal> proposals;
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               extension::control::ControlDirectiveSource::SURVIVABILITY),
      82, "increase burnthrough"});

  const extension::ControlReductionResult first =
      reducer.Reduce(previous_profile, proposals);
  ASSERT_FLOAT_EQ(first.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_hold_cycles_remaining, 1u);

  const extension::ControlReductionResult held =
      reducer.Reduce(first.profile, std::vector<extension::TacticalProposal>());
  EXPECT_FLOAT_EQ(held.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_hold_cycles_remaining, 0u);
  EXPECT_EQ(held.profile.version, first.profile.version);

  const extension::ControlReductionResult released =
      reducer.Reduce(held.profile, std::vector<extension::TacticalProposal>());
  EXPECT_FLOAT_EQ(released.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(released.profile.version, held.profile.version + 1u);
}

TEST(ControlReducerTest, ReducerRejectsReentryDuringConfiguredCooldown) {
  extension::ControlReducerConfig config;
  config.eccm_cooldown_cycles_after_release = 2u;
  decision::pipeline::ControlReducer reducer(config);

  extension::control::RadarControlProfile active_profile;
  active_profile.version = 7u;
  active_profile.eccm_burnthrough_gain = 1.5f;

  const extension::ControlReductionResult released =
      reducer.Reduce(active_profile, std::vector<extension::TacticalProposal>());
  EXPECT_FLOAT_EQ(released.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_cooldown_cycles_remaining, 2u);

  std::vector<extension::TacticalProposal> proposals;
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               extension::control::ControlDirectiveSource::SURVIVABILITY),
      82, "increase burnthrough"});

  const extension::ControlReductionResult blocked_once =
      reducer.Reduce(released.profile, proposals);
  EXPECT_FLOAT_EQ(blocked_once.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_cooldown_cycles_remaining, 1u);
  EXPECT_EQ(blocked_once.applied_directives.size(), 0u);
  EXPECT_EQ(blocked_once.rejected_directives.size(), 1u);

  const extension::ControlReductionResult blocked_twice =
      reducer.Reduce(blocked_once.profile, proposals);
  EXPECT_FLOAT_EQ(blocked_twice.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_cooldown_cycles_remaining, 0u);
  EXPECT_EQ(blocked_twice.applied_directives.size(), 0u);
  EXPECT_EQ(blocked_twice.rejected_directives.size(), 1u);

  const extension::ControlReductionResult allowed =
      reducer.Reduce(blocked_twice.profile, proposals);
  EXPECT_FLOAT_EQ(allowed.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(allowed.applied_directives.size(), 1u);
}

TEST(ControlReducerTest, BeamConflictPrefersSurvivabilityByDefault) {
  decision::pipeline::ControlReducer reducer;
  extension::control::RadarControlProfile previous_profile;

  std::vector<extension::TacticalProposal> proposals;
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_LPI_BEAMFORMING,
                               extension::control::ControlDirectiveSource::EMISSION_CONTROL),
      65, "beamforming for lpi"});
  proposals.push_back(extension::TacticalProposal{
      extension::control::ControlDirective(extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
                               extension::control::ControlDirectiveSource::SURVIVABILITY),
      85, "adaptive beamforming for eccm"});

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_FALSE(result.profile.enable_lpi_beamforming);
  EXPECT_TRUE(result.profile.enable_adaptive_beamforming);
  EXPECT_EQ(result.applied_directives.size(), 1u);
  EXPECT_EQ(result.rejected_directives.size(), 1u);
  EXPECT_EQ(result.rejected_directives[0].type,
            extension::control::ControlDirectiveType::REQUEST_LPI_BEAMFORMING);
}

}  // namespace tests
}  // namespace airborne_radar
