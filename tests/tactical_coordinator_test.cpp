// Copyright 2026. All Rights Reserved.
//
// Description: 验证新决策协调器与控制归并器的基础行为。

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <set>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/decision/pipeline/ControlReducer.h"
#include "1q/airborne_radar/decision/pipeline/TacticalCoordinator.h"

namespace airborne_radar { namespace tests {

namespace {

common::DecisionTrackSnapshot BuildTrack(
    float speed, float rcs, common::DecisionTrackStatus status,
    bool has_measurement_evidence, bool jamming_detected = false) {
  common::DecisionTrackSnapshot snapshot(
      speed, 0.0f, 0.0f, rcs, 0.0f, 0.0f, 0.0f, jamming_detected);
  snapshot.state.status = status;
  snapshot.evidence.has_measurement_evidence = has_measurement_evidence;
  snapshot.evidence.updated_this_cycle = has_measurement_evidence;
  snapshot.evidence.predicted_only_this_cycle = !has_measurement_evidence;
  snapshot.state.association_key =
      static_cast<std::uint64_t>(speed * 10.0f + rcs * 10.0f);
  return snapshot;
}

bool ContainsDirectiveType(
    const std::vector<decision::pipeline::TacticalProposal>& proposals,
    common::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const decision::pipeline::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

int FindDirectivePriority(const std::vector<decision::pipeline::TacticalProposal>& proposals,
                          common::ControlDirectiveType type) {
  const std::vector<decision::pipeline::TacticalProposal>::const_iterator found =
      std::find_if(proposals.begin(), proposals.end(),
                   [type](const decision::pipeline::TacticalProposal& proposal) {
                     return proposal.directive.type == type;
                   });
  return found == proposals.end() ? -1 : found->priority;
}

std::string FindDirectiveRationale(
    const std::vector<decision::pipeline::TacticalProposal>& proposals,
    common::ControlDirectiveType type) {
  const std::vector<decision::pipeline::TacticalProposal>::const_iterator found =
      std::find_if(proposals.begin(), proposals.end(),
                   [type](const decision::pipeline::TacticalProposal& proposal) {
                     return proposal.directive.type == type;
                   });
  return found == proposals.end() ? std::string() : found->rationale;
}

} // namespace

TEST(TacticalCoordinatorTest, HighThreatTrackGeneratesLpiProposal) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(800.0f, 4.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kThreatResponse);
  ASSERT_FALSE(result.proposals.empty());
  EXPECT_EQ(result.proposals[0].directive.type,
            common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
}

TEST(TacticalCoordinatorTest, LowEvidenceTrackDoesNotTriggerAggressiveControl) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(900.0f, 5.0f,
                                    common::DecisionTrackStatus::kTentative,
                                    false));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, JammingEnvironmentGeneratesEccmProposals) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.environment_jamming_detected = true;
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, true));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kProtectedEmission);
  ASSERT_GE(result.proposals.size(), 5u);

  std::set<common::ControlDirectiveType> directive_types;
  for (std::size_t i = 0; i < result.proposals.size(); ++i) {
    directive_types.insert(result.proposals[i].directive.type);
  }

  EXPECT_NE(directive_types.find(
                common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER),
            directive_types.end());
  EXPECT_NE(
      directive_types.find(common::ControlDirectiveType::
                               REQUEST_ENABLE_ADAPTIVE_BEAMFORMING),
      directive_types.end());
  EXPECT_NE(
      directive_types.find(common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY),
      directive_types.end());
  EXPECT_NE(directive_types.find(
                common::ControlDirectiveType::REQUEST_ECCM_REJITTER),
            directive_types.end());
  EXPECT_NE(directive_types.find(
                common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN),
            directive_types.end());
}

TEST(TacticalCoordinatorTest, DetailedEccmFactsSelectOnlyRelevantProposals) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.eccm_source_info.has_jamming_signal = true;
  frame.eccm_source_info.jammer_power_db = 5.0f;
  frame.eccm_source_info.frequency_overlap_ratio = 0.8f;
  frame.eccm_source_info.prf_lock_risk = 0.2f;
  frame.eccm_source_info.jammer_in_sidelobe = false;
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, true));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals, common::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
}

TEST(TacticalCoordinatorTest,
     MultiSourceEccmFactsCombineTypeSpecificCountermeasures) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.eccm_source_info.has_jamming_signal = true;

  common::EccmJammerSourceInfo noise_source;
  noise_source.technique = common::JammingTechnique::kNoiseSuppression;
  noise_source.jammer_power_db = 11.0f;
  noise_source.jammer_to_signal_db = 8.0f;
  noise_source.frequency_overlap_ratio = 0.2f;
  noise_source.prf_lock_risk = 0.1f;
  noise_source.jammer_in_sidelobe = true;

  common::EccmJammerSourceInfo deception_source;
  deception_source.technique = common::JammingTechnique::kDeception;
  deception_source.jammer_power_db = 4.0f;
  deception_source.jammer_to_signal_db = 5.0f;
  deception_source.frequency_overlap_ratio = 0.85f;
  deception_source.prf_lock_risk = 0.9f;
  deception_source.jammer_in_sidelobe = false;

  frame.eccm_source_info.jammer_sources.push_back(noise_source);
  frame.eccm_source_info.jammer_sources.push_back(deception_source);
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, true));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, common::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
  EXPECT_GT(FindDirectivePriority(
                result.proposals,
                common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER),
            FindDirectivePriority(
                result.proposals,
                common::ControlDirectiveType::
                    REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
}

TEST(TacticalCoordinatorTest,
     LowConfidenceMultiSourceFactsDoNotTriggerAggressiveTypeSpecificActions) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.eccm_source_info.has_jamming_signal = true;

  common::EccmJammerSourceInfo low_confidence_deception;
  low_confidence_deception.technique = common::JammingTechnique::kDeception;
  low_confidence_deception.jammer_power_db = 9.0f;
  low_confidence_deception.jammer_to_signal_db = 8.0f;
  low_confidence_deception.frequency_overlap_ratio = 0.95f;
  low_confidence_deception.prf_lock_risk = 0.95f;
  low_confidence_deception.confidence = 0.2f;

  frame.eccm_source_info.jammer_sources.push_back(low_confidence_deception);
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, true));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals, common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals, common::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
}

TEST(TacticalCoordinatorTest,
     AssociationStressCanBackfillDeceptionDrivenEccmTrigger) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.association_quality_info.dominant_jamming_semantic =
      common::JammingSemantic::kDeception;
  frame.association_quality_info.jamming_severity = 0.7f;
  frame.association_quality_info.association_stress = 0.35f;
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, false));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, common::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER));
  EXPECT_FALSE(ContainsDirectiveType(
      result.proposals,
      common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN));
  EXPECT_EQ(state_store.last_decision_summary,
            "protected-emission(association-pressure)");
}

TEST(TacticalCoordinatorTest,
     AssociationStressRaisesTypeSpecificEccmPriorityWithoutMutatingFacts) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore baseline_state_store;
  decision::pipeline::TacticalStateStore stressed_state_store;

  common::DecisionInputFrame baseline_frame;
  baseline_frame.cycle_index = 1u;
  baseline_frame.batch_id = 1u;
  baseline_frame.eccm_source_info.has_jamming_signal = true;
  baseline_frame.eccm_source_info.frequency_overlap_ratio = 0.7f;
  baseline_frame.eccm_source_info.prf_lock_risk = 0.7f;
  baseline_frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                             common::DecisionTrackStatus::kConfirmed,
                                             true, false));

  common::DecisionInputFrame stressed_frame = baseline_frame;
  stressed_frame.association_quality_info.dominant_jamming_semantic =
      common::JammingSemantic::kDeception;
  stressed_frame.association_quality_info.jamming_severity = 0.8f;
  stressed_frame.association_quality_info.association_stress = 0.5f;
  stressed_frame.association_quality_info.mean_match_cost = 1.2f;
  stressed_frame.association_quality_info.p95_match_cost = 2.4f;

  const decision::pipeline::TacticalDecisionResult baseline_result =
      coordinator.Evaluate(baseline_frame, baseline_state_store);
  const decision::pipeline::TacticalDecisionResult stressed_result =
      coordinator.Evaluate(stressed_frame, stressed_state_store);

  EXPECT_GT(FindDirectivePriority(
                stressed_result.proposals,
                common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY),
            FindDirectivePriority(
                baseline_result.proposals,
                common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_GT(FindDirectivePriority(
                stressed_result.proposals,
                common::ControlDirectiveType::REQUEST_ECCM_REJITTER),
            FindDirectivePriority(
                baseline_result.proposals,
                common::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_NE(FindDirectiveRationale(
                stressed_result.proposals,
                common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY)
                .find("association stress"),
            std::string::npos);
}

TEST(TacticalCoordinatorTest,
     PoorAssociationQualityWithoutJammingSemanticDoesNotTriggerEccm) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.association_quality_info.match_rate = 0.1f;
  frame.association_quality_info.missed_track_rate = 0.8f;
  frame.association_quality_info.mean_match_cost = 2.5f;
  frame.association_quality_info.p95_match_cost = 3.5f;
  frame.association_quality_info.association_stress = 0.45f;
  frame.association_quality_info.jamming_severity = 0.0f;
  frame.association_quality_info.dominant_jamming_semantic =
      common::JammingSemantic::kNone;
  frame.perception_quality_info.input_target_count = 4u;
  frame.perception_quality_info.detection_count = 1u;
  frame.perception_quality_info.detection_rate = 0.25f;
  frame.perception_quality_info.detection_stress = 0.75f;
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, false));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
  EXPECT_EQ(state_store.last_decision_summary, "baseline(detection-pressure)");
}

TEST(TacticalCoordinatorTest,
     EnvironmentJammingAndAssociationPressureAreBothReflectedInSummary) {
  decision::pipeline::TacticalCoordinator coordinator;
  decision::pipeline::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.environment_jamming_detected = true;
  frame.eccm_source_info.has_jamming_signal = true;
  frame.association_quality_info.dominant_jamming_semantic =
      common::JammingSemantic::kMixed;
  frame.association_quality_info.jamming_severity = 0.8f;
  frame.association_quality_info.association_stress = 0.5f;
  frame.perception_quality_info.input_target_count = 4u;
  frame.perception_quality_info.detection_count = 2u;
  frame.perception_quality_info.detection_rate = 0.5f;
  frame.perception_quality_info.detection_stress = 0.5f;
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, true));

  const decision::pipeline::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::pipeline::TacticalMode::kProtectedEmission);
  EXPECT_EQ(
      state_store.last_decision_summary,
      "protected-emission(environment-jamming+association-pressure+detection-pressure)");
}

TEST(ControlReducerTest, ReducerBuildsNextControlProfileAndRejectsDuplicates) {
  decision::pipeline::ControlReducer reducer;
  common::RadarControlProfile previous_profile;

  std::vector<decision::pipeline::TacticalProposal> proposals;
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      60,
      "reduce power"});
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::UNKNOWN),
      10,
      "duplicate reduce power"});
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
          common::ControlDirectiveSource::SURVIVABILITY),
      84,
      "enable agility"});

  const decision::pipeline::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_EQ(result.profile.version, 1u);
  EXPECT_TRUE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.5f);
  EXPECT_TRUE(result.profile.enable_agility_frequency);
  EXPECT_EQ(result.applied_directives.size(), 2u);
  EXPECT_EQ(result.rejected_directives.size(), 1u);
}

TEST(ControlReducerTest,
     BurnthroughGainFloorsLpiPowerReductionForSurvivability) {
  decision::pipeline::ControlReducer reducer;
  common::RadarControlProfile previous_profile;

  std::vector<decision::pipeline::TacticalProposal> proposals;
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      60,
      "reduce power"});
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::ControlDirectiveSource::SURVIVABILITY),
      82,
      "increase burnthrough"});

  const decision::pipeline::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_TRUE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.85f);
}

TEST(ControlReducerTest, ReducerSupportsCustomConfigPolicyTable) {
  decision::pipeline::ControlReducerConfig config;
  config.lpi_power_scale_on_reduction = 0.65f;
  config.lpi_dwell_scale = 0.60f;
  config.eccm_burnthrough_gain = 1.8f;
  config.burnthrough_lpi_power_floor = 0.92f;

  decision::pipeline::ControlReducer reducer(config);
  common::RadarControlProfile previous_profile;

  std::vector<decision::pipeline::TacticalProposal> proposals;
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      60,
      "reduce power"});
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_DWELL,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      55,
      "reduce dwell"});
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::ControlDirectiveSource::SURVIVABILITY),
      82,
      "increase burnthrough"});

  const decision::pipeline::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.8f);
  EXPECT_FLOAT_EQ(result.profile.lpi_dwell_scale, 0.60f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.92f);
}

TEST(ControlReducerTest, ReducerClearsExpiredDomainWhenNoProposalArrives) {
  decision::pipeline::ControlReducer reducer;
  common::RadarControlProfile previous_profile;
  previous_profile.version = 3u;
  previous_profile.enable_lpi_power_control = true;
  previous_profile.lpi_power_scale = 0.5f;

  const decision::pipeline::ControlReductionResult result =
      reducer.Reduce(previous_profile, std::vector<decision::pipeline::TacticalProposal>());

  EXPECT_EQ(result.profile.version, 4u);
  EXPECT_FALSE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 1.0f);
}

TEST(ControlReducerTest, ReducerPreservesDomainDuringConfiguredHoldWindow) {
  decision::pipeline::ControlReducerConfig config;
  config.eccm_hold_cycles_after_request = 1u;
  decision::pipeline::ControlReducer reducer(config);
  common::RadarControlProfile previous_profile;

  std::vector<decision::pipeline::TacticalProposal> proposals;
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::ControlDirectiveSource::SURVIVABILITY),
      82,
      "increase burnthrough"});

  const decision::pipeline::ControlReductionResult first =
      reducer.Reduce(previous_profile, proposals);
  ASSERT_FLOAT_EQ(first.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(first.profile.eccm_hold_cycles_remaining, 1u);

  const decision::pipeline::ControlReductionResult held =
      reducer.Reduce(first.profile, std::vector<decision::pipeline::TacticalProposal>());
  EXPECT_FLOAT_EQ(held.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(held.profile.eccm_hold_cycles_remaining, 0u);
  EXPECT_EQ(held.profile.version, first.profile.version);

  const decision::pipeline::ControlReductionResult released =
      reducer.Reduce(held.profile, std::vector<decision::pipeline::TacticalProposal>());
  EXPECT_FLOAT_EQ(released.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(released.profile.version, held.profile.version + 1u);
}

TEST(ControlReducerTest, ReducerRejectsReentryDuringConfiguredCooldown) {
  decision::pipeline::ControlReducerConfig config;
  config.eccm_cooldown_cycles_after_release = 2u;
  decision::pipeline::ControlReducer reducer(config);

  common::RadarControlProfile active_profile;
  active_profile.version = 7u;
  active_profile.eccm_burnthrough_gain = 1.5f;

  const decision::pipeline::ControlReductionResult released =
      reducer.Reduce(active_profile, std::vector<decision::pipeline::TacticalProposal>());
  EXPECT_FLOAT_EQ(released.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(released.profile.eccm_cooldown_cycles_remaining, 2u);

  std::vector<decision::pipeline::TacticalProposal> proposals;
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::ControlDirectiveSource::SURVIVABILITY),
      82,
      "increase burnthrough"});

  const decision::pipeline::ControlReductionResult blocked_once =
      reducer.Reduce(released.profile, proposals);
  EXPECT_FLOAT_EQ(blocked_once.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(blocked_once.profile.eccm_cooldown_cycles_remaining, 1u);
  EXPECT_EQ(blocked_once.applied_directives.size(), 0u);
  EXPECT_EQ(blocked_once.rejected_directives.size(), 1u);

  const decision::pipeline::ControlReductionResult blocked_twice =
      reducer.Reduce(blocked_once.profile, proposals);
  EXPECT_FLOAT_EQ(blocked_twice.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(blocked_twice.profile.eccm_cooldown_cycles_remaining, 0u);
  EXPECT_EQ(blocked_twice.applied_directives.size(), 0u);
  EXPECT_EQ(blocked_twice.rejected_directives.size(), 1u);

  const decision::pipeline::ControlReductionResult allowed =
      reducer.Reduce(blocked_twice.profile, proposals);
  EXPECT_FLOAT_EQ(allowed.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(allowed.applied_directives.size(), 1u);
}

TEST(ControlReducerTest, BeamConflictPrefersSurvivabilityByDefault) {
  decision::pipeline::ControlReducer reducer;
  common::RadarControlProfile previous_profile;

  std::vector<decision::pipeline::TacticalProposal> proposals;
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      65,
      "beamforming for lpi"});
  proposals.push_back(decision::pipeline::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
          common::ControlDirectiveSource::SURVIVABILITY),
      85,
      "adaptive beamforming for eccm"});

  const decision::pipeline::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_FALSE(result.profile.enable_lpi_beamforming);
  EXPECT_TRUE(result.profile.enable_adaptive_beamforming);
  EXPECT_EQ(result.applied_directives.size(), 1u);
  EXPECT_EQ(result.rejected_directives.size(), 1u);
  EXPECT_EQ(result.rejected_directives[0].type,
            common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING);
}

} } // namespace airborne_radar::tests
