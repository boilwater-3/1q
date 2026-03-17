// Copyright 2026. All Rights Reserved.
//
// Description: 验证新决策协调器与控制归并器的基础行为。

#include <gtest/gtest.h>

#include <set>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/decision/ControlReducer.h"
#include "1q/airborne_radar/decision/TacticalCoordinator.h"

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

} // namespace

TEST(TacticalCoordinatorTest, HighThreatTrackGeneratesLpiProposal) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(800.0f, 4.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true));

  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::TacticalMode::kThreatResponse);
  ASSERT_FALSE(result.proposals.empty());
  EXPECT_EQ(result.proposals[0].directive.type,
            common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
}

TEST(TacticalCoordinatorTest, LowEvidenceTrackDoesNotTriggerAggressiveControl) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(900.0f, 5.0f,
                                    common::DecisionTrackStatus::kTentative,
                                    false));

  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, JammingEnvironmentGeneratesEccmProposals) {
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

  common::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.environment_jamming_detected = true;
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f,
                                    common::DecisionTrackStatus::kConfirmed,
                                    true, true));

  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::TacticalMode::kProtectedEmission);
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
  decision::TacticalCoordinator coordinator;
  decision::TacticalStateStore state_store;

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

  const decision::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, decision::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(std::find_if(
                  result.proposals.begin(), result.proposals.end(),
                  [](const decision::TacticalProposal& proposal) {
                    return proposal.directive.type ==
                           common::ControlDirectiveType::
                               REQUEST_ENABLE_ADAPTIVE_BEAMFORMING;
                  }) != result.proposals.end());
  EXPECT_TRUE(std::find_if(
                  result.proposals.begin(), result.proposals.end(),
                  [](const decision::TacticalProposal& proposal) {
                    return proposal.directive.type ==
                           common::ControlDirectiveType::
                               REQUEST_AGILITY_FREQUENCY;
                  }) != result.proposals.end());
  EXPECT_TRUE(std::find_if(
                  result.proposals.begin(), result.proposals.end(),
                  [](const decision::TacticalProposal& proposal) {
                    return proposal.directive.type ==
                           common::ControlDirectiveType::
                               REQUEST_ENABLE_SIDELOBE_CANCELLER;
                  }) == result.proposals.end());
  EXPECT_TRUE(std::find_if(
                  result.proposals.begin(), result.proposals.end(),
                  [](const decision::TacticalProposal& proposal) {
                    return proposal.directive.type ==
                           common::ControlDirectiveType::
                               REQUEST_ECCM_REJITTER;
                  }) == result.proposals.end());
  EXPECT_TRUE(std::find_if(
                  result.proposals.begin(), result.proposals.end(),
                  [](const decision::TacticalProposal& proposal) {
                    return proposal.directive.type ==
                           common::ControlDirectiveType::
                               REQUEST_ECCM_BURNTHROUGH_GAIN;
                  }) == result.proposals.end());
}

TEST(ControlReducerTest, ReducerBuildsNextControlProfileAndRejectsDuplicates) {
  decision::ControlReducer reducer;
  common::RadarControlProfile previous_profile;

  std::vector<decision::TacticalProposal> proposals;
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      60,
      "reduce power"});
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::UNKNOWN),
      10,
      "duplicate reduce power"});
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
          common::ControlDirectiveSource::SURVIVABILITY),
      84,
      "enable agility"});

  const decision::ControlReductionResult result =
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
  decision::ControlReducer reducer;
  common::RadarControlProfile previous_profile;

  std::vector<decision::TacticalProposal> proposals;
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      60,
      "reduce power"});
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::ControlDirectiveSource::SURVIVABILITY),
      82,
      "increase burnthrough"});

  const decision::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_TRUE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.85f);
}

TEST(ControlReducerTest, ReducerSupportsCustomConfigPolicyTable) {
  decision::ControlReducerConfig config;
  config.lpi_power_scale_on_reduction = 0.65f;
  config.lpi_dwell_scale = 0.60f;
  config.eccm_burnthrough_gain = 1.8f;
  config.burnthrough_lpi_power_floor = 0.92f;

  decision::ControlReducer reducer(config);
  common::RadarControlProfile previous_profile;

  std::vector<decision::TacticalProposal> proposals;
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      60,
      "reduce power"});
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_LPI_DWELL,
          common::ControlDirectiveSource::EMISSION_CONTROL),
      55,
      "reduce dwell"});
  proposals.push_back(decision::TacticalProposal{
      common::ControlDirective(
          common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
          common::ControlDirectiveSource::SURVIVABILITY),
      82,
      "increase burnthrough"});

  const decision::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.8f);
  EXPECT_FLOAT_EQ(result.profile.lpi_dwell_scale, 0.60f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.92f);
}

} } // namespace airborne_radar::tests
