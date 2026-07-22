// Copyright 2026. All Rights Reserved.
//
// @file tactical_coordinator_test.cpp
// @brief 验证新决策协调器与控制归并器的基础行为。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>

#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/LpiEvaluator.h"
#include "airborne_radar/decision/TacticalCoordinator.h"

namespace airborne_radar {
namespace tests {

namespace {

session::TrackStateSnapshot BuildTrack(float speed, float rcs, session::TrackStatus status) {
  session::TrackStateSnapshot snapshot;
  snapshot.velocity_x = speed;
  snapshot.velocity_y = 0.0f;
  snapshot.velocity_z = 0.0f;
  snapshot.speed = std::sqrt(speed * speed);
  snapshot.rcs = rcs;
  snapshot.status = status;
  snapshot.association_key = static_cast<std::uint64_t>(speed * 10.0f + rcs * 10.0f);
  return snapshot;
}

bool ContainsDirectiveType(const std::vector<session::TacticalProposal>& proposals,
                           session::ControlDirectiveType type) {
  return std::find_if(proposals.begin(), proposals.end(),
                      [type](const session::TacticalProposal& proposal) {
                        return proposal.directive.type == type;
                      }) != proposals.end();
}

int FindDirectivePriority(const std::vector<session::TacticalProposal>& proposals,
                          session::ControlDirectiveType type) {
  const std::vector<session::TacticalProposal>::const_iterator found =
      std::find_if(proposals.begin(), proposals.end(),
                   [type](const session::TacticalProposal& proposal) {
                     return proposal.directive.type == type;
                   });
  return found == proposals.end() ? -1 : found->priority;
}

std::string FindDirectiveRationale(
    const std::vector<session::TacticalProposal>& proposals,
    session::ControlDirectiveType type) {
  const std::vector<session::TacticalProposal>::const_iterator found =
      std::find_if(proposals.begin(), proposals.end(),
                   [type](const session::TacticalProposal& proposal) {
                     return proposal.directive.type == type;
                   });
  return found == proposals.end() ? std::string() : found->rationale;
}

}  // namespace

TEST(TacticalCoordinatorTest, HighThreatTrackGeneratesLpiProposal) {
  decision::TacticalCoordinator coordinator;
  session::TacticalStateStore state_store;

  session::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(800.0f, 4.0f, session::TrackStatus::kConfirmed));

  const session::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, session::TacticalMode::kThreatResponse);
  ASSERT_FALSE(result.proposals.empty());
  EXPECT_EQ(result.proposals[0].directive.type,
            session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION);
}

TEST(LpiEvaluatorTest, EmitsDynamicPowerAndCoupledDwellAcrossRangeBands) {
  decision::LpiEvaluator evaluator;
  model::LpiSourceInfo info;
  info.has_recon_platform = true;

  struct Case {
    float range_km;
    float closure_speed_mps;
    float expected_power;
    float expected_dwell;
  };
  const Case cases[] = {
      {0.0f, 250.0f, 0.5f, 0.75f},  {10.0f, 250.0f, 0.3f, 0.65f},
      {30.0f, 250.0f, 0.4f, 0.70f}, {30.0f, 100.0f, 0.5f, 0.75f},
      {70.0f, 250.0f, 0.6f, 0.80f}, {70.0f, 100.0f, 0.7f, 0.85f},
      {120.0f, 250.0f, 0.8f, 0.90f},
  };
  for (std::size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    info.threat_range_km = cases[i].range_km;
    info.threat_closure_speed_mps = cases[i].closure_speed_mps;
    std::vector<session::TacticalProposal> proposals;
    const decision::LpiEvaluator::Result result = evaluator.Evaluate(info, &proposals);
    ASSERT_EQ(proposals.size(), 2U);
    EXPECT_FLOAT_EQ(result.power_scale, cases[i].expected_power);
    EXPECT_FLOAT_EQ(result.dwell_scale, cases[i].expected_dwell);
    EXPECT_FLOAT_EQ(proposals[0].directive.requested_value, cases[i].expected_power);
    EXPECT_FLOAT_EQ(proposals[1].directive.requested_value, cases[i].expected_dwell);
  }
}

TEST(TacticalCoordinatorTest, LowEvidenceTrackDoesNotTriggerAggressiveControl) {
  decision::TacticalCoordinator coordinator;
  session::TacticalStateStore state_store;

  session::DecisionInputFrame frame;
  frame.cycle_index = 1u;
  frame.batch_id = 1u;
  frame.tracks.push_back(BuildTrack(900.0f, 5.0f, session::TrackStatus::kTentative));

  const session::TacticalDecisionResult result =
      coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, session::TacticalMode::kBaseline);
  EXPECT_TRUE(result.proposals.empty());
}

TEST(TacticalCoordinatorTest, ReceiverRfObservationGeneratesEccmProposals) {
  decision::TacticalCoordinator coordinator;
  session::TacticalStateStore state_store;

  session::DecisionInputFrame frame;
  frame.cycle_index = 1U;
  frame.batch_id = 1U;
  session::ArInterferenceObservation observation;
  observation.observation_id = 1U;
  observation.estimated_off_boresight_deg = 8.0;
  observation.estimated_center_frequency_hz = 3.0e9;
  observation.estimated_bandwidth_hz = 2.0e6;
  observation.estimated_waveform_kind =
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  observation.jammer_to_noise_db = 12.0;
  frame.interference_observations.push_back(observation);
  frame.tracks.push_back(BuildTrack(200.0f, 2.0f, session::TrackStatus::kConfirmed));

  const session::TacticalDecisionResult result = coordinator.Evaluate(frame, state_store);

  EXPECT_EQ(result.selected_mode, session::TacticalMode::kProtectedEmission);
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY));
  EXPECT_TRUE(ContainsDirectiveType(
      result.proposals, session::ControlDirectiveType::REQUEST_ECCM_REJITTER));
  EXPECT_EQ(state_store.last_decision_summary,
            "protected-emission(receiver-rf-observation)");
}

TEST(TacticalCoordinatorTest, AssociationStressCannotTriggerOrBiasEccm) {
  decision::TacticalCoordinator coordinator;
  session::TacticalStateStore baseline_state_store;
  session::TacticalStateStore stressed_state_store;

  session::DecisionInputFrame baseline_frame;
  baseline_frame.cycle_index = 1U;
  baseline_frame.batch_id = 1U;
  baseline_frame.tracks.push_back(
      BuildTrack(200.0f, 2.0f, session::TrackStatus::kConfirmed));
  session::DecisionInputFrame stressed_frame = baseline_frame;
  stressed_frame.association_quality_info.match_rate = 0.1f;
  stressed_frame.association_quality_info.missed_track_rate = 0.9f;
  stressed_frame.association_quality_info.association_stress = 1.0f;

  const session::TacticalDecisionResult baseline =
      coordinator.Evaluate(baseline_frame, baseline_state_store);
  const session::TacticalDecisionResult stressed =
      coordinator.Evaluate(stressed_frame, stressed_state_store);

  EXPECT_EQ(baseline.selected_mode, session::TacticalMode::kBaseline);
  EXPECT_EQ(stressed.selected_mode, session::TacticalMode::kBaseline);
  EXPECT_TRUE(baseline.proposals.empty());
  EXPECT_TRUE(stressed.proposals.empty());
}
TEST(TacticalCoordinatorTest, LpiProposalStopsWithoutFreshThreatEvidence) {
  decision::TacticalCoordinator coordinator;
  session::TacticalStateStore state_store;

  session::DecisionInputFrame trigger_frame;
  trigger_frame.cycle_index = 1u;
  trigger_frame.batch_id = 1u;
  trigger_frame.tracks.push_back(
      BuildTrack(820.0f, 4.5f, session::TrackStatus::kConfirmed));
  const session::TacticalDecisionResult trigger_result =
      coordinator.Evaluate(trigger_frame, state_store);
  EXPECT_EQ(trigger_result.selected_mode, session::TacticalMode::kThreatResponse);
  EXPECT_TRUE(ContainsDirectiveType(
      trigger_result.proposals, session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));

  session::DecisionInputFrame next_frame;
  next_frame.cycle_index = 2u;
  next_frame.batch_id = 2u;
  const session::TacticalDecisionResult next_result =
      coordinator.Evaluate(next_frame, state_store);
  EXPECT_EQ(next_result.selected_mode, session::TacticalMode::kBaseline);
  EXPECT_FALSE(ContainsDirectiveType(
      next_result.proposals, session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));
}

TEST(TacticalCoordinatorTest, EccmProposalStopsWithoutFreshJammingEvidence) {
  decision::TacticalCoordinator coordinator;
  session::TacticalStateStore state_store;

  session::DecisionInputFrame trigger_frame;
  trigger_frame.cycle_index = 1u;
  trigger_frame.batch_id = 1u;
  session::ArInterferenceObservation observation;
  observation.observation_id = 1U;
  observation.estimated_off_boresight_deg = 8.0;
  observation.estimated_center_frequency_hz = 3.0e9;
  observation.estimated_bandwidth_hz = 2.0e6;
  observation.estimated_waveform_kind =
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  observation.jammer_to_noise_db = 12.0;
  trigger_frame.interference_observations.push_back(observation);
  trigger_frame.tracks.push_back(BuildTrack(260.0f, 2.2f, session::TrackStatus::kConfirmed));
  const session::TacticalDecisionResult trigger_result =
      coordinator.Evaluate(trigger_frame, state_store);
  EXPECT_EQ(trigger_result.selected_mode, session::TacticalMode::kProtectedEmission);

  session::DecisionInputFrame next_frame;
  next_frame.cycle_index = 2u;
  next_frame.batch_id = 2u;
  const session::TacticalDecisionResult next_result =
      coordinator.Evaluate(next_frame, state_store);
  EXPECT_EQ(next_result.selected_mode, session::TacticalMode::kBaseline);
}

TEST(TacticalCoordinatorTest, ControlHoldIsOwnedByReducerAfterProposalStops) {
  decision::TacticalCoordinator coordinator;
  extension::ControlReducerConfig reducer_config;
  reducer_config.lpi_hold_cycles_after_request = 2u;
  decision::ControlReducer reducer(reducer_config);
  session::TacticalStateStore state_store;
  session::ArControlProfile profile;

  session::DecisionInputFrame trigger_frame;
  trigger_frame.cycle_index = 1u;
  trigger_frame.batch_id = 1u;
  trigger_frame.tracks.push_back(
      BuildTrack(820.0f, 4.5f, session::TrackStatus::kConfirmed));

  const session::TacticalDecisionResult trigger_result =
      coordinator.Evaluate(trigger_frame, state_store);
  ASSERT_TRUE(ContainsDirectiveType(
      trigger_result.proposals, session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));

  const extension::ControlReductionResult activated =
      reducer.Reduce(profile, trigger_result.proposals);
  profile = activated.profile;
  ASSERT_TRUE(profile.enable_lpi_power_control);
  EXPECT_EQ(reducer.GetRuntimeState().lpi_hold_cycles_remaining, 2u);

  session::DecisionInputFrame next_frame;
  next_frame.cycle_index = 2u;
  next_frame.batch_id = 2u;
  const session::TacticalDecisionResult next_result =
      coordinator.Evaluate(next_frame, state_store);
  EXPECT_EQ(next_result.selected_mode, session::TacticalMode::kBaseline);
  EXPECT_FALSE(ContainsDirectiveType(
      next_result.proposals, session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION));

  const extension::ControlReductionResult held =
      reducer.Reduce(profile, next_result.proposals);
  EXPECT_TRUE(held.profile.enable_lpi_power_control);
  EXPECT_EQ(reducer.GetRuntimeState().lpi_hold_cycles_remaining, 1u);
}

TEST(TacticalCoordinatorTest, PrunesInactiveTrackStateMemoryByActiveTrackKeys) {
  decision::TacticalCoordinator coordinator;
  session::TacticalStateStore state_store;

  session::DecisionInputFrame frame_a;
  frame_a.cycle_index = 1u;
  frame_a.batch_id = 1u;
  frame_a.tracks.push_back(
      BuildTrack(780.0f, 3.8f, session::TrackStatus::kConfirmed));
  frame_a.tracks.back().association_key = 1001u;
  coordinator.Evaluate(frame_a, state_store);
  ASSERT_EQ(state_store.confidence_memory.size(), 1u);
  ASSERT_EQ(state_store.threat_memory.size(), 1u);
  EXPECT_EQ(state_store.confidence_memory.count(1001u), 1u);
  EXPECT_EQ(state_store.threat_memory.count(1001u), 1u);

  session::DecisionInputFrame frame_b;
  frame_b.cycle_index = 2u;
  frame_b.batch_id = 2u;
  frame_b.tracks.push_back(
      BuildTrack(260.0f, 1.8f, session::TrackStatus::kConfirmed));
  frame_b.tracks.back().association_key = 2002u;
  coordinator.Evaluate(frame_b, state_store);

  EXPECT_EQ(state_store.confidence_memory.size(), 1u);
  EXPECT_EQ(state_store.threat_memory.size(), 1u);
  EXPECT_EQ(state_store.confidence_memory.count(1001u), 0u);
  EXPECT_EQ(state_store.threat_memory.count(1001u), 0u);
  EXPECT_EQ(state_store.confidence_memory.count(2002u), 1u);
  EXPECT_EQ(state_store.threat_memory.count(2002u), 1u);
}

TEST(ControlReducerTest, ReducerBuildsNextControlProfileAndRejectsDuplicates) {
  decision::ControlReducer reducer;
  session::ArControlProfile previous_profile;

  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               session::ControlDirectiveSource::EMISSION_CONTROL, 0.5f),
      60, "reduce power"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               session::ControlDirectiveSource::UNKNOWN, 0.4f),
      10, "duplicate reduce power"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                               session::ControlDirectiveSource::SURVIVABILITY),
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
  decision::ControlReducer reducer;
  session::ArControlProfile previous_profile;

  const std::vector<session::TacticalProposal> proposals{
      session::TacticalProposal{
          session::ControlDirective(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                            session::ControlDirectiveSource::SURVIVABILITY),
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
      reducer.Reduce(third.profile, std::vector<session::TacticalProposal>());
  EXPECT_FALSE(released.profile.enable_agility_frequency);
  EXPECT_EQ(released.profile.agility_frequency_hop_phase, 0u);
}

TEST(ControlReducerTest, HopPhaseTogglingDuringHoldDoesNotIncreaseProfileVersion) {
  extension::ControlReducerConfig config;
  config.eccm_hold_cycles_after_request = 2u;
  decision::ControlReducer reducer(config);

  session::ArControlProfile previous_profile;
  const std::vector<session::TacticalProposal> proposals{
      session::TacticalProposal{
          session::ControlDirective(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                            session::ControlDirectiveSource::SURVIVABILITY),
          84, "enable agility"}};

  const extension::ControlReductionResult activated = reducer.Reduce(previous_profile, proposals);
  ASSERT_TRUE(activated.profile.enable_agility_frequency);
  EXPECT_EQ(activated.profile.agility_frequency_hop_phase, 0u);

  const extension::ControlReductionResult held_once =
      reducer.Reduce(activated.profile, std::vector<session::TacticalProposal>());
  EXPECT_TRUE(held_once.profile.enable_agility_frequency);
  EXPECT_EQ(held_once.profile.agility_frequency_hop_phase, 1u);
  EXPECT_EQ(held_once.profile.version, activated.profile.version);

  const extension::ControlReductionResult held_twice =
      reducer.Reduce(held_once.profile, std::vector<session::TacticalProposal>());
  EXPECT_TRUE(held_twice.profile.enable_agility_frequency);
  EXPECT_EQ(held_twice.profile.agility_frequency_hop_phase, 0u);
  EXPECT_EQ(held_twice.profile.version, activated.profile.version);
}

TEST(ControlReducerTest, BurnthroughGainFloorsLpiPowerReductionForSurvivability) {
  decision::ControlReducer reducer;
  session::ArControlProfile previous_profile;

  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               session::ControlDirectiveSource::EMISSION_CONTROL, 0.5f),
      60, "reduce power"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               session::ControlDirectiveSource::SURVIVABILITY, 1.5f),
      82, "increase burnthrough"});

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_TRUE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.85f);
}

TEST(ControlReducerTest, ReducerSupportsCustomConfigPolicyTable) {
  extension::ControlReducerConfig config;
  config.burnthrough_lpi_power_floor = 0.92f;

  decision::ControlReducer reducer(config);
  session::ArControlProfile previous_profile;

  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                               session::ControlDirectiveSource::EMISSION_CONTROL, 0.65f),
      60, "reduce power"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_DWELL,
                               session::ControlDirectiveSource::EMISSION_CONTROL, 0.60f),
      55, "reduce dwell"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               session::ControlDirectiveSource::SURVIVABILITY, 1.8f),
      82, "increase burnthrough"});

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_FLOAT_EQ(result.profile.eccm_burnthrough_gain, 1.8f);
  EXPECT_FLOAT_EQ(result.profile.lpi_dwell_scale, 0.60f);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 0.92f);
}

TEST(ControlReducerTest, ReducerClearsExpiredDomainWhenNoProposalArrives) {
  decision::ControlReducer reducer;
  session::ArControlProfile previous_profile;
  previous_profile.version = 3u;
  previous_profile.enable_lpi_power_control = true;
  previous_profile.lpi_power_scale = 0.5f;

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, std::vector<session::TacticalProposal>());

  EXPECT_EQ(result.profile.version, 4u);
  EXPECT_FALSE(result.profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(result.profile.lpi_power_scale, 1.0f);
}

TEST(ControlReducerTest, RuntimeConfigClampsActiveHoldAndCooldownWithoutExtendingThem) {
  extension::ControlReducerConfig config;
  config.lpi_hold_cycles_after_request = 10u;
  config.eccm_hold_cycles_after_request = 10u;
  config.lpi_cooldown_cycles_after_release = 10u;
  config.eccm_cooldown_cycles_after_release = 10u;
  decision::ControlReducer reducer(config);

  decision::ControlReducerRuntimeState state;
  state.lpi_hold_cycles_remaining = 10u;
  state.eccm_hold_cycles_remaining = 10u;
  state.lpi_cooldown_cycles_remaining = 10u;
  state.eccm_cooldown_cycles_remaining = 10u;
  reducer.RestoreRuntimeState(state);

  config.lpi_hold_cycles_after_request = 2u;
  config.eccm_hold_cycles_after_request = 2u;
  config.lpi_cooldown_cycles_after_release = 2u;
  config.eccm_cooldown_cycles_after_release = 2u;
  reducer.UpdateConfig(config);
  state = reducer.GetRuntimeState();
  EXPECT_EQ(state.lpi_hold_cycles_remaining, 2u);
  EXPECT_EQ(state.eccm_hold_cycles_remaining, 2u);
  EXPECT_EQ(state.lpi_cooldown_cycles_remaining, 2u);
  EXPECT_EQ(state.eccm_cooldown_cycles_remaining, 2u);

  config.lpi_hold_cycles_after_request = 5u;
  config.eccm_hold_cycles_after_request = 5u;
  config.lpi_cooldown_cycles_after_release = 5u;
  config.eccm_cooldown_cycles_after_release = 5u;
  reducer.UpdateConfig(config);
  state = reducer.GetRuntimeState();
  EXPECT_EQ(state.lpi_hold_cycles_remaining, 2u);
  EXPECT_EQ(state.eccm_hold_cycles_remaining, 2u);
  EXPECT_EQ(state.lpi_cooldown_cycles_remaining, 2u);
  EXPECT_EQ(state.eccm_cooldown_cycles_remaining, 2u);

  config.lpi_hold_cycles_after_request = 0u;
  config.eccm_hold_cycles_after_request = 0u;
  config.lpi_cooldown_cycles_after_release = 0u;
  config.eccm_cooldown_cycles_after_release = 0u;
  reducer.UpdateConfig(config);
  state = reducer.GetRuntimeState();
  EXPECT_EQ(state.lpi_hold_cycles_remaining, 0u);
  EXPECT_EQ(state.eccm_hold_cycles_remaining, 0u);
  EXPECT_EQ(state.lpi_cooldown_cycles_remaining, 0u);
  EXPECT_EQ(state.eccm_cooldown_cycles_remaining, 0u);
}

TEST(ControlReducerTest, IncreasedRuntimeConfigAppliesToNextNewWindow) {
  decision::ControlReducer reducer;
  extension::ControlReducerConfig config;
  config.lpi_hold_cycles_after_request = 5u;
  config.eccm_cooldown_cycles_after_release = 5u;
  reducer.UpdateConfig(config);

  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.7f),
      80, "start a new LPI hold window"});
  session::ArControlProfile inactive_profile;
  const extension::ControlReductionResult activated =
      reducer.Reduce(inactive_profile, proposals);
  EXPECT_TRUE(activated.profile.enable_lpi_power_control);
  EXPECT_EQ(reducer.GetRuntimeState().lpi_hold_cycles_remaining, 5u);

  decision::ControlReducerRuntimeState state = reducer.GetRuntimeState();
  state.lpi_hold_cycles_remaining = 0u;
  reducer.RestoreRuntimeState(state);
  session::ArControlProfile active_eccm_profile;
  active_eccm_profile.eccm_burnthrough_gain = 1.5f;
  reducer.Reduce(active_eccm_profile, std::vector<session::TacticalProposal>());
  EXPECT_EQ(reducer.GetRuntimeState().eccm_cooldown_cycles_remaining, 5u);
}

TEST(ControlReducerTest, ReducerPreservesDomainDuringConfiguredHoldWindow) {
  extension::ControlReducerConfig config;
  config.eccm_hold_cycles_after_request = 1u;
  decision::ControlReducer reducer(config);
  session::ArControlProfile previous_profile;

  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               session::ControlDirectiveSource::SURVIVABILITY, 1.5f),
      82, "increase burnthrough"});

  const extension::ControlReductionResult first =
      reducer.Reduce(previous_profile, proposals);
  ASSERT_FLOAT_EQ(first.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_hold_cycles_remaining, 1u);

  const extension::ControlReductionResult held =
      reducer.Reduce(first.profile, std::vector<session::TacticalProposal>());
  EXPECT_FLOAT_EQ(held.profile.eccm_burnthrough_gain, 1.5f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_hold_cycles_remaining, 0u);
  EXPECT_EQ(held.profile.version, first.profile.version);

  const extension::ControlReductionResult released =
      reducer.Reduce(held.profile, std::vector<session::TacticalProposal>());
  EXPECT_FLOAT_EQ(released.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(released.profile.version, held.profile.version + 1u);
}

TEST(ControlReducerTest, ReducerRejectsReentryDuringConfiguredCooldown) {
  extension::ControlReducerConfig config;
  config.eccm_cooldown_cycles_after_release = 2u;
  decision::ControlReducer reducer(config);

  session::ArControlProfile active_profile;
  active_profile.version = 7u;
  active_profile.eccm_burnthrough_gain = 1.5f;

  const extension::ControlReductionResult released =
      reducer.Reduce(active_profile, std::vector<session::TacticalProposal>());
  EXPECT_FLOAT_EQ(released.profile.eccm_burnthrough_gain, 1.0f);
  EXPECT_EQ(reducer.GetRuntimeState().eccm_cooldown_cycles_remaining, 2u);

  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               session::ControlDirectiveSource::SURVIVABILITY, 1.5f),
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

TEST(ControlReducerTest, RejectsMissingNonFiniteOutOfRangeAndUnexpectedValues) {
  decision::ControlReducer reducer;
  session::ArControlProfile previous_profile;
  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL),
      60, "missing power"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_DWELL,
                                session::ControlDirectiveSource::EMISSION_CONTROL,
                                std::numeric_limits<float>::infinity()),
      55, "non-finite dwell"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL,
                                std::numeric_limits<float>::quiet_NaN()),
      59, "nan power"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.0f),
      58, "zero power"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_DWELL,
                                session::ControlDirectiveSource::EMISSION_CONTROL, 0.2f),
      54, "low dwell"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                                session::ControlDirectiveSource::SURVIVABILITY, 2.1f),
      82, "excessive gain"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                                session::ControlDirectiveSource::SURVIVABILITY, 1.0f),
      81, "inactive gain"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                session::ControlDirectiveSource::SURVIVABILITY, 1.0f),
      84, "unexpected scalar"});

  const extension::ControlReductionResult result = reducer.Reduce(previous_profile, proposals);
  EXPECT_EQ(result.applied_directives.size(), 0u);
  EXPECT_EQ(result.rejected_directives.size(), 8u);
  EXPECT_EQ(result.profile.version, 0u);
}

TEST(ControlReducerTest, BeamConflictPrefersSurvivabilityByDefault) {
  decision::ControlReducer reducer;
  session::ArControlProfile previous_profile;

  std::vector<session::TacticalProposal> proposals;
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_LPI_BEAMFORMING,
                               session::ControlDirectiveSource::EMISSION_CONTROL),
      65, "beamforming for lpi"});
  proposals.push_back(session::TacticalProposal{
      session::ControlDirective(session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
                               session::ControlDirectiveSource::SURVIVABILITY),
      85, "adaptive beamforming for eccm"});

  const extension::ControlReductionResult result =
      reducer.Reduce(previous_profile, proposals);

  EXPECT_FALSE(result.profile.enable_lpi_beamforming);
  EXPECT_TRUE(result.profile.enable_adaptive_beamforming);
  EXPECT_EQ(result.applied_directives.size(), 1u);
  EXPECT_EQ(result.rejected_directives.size(), 1u);
  EXPECT_EQ(result.rejected_directives[0].type,
            session::ControlDirectiveType::REQUEST_LPI_BEAMFORMING);
}

}  // namespace tests
}  // namespace airborne_radar
