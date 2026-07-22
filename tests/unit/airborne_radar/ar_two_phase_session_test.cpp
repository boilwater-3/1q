#include <gtest/gtest.h>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace session {
namespace {

ArPrepareCycleInput MakePrepareInput(std::uint64_t cycle_index, double start_time_s) {
  ArPrepareCycleInput input;
  input.world_cycle_index = cycle_index;
  input.window_start_time_s = start_time_s;
  input.window_duration_s = 0.1;
  input.platform_id = 10U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.antenna_boresight_ecef.x = 1.0;
  return input;
}

ArCompleteCycleInput MakeCompleteInput(const ArPrepareCycleResult& prepared) {
  ArCompleteCycleInput input;
  input.rf_scene.world_cycle_index = prepared.token.world_cycle_index;
  input.rf_scene.window_start_time_s = prepared.receiver_state.window_start_time_s;
  input.rf_scene.window_duration_s = prepared.receiver_state.window_duration_s;
  input.rf_scene.emissions.push_back(prepared.emission);
  return input;
}

ArSceneTarget MakeTarget() {
  ArSceneTarget target;
  target.external_target_id = 1U;
  target.range_m = 1000.0f;
  target.position_x = 1000.0f;
  target.rcs = 10.0f;
  return target;
}

oneq::electromagnetics::RfSceneEmission MakeInBandJammer(const ArPrepareCycleResult& prepared) {
  oneq::electromagnetics::RfSceneEmission jammer;
  jammer.identity.platform_id = 20U;
  jammer.identity.equipment_id = 21U;
  jammer.identity.emission_id = 22U;
  jammer.position_ecef_m = prepared.receiver_state.position_ecef_m;
  jammer.position_ecef_m.x_m += 1000.0;
  jammer.antenna.boresight_ecef.x = -1.0;
  jammer.antenna.peak_gain_dbi = 35.0;
  jammer.polarization = prepared.receiver_state.polarization;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      prepared.receiver_state.window_start_time_s, prepared.receiver_state.window_duration_s,
      prepared.receiver_state.center_frequency_hz, prepared.receiver_state.bandwidth_hz, 1.0e6,
      &jammer.waveform));
  return jammer;
}

TEST(ArTwoPhaseSessionTest, EnforcesSingleTokenAndRetainsItAfterRejectedComplete) {
  ArSession session = ArSession::Create();
  const ArPrepareCycleResult prepared = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(prepared.status, ArPrepareCycleStatus::kPrepared);
  ASSERT_TRUE(prepared.has_emission);
  EXPECT_NE(prepared.token.value, 0U);

  EXPECT_EQ(session.PrepareCycle(MakePrepareInput(2U, 10.1)).status, ArPrepareCycleStatus::kBusy);

  ArCompleteCycleInput mismatched = MakeCompleteInput(prepared);
  mismatched.rf_scene.window_start_time_s += 1.0;
  EXPECT_EQ(session.CompleteCycle(prepared.token, mismatched).status,
            ArCompleteCycleStatus::kRejected);

  const ArCompleteCycleResult completed =
      session.CompleteCycle(prepared.token, MakeCompleteInput(prepared));
  EXPECT_EQ(completed.status, ArCompleteCycleStatus::kCompleted);
  EXPECT_EQ(completed.world_cycle_index, 1U);
  EXPECT_EQ(session.CompleteCycle(prepared.token, MakeCompleteInput(prepared)).status,
            ArCompleteCycleStatus::kTokenMismatch);
}

TEST(ArTwoPhaseSessionTest, AbandonReleasesReceiveStageWithoutReusingEmissionIdentity) {
  ArSession session = ArSession::Create();
  const ArPrepareCycleResult first = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(first.status, ArPrepareCycleStatus::kPrepared);
  EXPECT_EQ(session.AbandonCycle(first.token), ArAbandonCycleStatus::kAbandoned);

  const ArPrepareCycleResult second = session.PrepareCycle(MakePrepareInput(2U, 10.1));
  ASSERT_EQ(second.status, ArPrepareCycleStatus::kPrepared);
  EXPECT_GT(second.emission.identity.emission_id, first.emission.identity.emission_id);
  EXPECT_EQ(session.AbandonCycle(first.token), ArAbandonCycleStatus::kTokenMismatch);
}

TEST(ArTwoPhaseSessionTest, PoweredOffAdvancesChronologyWithoutPublishingEmission) {
  config::ArSessionConfig config;
  config.mission.power_on = false;
  ArSession session = ArSession::Create(config);
  const ArPrepareCycleResult powered_off = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  EXPECT_EQ(powered_off.status, ArPrepareCycleStatus::kPoweredOff);
  EXPECT_FALSE(powered_off.has_emission);

  ArPrepareCycleInput stale = MakePrepareInput(2U, 10.05);
  EXPECT_EQ(session.PrepareCycle(stale).status, ArPrepareCycleStatus::kRejected);
}

TEST(ArTwoPhaseSessionTest, PatchSubmittedAfterPrepareWaitsForNextPrepare) {
  ArSession session = ArSession::Create();
  const ArPrepareCycleResult prepared = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(prepared.status, ArPrepareCycleStatus::kPrepared);

  config::ArRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));

  EXPECT_EQ(session.CompleteCycle(prepared.token, MakeCompleteInput(prepared)).status,
            ArCompleteCycleStatus::kCompleted);
  const ArPrepareCycleResult next = session.PrepareCycle(MakePrepareInput(2U, 10.1));
  EXPECT_EQ(next.status, ArPrepareCycleStatus::kPoweredOff);
  EXPECT_FALSE(next.has_emission);
}

TEST(ArTwoPhaseSessionTest, FrontEndSaturationCompletesWithStructuredImpairment) {
  config::ArSessionConfig config;
  config.hardware.receiver.maximum_linear_input_power_w = 1.0e-12f;
  ArSession session = ArSession::Create(config);
  const ArPrepareCycleResult prepared = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(prepared.status, ArPrepareCycleStatus::kPrepared);

  const ArCompleteCycleResult completed =
      session.CompleteCycle(prepared.token, MakeCompleteInput(prepared));
  EXPECT_EQ(completed.status, ArCompleteCycleStatus::kCompleted);
  EXPECT_EQ(completed.receiver_impairment, ArReceiverImpairment::kSaturated);
  EXPECT_TRUE(completed.track_output_frame.tracks.empty());
  EXPECT_TRUE(completed.interference_observations.empty());
}

TEST(ArTwoPhaseSessionTest, MissingPreparedReceiverCoSitePathRejectsAndRetainsToken) {
  config::ArSessionConfig config;
  config.hardware.receiver.co_site_paths.clear();
  ArSession session = ArSession::Create(config);
  const ArPrepareCycleResult prepared = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(prepared.status, ArPrepareCycleStatus::kPrepared);

  EXPECT_EQ(session.CompleteCycle(prepared.token, MakeCompleteInput(prepared)).status,
            ArCompleteCycleStatus::kRejected);
  EXPECT_EQ(session.AbandonCycle(prepared.token), ArAbandonCycleStatus::kAbandoned);
}

TEST(ArTwoPhaseSessionTest, RfSceneSuppressionReachesDetectionOnlyThroughReceivedPower) {
  config::ArSessionConfig config;
  config.hardware.receiver.maximum_linear_input_power_w = 1.0e6f;
  config.policy.lifecycle.confirm_hits = 1U;
  config.policy.detection.minimum_snr_db = -100.0f;
  config.policy.detection.minimum_detection_margin_db = -100.0f;

  ArSession baseline_session = ArSession::Create(config);
  const ArPrepareCycleResult baseline_prepared =
      baseline_session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(baseline_prepared.status, ArPrepareCycleStatus::kPrepared);
  ArCompleteCycleInput baseline_input = MakeCompleteInput(baseline_prepared);
  baseline_input.targets.push_back(MakeTarget());
  const ArCompleteCycleResult baseline =
      baseline_session.CompleteCycle(baseline_prepared.token, baseline_input);
  ASSERT_EQ(baseline.status, ArCompleteCycleStatus::kCompleted);
  ASSERT_FALSE(baseline.track_output_frame.tracks.empty());
  EXPECT_TRUE(baseline.interference_observations.empty());

  ArSession jammed_session = ArSession::Create(config);
  const ArPrepareCycleResult jammed_prepared =
      jammed_session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(jammed_prepared.status, ArPrepareCycleStatus::kPrepared);
  ArCompleteCycleInput jammed_input = MakeCompleteInput(jammed_prepared);
  jammed_input.targets.push_back(MakeTarget());
  jammed_input.rf_scene.emissions.push_back(MakeInBandJammer(jammed_prepared));
  const ArCompleteCycleResult jammed =
      jammed_session.CompleteCycle(jammed_prepared.token, jammed_input);
  ASSERT_EQ(jammed.status, ArCompleteCycleStatus::kCompleted);
  EXPECT_TRUE(jammed.track_output_frame.tracks.empty());
  ASSERT_EQ(jammed.interference_observations.size(), 1U);
  EXPECT_EQ(jammed.interference_observations.front().observation_id, 1U);
  EXPECT_GT(jammed.interference_observations.front().jammer_to_noise_db,
            config.hardware.receiver.interference_observation_jn_gate_db);
}

TEST(ArTwoPhaseSessionTest, ExternalEccmChangesNextPreparedOperatingState) {
  config::ArSessionConfig config;
  config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  config.hardware.transmitter.maximum_peak_power_w = 2.0e6f;
  ArSession session = ArSession::Create(config);
  const ArPrepareCycleResult first = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(first.status, ArPrepareCycleStatus::kPrepared);
  const ArCompleteCycleResult first_complete =
      session.CompleteCycle(first.token, MakeCompleteInput(first));
  ASSERT_EQ(first_complete.status, ArCompleteCycleStatus::kCompleted);

  ExternalDecisionResponse response;
  response.source_cycle_index = first_complete.track_output_frame.cycle_index;
  response.source_batch_id = first_complete.track_output_frame.batch_id;
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                        ControlDirectiveSource::SURVIVABILITY),
                       90, "agility"});
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_ECCM_REJITTER,
                                        ControlDirectiveSource::SURVIVABILITY),
                       89, "rejitter"});
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                                        ControlDirectiveSource::SURVIVABILITY, 1.5f),
                       88, "burnthrough"});
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
                                        ControlDirectiveSource::SURVIVABILITY),
                       87, "sidelobe"});
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
                                        ControlDirectiveSource::SURVIVABILITY),
                       86, "adaptive"});
  ASSERT_EQ(session.SubmitExternalDecision(response), ExternalDecisionSubmitStatus::kAccepted);

  const ArPrepareCycleResult second = session.PrepareCycle(MakePrepareInput(2U, 10.1));
  ASSERT_EQ(second.status, ArPrepareCycleStatus::kPrepared);
  EXPECT_DOUBLE_EQ(second.emission.waveform.center_frequency_hz, 3.1e9);
  EXPECT_GT(second.emission.waveform.pulse_jitter_fraction, 0.0);
  EXPECT_GT(second.emission.waveform.transmit_power_w, first.emission.waveform.transmit_power_w);
  EXPECT_LT(second.receiver_state.antenna.sidelobe_level_db,
            first.receiver_state.antenna.sidelobe_level_db);
  EXPECT_LT(second.receiver_state.antenna.half_power_beamwidth_deg,
            first.receiver_state.antenna.half_power_beamwidth_deg);
}

TEST(ArTwoPhaseSessionTest, ReceiverObservationDrivesNextPreparedOperatingState) {
  config::ArSessionConfig config;
  config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  config.hardware.transmitter.maximum_peak_power_w = 2.0e6f;
  config.hardware.receiver.maximum_linear_input_power_w = 1.0e6f;
  ArSession session = ArSession::Create(config);

  const ArPrepareCycleResult first = session.PrepareCycle(MakePrepareInput(1U, 10.0));
  ASSERT_EQ(first.status, ArPrepareCycleStatus::kPrepared);
  ArCompleteCycleInput complete_input = MakeCompleteInput(first);
  complete_input.rf_scene.emissions.push_back(MakeInBandJammer(first));
  const ArCompleteCycleResult completed = session.CompleteCycle(first.token, complete_input);
  ASSERT_EQ(completed.status, ArCompleteCycleStatus::kCompleted);
  ASSERT_EQ(completed.interference_observations.size(), 1U);
  EXPECT_NEAR(completed.interference_observations[0].estimated_off_boresight_deg, 0.0, 1.0e-9);

  const ArPrepareCycleResult second = session.PrepareCycle(MakePrepareInput(2U, 10.1));
  ASSERT_EQ(second.status, ArPrepareCycleStatus::kPrepared);
  EXPECT_DOUBLE_EQ(second.emission.waveform.center_frequency_hz, 3.1e9);
  EXPECT_GT(second.emission.waveform.transmit_power_w, first.emission.waveform.transmit_power_w);
  EXPECT_LT(second.receiver_state.antenna.half_power_beamwidth_deg,
            first.receiver_state.antenna.half_power_beamwidth_deg);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
