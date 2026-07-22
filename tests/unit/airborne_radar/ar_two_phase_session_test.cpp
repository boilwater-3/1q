#include <gtest/gtest.h>

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

}  // namespace
}  // namespace session
}  // namespace airborne_radar
