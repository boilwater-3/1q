/**
 * @file ecm_session_test.cpp
 * @brief 验证 ECM 双模式、资源调度、滑行和快照确定性契约。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>

#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "1q/electronic_countermeasure/EcmSession.h"

namespace electronic_countermeasure {
namespace session {
namespace {

EcmSensorObservation MakeObservation(std::uint64_t id, double center_hz, float threat_score) {
  EcmSensorObservation observation;
  observation.source_hypothesis_id = id;
  observation.estimated_center_frequency_hz = center_hz;
  observation.estimated_bandwidth_hz = 10.0e6;
  observation.estimated_pri_s = 1.0e-3;
  observation.estimated_pulse_width_s = 1.0e-6;
  observation.center_frequency_std_hz = 1000.0;
  observation.bandwidth_std_hz = 2000.0;
  observation.bearing_std_deg = 1.0;
  observation.threat_score = threat_score;
  observation.confidence = 0.9f;
  return observation;
}

EcmCycleInput MakeSensorInput(std::uint32_t cycle_index, bool has_frame) {
  EcmCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kSensorDriven;
  input.platform_entity_id = 900U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_sensor_observation_frame = has_frame;
  if (has_frame) {
    input.sensor_observation_frame.source_esr_success_cycle_index = cycle_index - 1U;
    input.sensor_observation_frame.observations.push_back(
        MakeObservation(10U, 10.0e9, 0.9f));
  }
  return input;
}

TEST(EcmSessionTest, SensorFrameGlidesTwoSuccessfulCyclesThenSafelyStops) {
  EcmSession session = EcmSession::Create();
  const EcmCycleResult fresh = session.StepWithResult(MakeSensorInput(2U, true));
  ASSERT_EQ(fresh.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(fresh.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(fresh.emission_frame.source_esr_success_cycle_index, 1U);

  const EcmCycleResult glide_one = session.StepWithResult(MakeSensorInput(3U, false));
  EXPECT_EQ(glide_one.status, EcmCycleStatus::kExecuted);
  EXPECT_TRUE(glide_one.used_glided_observation);
  EXPECT_EQ(glide_one.observation_age_successful_ecm_cycles, 1U);

  const EcmCycleResult glide_two = session.StepWithResult(MakeSensorInput(4U, false));
  EXPECT_EQ(glide_two.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(glide_two.observation_age_successful_ecm_cycles, 2U);

  const EcmCycleResult expired = session.StepWithResult(MakeSensorInput(5U, false));
  EXPECT_EQ(expired.status, EcmCycleStatus::kSafeStopNoFreshObservation);
  EXPECT_TRUE(expired.emission_frame.emissions.empty());
}

TEST(EcmSessionTest, RejectedMixedModeDoesNotAdvanceSuccessfulState) {
  EcmSession session = EcmSession::Create();
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kExecuted);

  EcmCycleInput mixed = MakeSensorInput(3U, false);
  EcmTruthThreat truth;
  truth.truth_entity_id = 77U;
  truth.center_frequency_hz = 9.0e9;
  truth.bandwidth_hz = 5.0e6;
  truth.threat_score = 1.0f;
  mixed.truth_threats.push_back(truth);
  EXPECT_EQ(session.StepWithResult(mixed).status, EcmCycleStatus::kRejectedInvalidInput);

  const EcmCycleResult retry = session.StepWithResult(MakeSensorInput(3U, false));
  EXPECT_EQ(retry.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(retry.observation_age_successful_ecm_cycles, 1U);
}

TEST(EcmSessionTest, ChannelAndPowerBudgetsAreConservedForAllTechniques) {
  for (EcmTechnique technique :
       {EcmTechnique::kSpot, EcmTechnique::kBarrage, EcmTechnique::kSweep}) {
    config::EcmSessionConfig config;
    config.channel_count = 2U;
    config.maximum_total_transmit_power_w = 1000.0;
    config.maximum_channel_transmit_power_w = 600.0;
    config.default_technique = technique;
    EcmSession session = EcmSession::Create(config);
    EcmCycleInput input = MakeSensorInput(2U, true);
    input.sensor_observation_frame.observations.push_back(
        MakeObservation(11U, 9.0e9, 0.8f));
    input.sensor_observation_frame.observations.push_back(
        MakeObservation(12U, 8.0e9, 0.7f));

    const EcmCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
    ASSERT_EQ(result.emission_frame.emissions.size(), 2U);
    double allocated_power_w = 0.0;
    for (const EcmResourceDecision& decision : result.decisions) {
      EXPECT_LE(decision.allocated_power_w, 600.0);
      allocated_power_w += decision.allocated_power_w;
    }
    EXPECT_LE(allocated_power_w, 1000.0);
    if (technique == EcmTechnique::kSweep) {
      EXPECT_EQ(result.emission_frame.emissions.front().segments.size(),
                config.sweep_segment_count);
    }
  }
}

TEST(EcmSessionTest, TruthAssistedOwnershipIsExplicitAndSeparate) {
  EcmSession session = EcmSession::Create();
  EcmCycleInput input;
  input.cycle_index = 1U;
  input.input_mode = EcmInputMode::kTruthAssisted;
  input.platform_entity_id = 900U;
  EcmTruthThreat truth;
  truth.truth_entity_id = 77U;
  truth.center_frequency_hz = 9.0e9;
  truth.bandwidth_hz = 5.0e6;
  truth.threat_score = 1.0f;
  input.truth_threats.push_back(truth);

  const EcmCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  EXPECT_TRUE(result.truth_assisted);
  ASSERT_EQ(result.decisions.size(), 1U);
  EXPECT_EQ(result.decisions.front().truth_entity_id, 77U);
  EXPECT_EQ(result.emission_frame.emissions.front().entity_id, 900U);
}

TEST(EcmSessionTest, SweepSnapshotContinuationIsDeterministic) {
  config::EcmSessionConfig config;
  config.default_technique = EcmTechnique::kSweep;
  EcmSession session = EcmSession::Create(config);
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kExecuted);
  const EcmRuntimeState snapshot = session.CaptureRuntimeState();

  const EcmCycleResult first = session.StepWithResult(MakeSensorInput(3U, false));
  ASSERT_TRUE(session.RestoreRuntimeState(snapshot));
  const EcmCycleResult replayed = session.StepWithResult(MakeSensorInput(3U, false));

  ASSERT_EQ(first.emission_frame.emissions.size(), 1U);
  ASSERT_EQ(replayed.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(first.emission_frame.emissions.front().emission_id,
            replayed.emission_frame.emissions.front().emission_id);
  ASSERT_EQ(first.emission_frame.emissions.front().segments.size(),
            replayed.emission_frame.emissions.front().segments.size());
  for (std::size_t index = 0U; index < first.emission_frame.emissions.front().segments.size();
       ++index) {
    EXPECT_DOUBLE_EQ(first.emission_frame.emissions.front().segments[index].center_frequency_hz,
                     replayed.emission_frame.emissions.front().segments[index]
                         .center_frequency_hz);
  }
}

TEST(EcmSessionTest, SnapshotFollowsImplAcrossFacadeMoveAndRejectsForeignSession) {
  EcmSession session = EcmSession::Create();
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kExecuted);
  const EcmRuntimeState snapshot = session.CaptureRuntimeState();

  EcmSession moved = std::move(session);
  EXPECT_TRUE(moved.RestoreRuntimeState(snapshot));

  EcmSession foreign = EcmSession::Create();
  const EcmRuntimeState foreign_before = foreign.CaptureRuntimeState();
  EXPECT_FALSE(foreign.RestoreRuntimeState(snapshot));
  const EcmRuntimeState foreign_after = foreign.CaptureRuntimeState();
  EXPECT_EQ(foreign_after.next_emission_id, foreign_before.next_emission_id);
  EXPECT_DOUBLE_EQ(foreign_after.thermal_energy_j, foreign_before.thermal_energy_j);
}

TEST(EcmSessionTest, EsrAdapterCopiesOnlyDetruthEstimatedFields) {
  electronic_surveillance_radar::session::EmitterHypothesis hypothesis;
  hypothesis.hypothesis_id = 55U;
  hypothesis.estimated_center_frequency_hz = 10.0e9;
  hypothesis.estimated_bandwidth_hz = 5.0e6;
  hypothesis.estimated_pri_s = 1.0e-3;
  hypothesis.estimated_pulse_width_s = 1.0e-6;
  hypothesis.center_frequency_std_hz = 1000.0;
  hypothesis.bandwidth_std_hz = 2000.0;
  hypothesis.bearing_std_deg = 1.0f;
  hypothesis.confidence = 0.8f;
  hypothesis.threat_level =
      electronic_surveillance_radar::session::EsrThreatLevel::kHigh;
  electronic_surveillance_radar::session::EmitterHypothesisList hypotheses;
  hypotheses.push_back(hypothesis);

  EcmSensorObservationFrame frame;
  ASSERT_TRUE(TryBuildEcmSensorObservationFrame(hypotheses, 7U, &frame));
  ASSERT_EQ(frame.observations.size(), 1U);
  EXPECT_EQ(frame.source_esr_success_cycle_index, 7U);
  EXPECT_EQ(frame.observations.front().source_hypothesis_id, 55U);
  EXPECT_DOUBLE_EQ(frame.observations.front().estimated_center_frequency_hz, 10.0e9);
  EXPECT_FLOAT_EQ(frame.observations.front().threat_score, 0.8f);
}

}  // namespace
}  // namespace session
}  // namespace electronic_countermeasure
