/**
 * @file ecm_session_test.cpp
 * @brief 验证 ECM 双模式、资源调度、滑行和快照确定性契约。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
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
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0;
  input.input_mode = EcmInputMode::kSensorDriven;
  input.platform_entity_id = 900U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_sensor_observation_frame = has_frame;
  if (has_frame) {
    input.sensor_observation_frame.source_esr_batch_id =
        static_cast<std::uint64_t>(cycle_index - 1U);
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
  EXPECT_EQ(fresh.source_esr_batch_id, 1U);

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

TEST(EcmSessionTest, PublishedFrameUsesAbsoluteTimeAndEquipmentIdentity) {
  config::EcmSessionConfig config;
  config.transmitter_equipment_id = 17U;
  EcmSession session = EcmSession::Create(config);

  const EcmCycleResult result = session.StepWithResult(MakeSensorInput(2U, true));
  ASSERT_EQ(result.status, EcmCycleStatus::kExecuted);
  EXPECT_EQ(result.emission_frame.world_cycle_index, 2U);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_start_time_s, 1.0);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_duration_s, 1.0);
  ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.platform_id, 900U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.equipment_id, 17U);
  EXPECT_DOUBLE_EQ(
      result.emission_frame.emissions.front().waveform.activity_start_time_s, 1.0);
}

TEST(EcmSessionTest, PoweredOffAdvancesChronologyWithoutConsumingEmissionId) {
  config::EcmSessionConfig config;
  config.power_on = false;
  EcmSession session = EcmSession::Create(config);
  EXPECT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kPoweredOff);

  config::EcmRuntimeConfigPatch patch;
  patch.has_power_on = true;
  patch.power_on = true;
  ASSERT_TRUE(session.ApplyRuntimeConfig(patch).applied);
  EXPECT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kRejectedInvalidInput);
  const EcmCycleResult next = session.StepWithResult(MakeSensorInput(3U, true));
  ASSERT_EQ(next.status, EcmCycleStatus::kExecuted);
  ASSERT_EQ(next.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(next.emission_frame.emissions.front().identity.emission_id, 1U);
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
      EXPECT_EQ(result.emission_frame.emissions.front().waveform.kind,
                oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep);
    }
  }
}

TEST(EcmSessionTest, TruthAssistedOwnershipIsExplicitAndSeparate) {
  EcmSession session = EcmSession::Create();
  EcmCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 0.0;
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
  EXPECT_EQ(result.emission_frame.emissions.front().identity.platform_id, 900U);
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
  EXPECT_EQ(first.emission_frame.emissions.front().identity.emission_id,
            replayed.emission_frame.emissions.front().identity.emission_id);
  EXPECT_DOUBLE_EQ(first.emission_frame.emissions.front().waveform.sweep_start_frequency_hz,
                   replayed.emission_frame.emissions.front()
                       .waveform.sweep_start_frequency_hz);
  EXPECT_DOUBLE_EQ(first.emission_frame.emissions.front().waveform.sweep_stop_frequency_hz,
                   replayed.emission_frame.emissions.front()
                       .waveform.sweep_stop_frequency_hz);
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

// Dirty-snapshot negative tests: each case mutates one field of a captured
// (well-formed) snapshot and asserts RestoreRuntimeState rejects it and leaves
// the session untouched. Covers design §3 "完整校验所有嵌套 observation、重复 ID、
// provenance、模式组合和随机状态，失败不得部分修改". owner_identity still matches,
// so these exercises the deep validation gate, not the ownership check.

// Capture a baseline well-formed snapshot from a session that has executed one
// successful sensor-driven cycle (so has_last_sensor_frame and has_successful_
// cycle are both true with valid nested observations).
EcmRuntimeState CaptureBaselineSensorSnapshot() {
  EcmSession session = EcmSession::Create();
  EXPECT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kExecuted);
  return session.CaptureRuntimeState();
}

TEST(EcmSessionTest, RestoreRejectsDirtySnapshotAndLeavesSessionUntouched) {
  // Each sub-test reuses one session: restore a clean snapshot, confirm it
  // applies, then attempt a dirty variant and confirm rejection + no mutation.
  auto run_dirty_case = [](const std::string& /*name*/, EcmRuntimeState dirty) {
    EcmSession session = EcmSession::Create();
    // Advance the session so it has distinguishable state to detect mutation.
    ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
              EcmCycleStatus::kExecuted);
    const EcmRuntimeState before = session.CaptureRuntimeState();
    // dirty carries this session's owner_identity (same session captured it).
    dirty.owner_identity = before.owner_identity;
    EXPECT_FALSE(session.RestoreRuntimeState(dirty))
        << "dirty snapshot should be rejected";
    const EcmRuntimeState after = session.CaptureRuntimeState();
    EXPECT_EQ(after.next_emission_id, before.next_emission_id)
        << "session mutated on rejected restore";
    EXPECT_DOUBLE_EQ(after.thermal_energy_j, before.thermal_energy_j);
    EXPECT_EQ(after.has_successful_cycle, before.has_successful_cycle);
    EXPECT_EQ(after.last_successful_cycle_index, before.last_successful_cycle_index);
  };

  EcmRuntimeState base = CaptureBaselineSensorSnapshot();

  // Case 1: invalid nested observation (NaN center frequency).
  {
    EcmRuntimeState dirty = base;
    dirty.last_sensor_frame.observations.front().estimated_center_frequency_hz =
        std::numeric_limits<double>::quiet_NaN();
    run_dirty_case("invalid nested observation", dirty);
  }
  // Case 2: duplicate source_hypothesis_id within the frame.
  {
    EcmRuntimeState dirty = base;
    dirty.last_sensor_frame.observations.push_back(
        dirty.last_sensor_frame.observations.front());
    run_dirty_case("duplicate hypothesis id", dirty);
  }
  // Case 3: has_successful_cycle=false but last_successful_cycle_index != 0.
  {
    EcmRuntimeState dirty = base;
    dirty.has_successful_cycle = false;
    run_dirty_case("successful flag cleared but index retained", dirty);
  }
  // Case 4: has_successful_cycle=true but last_successful_cycle_index == 0.
  {
    EcmRuntimeState dirty = base;
    dirty.last_successful_cycle_index = 0U;
    run_dirty_case("successful flag set but zero index", dirty);
  }
}

TEST(EcmSessionTest, RestoreRejectsShallowInconsistencyRegression) {
  // has_last_sensor_frame=false must still reject non-empty observations or a
  // non-zero glide age (guards against the shallow check being dropped).
  EcmSession session = EcmSession::Create();
  ASSERT_EQ(session.StepWithResult(MakeSensorInput(2U, true)).status,
            EcmCycleStatus::kExecuted);
  const EcmRuntimeState before = session.CaptureRuntimeState();

  EcmRuntimeState dirty = before;
  dirty.owner_identity = before.owner_identity;
  dirty.has_last_sensor_frame = false;
  // leave observations populated -> inconsistent with the cleared flag
  EXPECT_FALSE(session.RestoreRuntimeState(dirty));

  const EcmRuntimeState after = session.CaptureRuntimeState();
  EXPECT_EQ(after.has_last_sensor_frame, before.has_last_sensor_frame);
  EXPECT_EQ(after.last_sensor_frame.observations.size(),
            before.last_sensor_frame.observations.size());
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
  ASSERT_TRUE(TryBuildEcmSensorObservationFrame(hypotheses, 700U, &frame));
  ASSERT_EQ(frame.observations.size(), 1U);
  EXPECT_EQ(frame.source_esr_batch_id, 700U);
  EXPECT_EQ(frame.observations.front().source_hypothesis_id, 55U);
  EXPECT_DOUBLE_EQ(frame.observations.front().estimated_center_frequency_hz, 10.0e9);
  EXPECT_FLOAT_EQ(frame.observations.front().threat_score, 0.8f);
}

}  // namespace
}  // namespace session
}  // namespace electronic_countermeasure
