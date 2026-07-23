#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArReplaySession.h"
#include "1q/airborne_radar/session/ArTraceSession.h"
#include "1q/airborne_radar/session/ControlDirective.h"
#include "1q/replay/ReplayTrace.h"
#include "airborne_radar/session/ArReplayCycleRecord.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace {

std::string MakeTraceDir(const char* prefix) {
  std::ostringstream stream;
  stream << "/tmp/" << prefix << "-" << std::time(nullptr) << "-"
         << std::chrono::high_resolution_clock::now().time_since_epoch().count()
         << "-" << std::rand() << ".trace";
  return stream.str();
}

ArCycleInput MakeCycleInput(std::uint32_t cycle, double start_time_s) {
  ArCycleInput input;
  input.cycle_index = cycle;
  input.cycle_start_time_s = start_time_s;
  input.dt_sec = 0.1;
  input.platform.platform_entity_id = 10U;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  return input;
}

std::shared_ptr<oneq::replay::ReplayTraceWriter> MakeWriter(
    const std::string& trace_dir) {
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-single-cycle-v3";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "ar-single-cycle-replay-test";
  return std::shared_ptr<oneq::replay::ReplayTraceWriter>(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
}

void WriteEvent(oneq::replay::ReplayTraceWriter* writer, const char* type,
                const char* payload_type, const std::string& payload) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = type;
  event.payload_type = payload_type;
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = payload;
  ASSERT_EQ(writer->WriteEvent(event),
            oneq::replay::ReplayTraceWriteStatus::kSuccess);
}

TEST(ArRfTraceSessionTest, SingleCycleInputAndOutputReplayExactly) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-cycle-complete");
  {
    const auto writer = MakeWriter(trace_dir);
    ArTraceSessionOptions options;
    options.replay_writer = writer;
    ArTraceSession traced(config::ArSessionConfig{}, options);

    const ArCycleResult result =
        traced.StepWithResult(MakeCycleInput(1U, 10.0));
    ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(writer->Flush(),
              oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 1U);
}

TEST(ArRfTraceSessionTest, RejectedCycleAndSameCycleRetryReplayExactly) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-cycle-retry");
  {
    const auto writer = MakeWriter(trace_dir);
    ArTraceSessionOptions options;
    options.replay_writer = writer;
    ArTraceSession traced(config::ArSessionConfig{}, options);

    ArCycleInput invalid = MakeCycleInput(2U, 20.0);
    invalid.platform.platform_entity_id = 0U;
    EXPECT_EQ(traced.StepWithResult(invalid).status,
              ArCycleStatus::kRejectedInvalidInput);

    config::ArRuntimeConfigPatch patch;
    patch.has_scan_center_deg = true;
    patch.scan_center_deg.az_deg = 4.0f;
    EXPECT_TRUE(traced.TryApplyRuntimeConfig(patch));

    const ArCycleResult retried =
        traced.StepWithResult(MakeCycleInput(2U, 20.0));
    ASSERT_EQ(retried.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(retried.emission_frame.emissions.size(), 1U);
    EXPECT_EQ(retried.emission_frame.emissions.front().identity.emission_id,
              1U);
    ASSERT_EQ(writer->Flush(),
              oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }
  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 2U);
  EXPECT_EQ(replay.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(ArRfTraceSessionTest, RejectedPatchAndDecisionAttemptsReplayExactly) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-cycle-attempts");
  {
    const auto writer = MakeWriter(trace_dir);
    ArTraceSessionOptions options;
    options.replay_writer = writer;
    ArTraceSession traced(config::ArSessionConfig{}, options);

    config::ArRuntimeConfigPatch empty_patch;
    EXPECT_FALSE(traced.TryApplyRuntimeConfig(empty_patch));
    ExternalDecisionResponse response;
    EXPECT_EQ(traced.SubmitExternalDecision(response),
              ExternalDecisionSubmitStatus::kNoPendingObservation);
    ASSERT_EQ(writer->Flush(),
              oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }
  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay.playback.applied_decision_input_count, 1U);
}

TEST(ArRfTraceSessionTest, AcceptedExternalDecisionReplaysIntoNextCycle) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-cycle-decision");
  {
    const auto writer = MakeWriter(trace_dir);
    ArTraceSessionOptions options;
    options.replay_writer = writer;
    config::ArSessionConfig config;
    config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
    ArTraceSession traced(config, options);

    const ArCycleResult first =
        traced.StepWithResult(MakeCycleInput(5U, 50.0));
    ASSERT_EQ(first.status, ArCycleStatus::kCompleted);
    ASSERT_TRUE(first.has_decision_observation);

    ExternalDecisionResponse response;
    response.source_cycle_index =
        first.decision_observation.input_frame.cycle_index;
    response.source_batch_id =
        first.decision_observation.input_frame.batch_id;
    response.proposals.push_back(TacticalProposal{
        ControlDirective(ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                         ControlDirectiveSource::SURVIVABILITY),
        90, "agility"});
    ASSERT_EQ(traced.SubmitExternalDecision(response),
              ExternalDecisionSubmitStatus::kAccepted);

    const ArCycleResult second =
        traced.StepWithResult(MakeCycleInput(6U, 50.1));
    ASSERT_EQ(second.status, ArCycleStatus::kCompleted);
    ASSERT_EQ(second.emission_frame.emissions.size(), 1U);
    EXPECT_DOUBLE_EQ(
        second.emission_frame.emissions.front().waveform.center_frequency_hz,
        3.1e9);
    ASSERT_EQ(writer->Flush(),
              oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 2U);
  EXPECT_EQ(replay.playback.applied_decision_input_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(ArRfTraceSessionTest, DetectsSingleCycleStateDivergence) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-cycle-divergence");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-cycle-v3-divergence";
  manifest.module = "airborne_radar";
  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  const config::ArSessionConfig config;
  WriteEvent(&writer, "session_config", "ArSessionConfig",
             EncodeSessionConfigFlatbuffer(config));

  const ArCycleInput input = MakeCycleInput(4U, 40.0);
  WriteEvent(&writer, "cycle_input", "ArCycleInputV3",
             EncodeCycleInputFlatbuffer(input));
  ArSession original = ArSession::Create(config);
  ArCycleReplayRecord record;
  record.result = original.StepWithResult(input);
  record.session_state = ArSessionReplayAccess::CaptureSessionState(original);
  ++record.session_state.next_emission_id;
  WriteEvent(&writer, "cycle_output", "ArCycleReplayRecordV3",
             EncodeCycleReplayRecordFlatbuffer(record));
  ASSERT_EQ(writer.Flush(),
            oneq::replay::ReplayTraceWriteStatus::kSuccess);

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_TRUE(replay.playback.divergence_found);
  EXPECT_NE(replay.first_error.find("ArCycleReplayRecordV3"),
            std::string::npos);
}

TEST(ArRfTraceSessionTest, RejectsRetiredTwoStageTracePayload) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-cycle-legacy-reject");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-retired-two-stage";
  manifest.module = "airborne_radar";
  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  WriteEvent(&writer, "session_config", "ArSessionConfig",
             EncodeSessionConfigFlatbuffer(config::ArSessionConfig{}));
  WriteEvent(&writer, "cycle_input", "ArPrepareCycleInputV2",
             "retired-two-stage-payload");
  ASSERT_EQ(writer.Flush(),
            oneq::replay::ReplayTraceWriteStatus::kSuccess);

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.first_error.find("legacy"), std::string::npos);
}

TEST(ArRfTraceSessionTest, RejectsTrailingCycleInputWithoutOutput) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-cycle-trailing-input");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-cycle-v3-trailing-input";
  manifest.module = "airborne_radar";
  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  WriteEvent(&writer, "session_config", "ArSessionConfig",
             EncodeSessionConfigFlatbuffer(config::ArSessionConfig{}));
  WriteEvent(&writer, "cycle_input", "ArCycleInputV3",
             EncodeCycleInputFlatbuffer(MakeCycleInput(7U, 70.0)));
  ASSERT_EQ(writer.Flush(),
            oneq::replay::ReplayTraceWriteStatus::kSuccess);

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.first_error.find("pending cycle_input"),
            std::string::npos);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
