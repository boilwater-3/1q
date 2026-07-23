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
         << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "-"
         << std::rand() << ".trace";
  return stream.str();
}

ArPrepareCycleInput MakePrepareInput(std::uint64_t cycle, double start_time_s) {
  ArPrepareCycleInput input;
  input.world_cycle_index = cycle;
  input.window_start_time_s = start_time_s;
  input.window_duration_s = 0.1;
  input.platform_id = 10U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  return input;
}

ArCompleteCycleInput MakeCompleteInput(const ArPrepareCycleResult& prepared) {
  ArCompleteCycleInput input;
  input.rf_scene.world_cycle_index = prepared.token.world_cycle_index;
  input.rf_scene.window_start_time_s = prepared.operating_state.rf_receiver.window_start_time_s;
  input.rf_scene.window_duration_s = prepared.operating_state.rf_receiver.window_duration_s;
  input.rf_scene.emissions.push_back(prepared.emission);
  return input;
}

std::shared_ptr<oneq::replay::ReplayTraceWriter> MakeWriter(const std::string& trace_dir) {
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-rf-v2";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "rf-v2-replay-test";
  return std::shared_ptr<oneq::replay::ReplayTraceWriter>(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
}

void WriteEvent(oneq::replay::ReplayTraceWriter* writer, const char* type, const char* payload_type,
                const std::string& payload) {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = type;
  event.payload_type = payload_type;
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = payload;
  ASSERT_EQ(writer->WriteEvent(event), oneq::replay::ReplayTraceWriteStatus::kSuccess);
}

TEST(ArRfTraceSessionTest, PrepareIsRecordedImmediatelyAndCompleteReplays) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-rf-complete");
  {
    const auto writer = MakeWriter(trace_dir);
    ArTraceSessionOptions options;
    options.replay_writer = writer;
    ArTraceSession traced(config::ArSessionConfig{}, options);

    const ArPrepareCycleResult prepared = traced.PrepareCycle(MakePrepareInput(1U, 10.0));
    ASSERT_EQ(prepared.status, ArPrepareCycleStatus::kPrepared);

    oneq::replay::ReplayTraceReader reader(trace_dir);
    oneq::replay::ReplayTraceReadEvent event;
    bool saw_prepare_output = false;
    while (reader.ReadNextEvent(&event) == oneq::replay::ReplayTraceReadStatus::kEvent) {
      if (event.payload_type == "ArPrepareReplayRecordV2") {
        saw_prepare_output = true;
      }
    }
    EXPECT_TRUE(saw_prepare_output);

    const ArCompleteCycleResult completed =
        traced.CompleteCycle(prepared.token, MakeCompleteInput(prepared));
    ASSERT_EQ(completed.status, ArCompleteCycleStatus::kCompleted);
    ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 2U);
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(ArRfTraceSessionTest, RejectedCompleteRetainsTokenForRecordedRetry) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-rf-retry");
  {
    const auto writer = MakeWriter(trace_dir);
    ArTraceSessionOptions options;
    options.replay_writer = writer;
    ArTraceSession traced(config::ArSessionConfig{}, options);
    const ArPrepareCycleResult prepared = traced.PrepareCycle(MakePrepareInput(2U, 20.0));
    ASSERT_EQ(prepared.status, ArPrepareCycleStatus::kPrepared);

    ArCompleteCycleInput rejected_input = MakeCompleteInput(prepared);
    rejected_input.rf_scene.window_start_time_s += 1.0;
    EXPECT_EQ(traced.CompleteCycle(prepared.token, rejected_input).status,
              ArCompleteCycleStatus::kRejected);

    config::ArRuntimeConfigPatch staged_patch;
    staged_patch.has_scan_center_deg = true;
    staged_patch.scan_center_deg.az_deg = 4.0f;
    EXPECT_TRUE(traced.TryApplyRuntimeConfig(staged_patch));

    EXPECT_EQ(traced.CompleteCycle(prepared.token, MakeCompleteInput(prepared)).status,
              ArCompleteCycleStatus::kCompleted);
    ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }
  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 3U);
  EXPECT_EQ(replay.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 3U);
}

TEST(ArRfTraceSessionTest, AbandonAndRejectedAttemptsReplayExactly) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-rf-abandon");
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

    const ArPrepareCycleResult prepared = traced.PrepareCycle(MakePrepareInput(3U, 30.0));
    ASSERT_EQ(prepared.status, ArPrepareCycleStatus::kPrepared);
    EXPECT_EQ(traced.AbandonCycle(prepared.token), ArAbandonCycleStatus::kAbandoned);
    ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }
  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay.playback.applied_decision_input_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

TEST(ArRfTraceSessionTest, AcceptedExternalDecisionReplaysIntoNextPrepare) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-rf-external-decision");
  {
    const auto writer = MakeWriter(trace_dir);
    ArTraceSessionOptions options;
    options.replay_writer = writer;
    config::ArSessionConfig config;
    config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
    ArTraceSession traced(config, options);

    const ArPrepareCycleResult first = traced.PrepareCycle(MakePrepareInput(5U, 50.0));
    ASSERT_EQ(first.status, ArPrepareCycleStatus::kPrepared);
    const ArCompleteCycleResult completed =
        traced.CompleteCycle(first.token, MakeCompleteInput(first));
    ASSERT_EQ(completed.status, ArCompleteCycleStatus::kCompleted);
    ASSERT_TRUE(completed.has_decision_observation);

    ExternalDecisionResponse response;
    response.source_cycle_index = completed.decision_observation.input_frame.cycle_index;
    response.source_batch_id = completed.decision_observation.input_frame.batch_id;
    response.proposals.push_back(
        TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                          ControlDirectiveSource::SURVIVABILITY),
                         90, "agility"});
    ASSERT_EQ(traced.SubmitExternalDecision(response), ExternalDecisionSubmitStatus::kAccepted);

    const ArPrepareCycleResult second = traced.PrepareCycle(MakePrepareInput(6U, 50.1));
    ASSERT_EQ(second.status, ArPrepareCycleStatus::kPrepared);
    EXPECT_DOUBLE_EQ(second.emission.waveform.center_frequency_hz, 3.1e9);
    EXPECT_EQ(traced.AbandonCycle(second.token), ArAbandonCycleStatus::kAbandoned);
    ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);
  }

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.applied_input_count, 4U);
  EXPECT_EQ(replay.playback.applied_decision_input_count, 1U);
  EXPECT_EQ(replay.playback.compared_output_count, 4U);
}

TEST(ArRfTraceSessionTest, DetectsPrepareStateDivergence) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-rf-divergence");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-rf-v2-divergence";
  manifest.module = "airborne_radar";
  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  const config::ArSessionConfig config;
  WriteEvent(&writer, "session_config", "ArSessionConfig", EncodeSessionConfigFlatbuffer(config));

  const ArPrepareCycleInput input = MakePrepareInput(4U, 40.0);
  WriteEvent(&writer, "cycle_input", "ArPrepareCycleInputV2",
             EncodePrepareCycleInputFlatbuffer(input));
  ArSession original = ArSession::Create(config);
  ArPrepareReplayRecord record;
  record.result = original.PrepareCycle(input);
  record.session_state = ArSessionReplayAccess::CaptureSessionState(original);
  ++record.session_state.next_emission_id;
  WriteEvent(&writer, "cycle_output", "ArPrepareReplayRecordV2",
             EncodePrepareReplayRecordFlatbuffer(record));
  ASSERT_EQ(writer.Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_TRUE(replay.playback.divergence_found);
  EXPECT_NE(replay.first_error.find("ArPrepareReplayRecordV2"), std::string::npos);
}

TEST(ArRfTraceSessionTest, RejectsLegacySingleStageTracePayload) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-rf-legacy-reject");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-legacy";
  manifest.module = "airborne_radar";
  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  WriteEvent(&writer, "session_config", "ArSessionConfig",
             EncodeSessionConfigFlatbuffer(config::ArSessionConfig{}));
  WriteEvent(&writer, "cycle_input", "ArCycleInput", EncodeCycleInputFlatbuffer(ArCycleInput{}));
  ASSERT_EQ(writer.Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.first_error.find("legacy"), std::string::npos);
}

TEST(ArRfTraceSessionTest, RejectsTrailingV2InputWithoutOutput) {
  const std::string trace_dir = MakeTraceDir("oneq-ar-rf-trailing-input");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "ar-rf-v2-trailing-input";
  manifest.module = "airborne_radar";
  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  WriteEvent(&writer, "session_config", "ArSessionConfig",
             EncodeSessionConfigFlatbuffer(config::ArSessionConfig{}));
  WriteEvent(&writer, "cycle_input", "ArPrepareCycleInputV2",
             EncodePrepareCycleInputFlatbuffer(MakePrepareInput(7U, 70.0)));
  ASSERT_EQ(writer.Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);

  const ArReplaySessionResult replay = ReplayArTrace(trace_dir);
  EXPECT_FALSE(replay.ok);
  EXPECT_NE(replay.first_error.find("pending cycle_input"), std::string::npos);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
