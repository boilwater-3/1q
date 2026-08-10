/**
 * @file eos_replay_session_test.cpp
 * @brief 验证 EOS ReplaySession 能够回放 trace 并比对输出。
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosReplaySession.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/replay/ReplayTrace.h"
#include "electro_optical_sensor/session/EosReplayFlatbufferCodec.h"

namespace {

std::string MakeTempTracePath(const char* prefix) {
  static unsigned int unique_counter = 0U;
  const char* temp_dir = std::getenv("TMPDIR");
  if (temp_dir == nullptr || temp_dir[0] == '\0') {
    temp_dir = "/tmp";
  }
  std::ostringstream stream;
  stream << temp_dir;
  const std::string path = stream.str();
  if (!path.empty() && path[path.size() - 1] != '/') {
    stream << "/";
  }
  const long long ticks =
      static_cast<long long>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  stream << prefix << "-" << std::time(nullptr) << "-" << ticks << "-" << std::rand() << "-"
         << unique_counter++ << ".trace";
  return stream.str();
}

}  // namespace

namespace electro_optical_sensor {
namespace session {
namespace tests {

TEST(EosReplaySessionTest, ReplayEosTraceRoundtrip) {
  const std::string trace_dir = MakeTempTracePath("oneq-eos-replay-roundtrip");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-replay-roundtrip-test";
  manifest.module = "electro_optical_sensor";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

    config::EosSessionConfig config;
    config.policy.detection.minimum_snr_db = 4.5f;
    config.policy.detection.detection_sensitivity_w = 0.8e-12f;
    config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;

    EosTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    EosTraceSession session(config, options);

    EosCycleInput input;
    input.cycle_index = 1U;
    input.dt_sec = 0.1f;  // 合法步长：受 53c56e21 收紧的 dt_sec <= 10/frame_rate_hz 上界约束（30Hz → ≈0.333s）。

    const EosCycleResult result = session.StepWithResult(input);
    EXPECT_GE(result.output_frame.detections.size(), 0U);
    replay_writer->Flush();
  }

  const EosReplaySessionResult replay_result = ReplayEosTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
  EXPECT_FALSE(replay_result.reached_failure_marker);
}

TEST(EosReplaySessionTest, ReplayInitialPoweredOffTraceRoundtrip) {
  const std::string trace_dir = MakeTempTracePath("oneq-eos-powered-off-replay");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-powered-off-replay-test";
  manifest.module = "electro_optical_sensor";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    config::EosSessionConfig config;
    config.sensor_enabled = false;
    EosTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    EosTraceSession session(config, options);

    EosCycleInput input;
    input.cycle_index = 1U;
    input.dt_sec = 0.1f;  // 合法步长：受 53c56e21 收紧的 dt_sec <= 10/frame_rate_hz 上界约束（30Hz → ≈0.333s）。
    const EosCycleResult result = session.StepWithResult(input);
    EXPECT_EQ(result.status, EosCycleStatus::kPoweredOff);
    EXPECT_EQ(result.abort_reason, EosPipelineAbortReason::kSensorPoweredOff);
    replay_writer->Flush();
  }

  const EosReplaySessionResult replay_result = ReplayEosTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(EosReplaySessionTest, ReplayEosTraceDetectsDivergence) {
  const std::string trace_dir = MakeTempTracePath("oneq-eos-replay-divergence");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-replay-divergence-test";
  manifest.module = "electro_optical_sensor";
  manifest.scenario_id = "unit-test";

  // Tamper: record a real session_config + cycle_input, then write a cycle_output
  // whose expected EosCycleResult carries a sentinel scan_azimuth_deg that the
  // replayed cycle cannot produce. The replay re-executes the cycle from the
  // recorded config/input, compares the recomputed result against the recorded
  // cycle_output, and must flag the mismatch as output divergence.
  {
    config::EosSessionConfig config;
    EosCycleInput input;
    input.cycle_index = 1U;
    input.dt_sec = 0.1f;  // 合法步长：受 53c56e21 收紧的 dt_sec <= 10/frame_rate_hz 上界约束（30Hz → ≈0.333s）。

    EosCycleResult tampered_result;
    tampered_result.input_cycle_index = input.cycle_index;
    // Sentinel value the real pipeline cannot produce; EosOutputFrameEqual
    // compares scan_azimuth_deg, so this guarantees a divergence.
    tampered_result.output_frame.scan_azimuth_deg = 777.7f;

    oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);

    oneq::replay::ReplayTraceEvent config_event;
    config_event.module = "electro_optical_sensor";
    config_event.event_type = "session_config";
    config_event.payload_type = "EosSessionConfig";
    config_event.payload_encoding = "flatbuffers";
    config_event.payload_bytes = EncodeEosSessionConfig(config);
    writer.WriteEvent(config_event);

    oneq::replay::ReplayTraceEvent input_event;
    input_event.module = "electro_optical_sensor";
    input_event.event_type = "cycle_input";
    input_event.payload_type = "EosCycleInput";
    input_event.payload_encoding = "flatbuffers";
    input_event.payload_bytes = EncodeEosCycleInput(input);
    input_event.has_cycle_index = true;
    input_event.cycle_index = input.cycle_index;
    writer.WriteEvent(input_event);

    oneq::replay::ReplayTraceEvent output_event;
    output_event.module = "electro_optical_sensor";
    output_event.event_type = "cycle_output";
    output_event.payload_type = "EosCycleResult";
    output_event.payload_encoding = "flatbuffers";
    output_event.payload_bytes = EncodeEosCycleResult(tampered_result);
    output_event.has_cycle_index = true;
    output_event.cycle_index = input.cycle_index;
    writer.WriteEvent(output_event);

    writer.Flush();
  }

  const EosReplaySessionResult replay_result = ReplayEosTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_NE(replay_result.first_error.find("divergence"), std::string::npos)
      << replay_result.first_error;
  EXPECT_TRUE(replay_result.playback.divergence_found);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.reached_failure_marker);
}

TEST(EosReplaySessionTest, ReplayEosTraceRejectsWrongModule) {
  const std::string trace_dir = MakeTempTracePath("oneq-eos-replay-wrong-module");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-replay-wrong-module-test";
  manifest.module = "wrong_module";
  manifest.scenario_id = "unit-test";

  {
    oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
    writer.Flush();
  }

  const EosReplaySessionResult replay_result = ReplayEosTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
}

TEST(EosReplaySessionTest, ReplayEosTraceContinuesAfterFailureMarker) {
  const std::string trace_dir = MakeTempTracePath("oneq-eos-replay-failure");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-replay-failure-test";
  manifest.module = "electro_optical_sensor";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

    config::EosSessionConfig config;
    EosTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    EosTraceSession session(config, options);

    EosCycleInput input;
    input.cycle_index = 1U;
    input.dt_sec = 0.1f;  // 合法步长：受 53c56e21 收紧的 dt_sec <= 10/frame_rate_hz 上界约束（30Hz → ≈0.333s）。
    session.StepWithResult(input);

    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "EOS_SIM_ASSERT";
    failure.message = "synthetic replay failure marker";
    failure.has_cycle_index = true;
    failure.cycle_index = 1U;
    const std::string failure_bytes = EncodeEosFailureMarker(failure);
    replay_writer->WriteFailureMarker(failure, failure_bytes);
    input.cycle_index = 2U;
    session.StepWithResult(input);
    replay_writer->Flush();
  }

  const EosReplaySessionResult replay_result = ReplayEosTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.has_failure_marker);
  EXPECT_TRUE(replay_result.reached_failure_marker);
  EXPECT_EQ(replay_result.playback.failure_marker_count, 1U);
  EXPECT_EQ(replay_result.playback.applied_input_count, 2U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 2U);
  EXPECT_EQ(replay_result.failure_marker_data.error_code, "EOS_SIM_ASSERT");
  EXPECT_EQ(replay_result.failure_marker_data.message, "synthetic replay failure marker");
  EXPECT_TRUE(replay_result.failure_marker_data.has_cycle_index);
}

TEST(EosReplaySessionTest, ReplayEosTraceRejectsTrailingCycleInput) {
  const std::string trace_dir = MakeTempTracePath("oneq-eos-replay-trailing-input");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-replay-trailing-input-test";
  manifest.module = "electro_optical_sensor";
  manifest.scenario_id = "unit-test";

  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);

  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "electro_optical_sensor";
  config_event.event_type = "session_config";
  config_event.payload_type = "EosSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes = EncodeEosSessionConfig(config::EosSessionConfig());
  writer.WriteEvent(config_event);

  EosCycleInput input;
  input.cycle_index = 3U;
  input.dt_sec = 0.1f;  // 合法步长：受 53c56e21 收紧的 dt_sec <= 10/frame_rate_hz 上界约束（30Hz → ≈0.333s）。
  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "electro_optical_sensor";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "EosCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = EncodeEosCycleInput(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);
  writer.Flush();

  const EosReplaySessionResult replay_result = ReplayEosTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_NE(replay_result.first_error.find("pending cycle_input"), std::string::npos);
}

}  // namespace tests
}  // namespace session
}  // namespace electro_optical_sensor
