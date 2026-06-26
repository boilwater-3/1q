/**
 * @file trace_session_adapter_test.cpp
 * @brief 验证三模块 TraceSession 中间层能够落盘记录 config/input/output。
 */

#include <flatbuffers/flexbuffers.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarReplaySession.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"
#include "electro_optical_sensor/session/EosReplayFlatbufferCodec.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"

namespace {

std::string MakeTempTracePath(const char* prefix) {
  static unsigned int unique_counter = 0U;
  const char* temp_dir = nullptr;
#if defined(_WIN32)
  temp_dir = std::getenv("TEMP");
  if (temp_dir == nullptr || temp_dir[0] == '\0') {
    temp_dir = std::getenv("TMP");
  }
#else
  temp_dir = std::getenv("TMPDIR");
#endif
  if (temp_dir == nullptr || temp_dir[0] == '\0') {
#if defined(_WIN32)
    temp_dir = ".";
#else
    temp_dir = "/tmp";
#endif
  }

  std::ostringstream stream;
  stream << temp_dir;
  const std::string path = stream.str();
  if (!path.empty() && path[path.size() - 1] != '/' && path[path.size() - 1] != '\\') {
    stream << "/";
  }
  const long long ticks =
      static_cast<long long>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  stream << prefix << "-" << std::time(nullptr) << "-" << ticks << "-" << std::rand() << "-"
         << unique_counter++ << ".trace";
  return stream.str();
}

std::string ReadFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

static std::vector<std::uint8_t> ReadBinaryFile(const std::string& path) {
  std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
  return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(input)),
                                   std::istreambuf_iterator<char>());
}

static void ExpectFlatbufferRecord(const std::vector<std::uint8_t>& content,
                                                    const std::string& module_name,
                                                    const std::string& phase_name) {
  ASSERT_GE(content.size(), 4U);

  const std::uint32_t payload_size = static_cast<std::uint32_t>(content[0]) |
                                     (static_cast<std::uint32_t>(content[1]) << 8U) |
                                     (static_cast<std::uint32_t>(content[2]) << 16U) |
                                     (static_cast<std::uint32_t>(content[3]) << 24U);
  ASSERT_EQ(content.size(), static_cast<std::size_t>(payload_size) + 4U);

  const flexbuffers::Reference root = flexbuffers::GetRoot(content.data() + 4U, payload_size);
  const flexbuffers::Map map = root.AsMap();

  EXPECT_EQ(map["module"].AsString().str(), module_name);
  EXPECT_EQ(map["phase"].AsString().str(), phase_name);
  EXPECT_GT(map["timestamp_ms"].AsInt64(), 0);
  EXPECT_FALSE(map["payload_json"].AsString().str().empty());
}

}  // namespace

namespace airborne_radar {
namespace tests {

TEST(TraceSessionAdapterTest, RadarTraceSessionWritesReplayEventsWithFullInput) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-replay-trace");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-replay-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  config::RadarSessionConfig config = config::RadarSessionConfigBuilder().Build();
  session::RadarTraceSessionOptions options;
  options.replay_writer = replay_writer;
  options.trace_config_on_construct = true;

  session::RadarTraceSession session(config, options);
  session::RadarCycleInput input;
  input.cycle_index = 11U;
  input.dt_sec = 1.0f;

  session::RadarSceneTarget target;
  target.external_target_id = 2001U;
  target.velocity_x = 120.0f;
  target.velocity_y = 0.0f;
  target.velocity_z = 0.0f;
  target.rcs = 1.5f;
  target.range_m = 1500.0f;
  target.position_x = 1500.0f;
  target.position_y = 50.0f;
  target.position_z = 100.0f;
  input.scene.push_back(target);

  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.track_output_frame.tracks.size(), 0U);

  const std::string content = ReadFile(trace_dir + "/events/000000.events.jsonl");
  EXPECT_NE(content.find("\"event_type\":\"session_config\""), std::string::npos);
  EXPECT_NE(content.find("\"event_type\":\"cycle_input\""), std::string::npos);
  EXPECT_NE(content.find("\"event_type\":\"cycle_output\""), std::string::npos);
  EXPECT_NE(content.find("\"payload_type\":\"RadarCycleInput\""), std::string::npos);
  EXPECT_NE(content.find("\"payload_encoding\":\"flatbuffers\""), std::string::npos);
  EXPECT_NE(content.find("\"payload_base64\":\""), std::string::npos);

  oneq::replay::ReplayTraceReader reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent event;
  bool saw_flatbuffer_session_config = false;
  bool saw_flatbuffer_input = false;
  bool saw_flatbuffer_output = false;
  while (reader.ReadNextEvent(&event)) {
    if (event.event_type == "session_config") {
      saw_flatbuffer_session_config = true;
      EXPECT_EQ(event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(event.payload_bytes.empty());
      EXPECT_TRUE(event.payload_hash_matches);
    }
    if (event.event_type == "cycle_input") {
      saw_flatbuffer_input = true;
      EXPECT_EQ(event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(event.payload_bytes.empty());
      EXPECT_TRUE(event.payload_hash_matches);
      EXPECT_TRUE(event.has_cycle_index);
      EXPECT_EQ(event.cycle_index, 11U);
    }
    if (event.event_type == "cycle_output") {
      saw_flatbuffer_output = true;
      EXPECT_EQ(event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(event.payload_bytes.empty());
      EXPECT_TRUE(event.payload_hash_matches);
      session::RadarCycleResult decoded_result;
      std::string decode_error;
      EXPECT_TRUE(
          session::DecodeCycleResultFlatbuffer(event.payload_bytes, &decoded_result, &decode_error))
          << decode_error;
      EXPECT_EQ(decoded_result.input_cycle_index, result.input_cycle_index);
      EXPECT_EQ(decoded_result.track_output_frame.cycle_index,
                result.track_output_frame.cycle_index);
      EXPECT_EQ(decoded_result.track_output_frame.tracks.size(),
                result.track_output_frame.tracks.size());
    }
  }
  EXPECT_TRUE(saw_flatbuffer_session_config);
  EXPECT_TRUE(saw_flatbuffer_input);
  EXPECT_TRUE(saw_flatbuffer_output);
}

TEST(TraceSessionAdapterTest, RadarReplaySessionReplaysTraceAndComparesOutput) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-replay-run");
  const float expected_kalman_noise_std = 7.5f;
  const float expected_scan_center_az_deg = 4.0f;
  const float expected_scan_center_el_deg = -1.0f;

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-replay-run-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

    session::RadarTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;

    config::RadarSessionConfig config;
    config.policy.lifecycle.confirm_hits = 1U;
    config.policy.lifecycle.max_miss_before_lost = 1U;
    config.policy.tracking.enable_kalman_filter = false;
    config.mission.orientation.scan_center_deg.az_deg = 12.5f;
    config.hardware.min_detection_margin_db = -25.0f;
    session::RadarTraceSession session(config, options);

    config::RadarRuntimeConfigPatch runtime_patch;
    runtime_patch.has_policy = true;
    runtime_patch.policy = config.policy;
    runtime_patch.policy.tracking.kalman_measurement_noise_std = expected_kalman_noise_std;
    runtime_patch.has_scan_center_deg = true;
    runtime_patch.scan_center_deg.az_deg = expected_scan_center_az_deg;
    runtime_patch.scan_center_deg.el_deg = expected_scan_center_el_deg;
    runtime_patch.has_commanded_beamwidth_enabled = true;
    runtime_patch.commanded_beamwidth_enabled = true;
    runtime_patch.has_environment = true;
    runtime_patch.environment.has_jamming_sensitivity_profile = true;
    runtime_patch.environment.jamming_sensitivity_profile =
        config::JammingSensitivityProfile::kStrict;
    runtime_patch.environment.has_scenario_config = true;
    runtime_patch.environment.scenario_config.atmospheric_physics
        .enable_physical_model = true;
    runtime_patch.environment.scenario_config.atmospheric_physics.relative_humidity =
        0.4f;
    session.ApplyRuntimeConfig(runtime_patch);

    session::RadarCycleInput input;
    input.dt_sec = 1.0f;

    session::RadarSceneTarget target;
    target.external_target_id = 2002U;
    target.velocity_x = 80.0f;
    target.velocity_y = 1.0f;
    target.velocity_z = 0.0f;
    target.rcs = 2.0f;
    target.range_m = 2000.0f;
    target.position_x = 2000.0f;
    target.position_y = 0.0f;
    target.position_z = 150.0f;
    input.scene.push_back(target);

    input.environment.atmospheric_observation.enable_physical_model = true;
    input.environment.atmospheric_observation.relative_humidity = 0.65f;
    config::JammerEmitterState jammer;
    jammer.technique = session::JammingTechnique::kNoiseSuppression;
    jammer.power_db = 24.0f;
    jammer.js_db = 7.0f;
    jammer.has_direction_deg = true;
    jammer.azimuth_deg = 18.0f;
    jammer.elevation_deg = 2.0f;
    input.environment.jammer_sources.push_back(jammer);

    const session::RadarCycleResult result = session.StepWithResult(input);
    EXPECT_GE(result.track_output_frame.tracks.size(), 0U);
    replay_writer->Flush();
  }

  const std::string content = ReadFile(trace_dir + "/events/000000.events.jsonl");
  EXPECT_NE(content.find("\"event_type\":\"runtime_config_patch\""), std::string::npos);
  EXPECT_NE(content.find("\"payload_type\":\"RadarRuntimeConfigPatch\""), std::string::npos);
  EXPECT_EQ(content.find("\"event_type\":\"scene_state\""), std::string::npos);
  EXPECT_NE(content.find("\"payload_encoding\":\"flatbuffers\""), std::string::npos);
  EXPECT_NE(content.find("\"payload_base64\":\""), std::string::npos);

  oneq::replay::ReplayTraceReader replay_reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent replay_event;
  bool saw_session_config = false;
  bool saw_runtime_patch = false;
  bool saw_cycle_output = false;
  while (replay_reader.ReadNextEvent(&replay_event)) {
    if (replay_event.event_type == "session_config") {
      saw_session_config = true;
      EXPECT_EQ(replay_event.payload_type, "RadarSessionConfig");
      EXPECT_EQ(replay_event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(replay_event.payload_bytes.empty());
      EXPECT_TRUE(replay_event.payload_hash_matches);
    }
    if (replay_event.event_type == "runtime_config_patch") {
      saw_runtime_patch = true;
      EXPECT_EQ(replay_event.payload_type, "RadarRuntimeConfigPatch");
      EXPECT_EQ(replay_event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(replay_event.payload_bytes.empty());
      EXPECT_TRUE(replay_event.payload_hash_matches);
      config::RadarRuntimeConfigPatch decoded_patch;
      std::string decode_error;
      EXPECT_TRUE(session::DecodeRuntimeConfigPatchFlatbuffer(replay_event.payload_bytes,
                                                              &decoded_patch, &decode_error))
          << decode_error;
      EXPECT_TRUE(decoded_patch.has_policy);
      EXPECT_FLOAT_EQ(decoded_patch.policy.tracking.kalman_measurement_noise_std,
                      expected_kalman_noise_std);
      EXPECT_TRUE(decoded_patch.has_scan_center_deg);
      EXPECT_FLOAT_EQ(decoded_patch.scan_center_deg.az_deg, expected_scan_center_az_deg);
      EXPECT_FLOAT_EQ(decoded_patch.scan_center_deg.el_deg, expected_scan_center_el_deg);
      EXPECT_TRUE(decoded_patch.has_environment);
      EXPECT_TRUE(decoded_patch.environment.has_jamming_sensitivity_profile);
    }
    if (replay_event.event_type == "cycle_output") {
      saw_cycle_output = true;
      EXPECT_EQ(replay_event.payload_type, "RadarCycleResult");
      EXPECT_EQ(replay_event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(replay_event.payload_bytes.empty());
      EXPECT_TRUE(replay_event.payload_hash_matches);
      session::RadarCycleResult decoded_result;
      std::string decode_error;
      EXPECT_TRUE(session::DecodeCycleResultFlatbuffer(replay_event.payload_bytes, &decoded_result,
                                                       &decode_error))
          << decode_error;
      EXPECT_TRUE(decoded_result.executed_this_cycle);
      EXPECT_TRUE(replay_event.has_cycle_index);
      EXPECT_EQ(decoded_result.input_cycle_index, replay_event.cycle_index);
    }
  }
  EXPECT_TRUE(saw_session_config);
  EXPECT_TRUE(saw_runtime_patch);
  EXPECT_TRUE(saw_cycle_output);

  const session::RadarReplaySessionResult replay_result = session::ReplayRadarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.applied_scene_state_count, 0U);
  EXPECT_EQ(replay_result.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(TraceSessionAdapterTest, RadarTraceSessionUsesInputCycleIndexForValidationFailures) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-validation-failure");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-validation-failure-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  session::RadarTraceSessionOptions options;
  options.replay_writer = replay_writer;
  options.trace_config_on_construct = true;

  config::RadarSessionConfig config;
  session::RadarTraceSession session(config, options);

  session::RadarCycleInput input;
  input.cycle_index = 77U;
  input.dt_sec = -1.0f;
  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_TRUE(result.has_validation_error);
  EXPECT_EQ(result.input_cycle_index, 77U);
  replay_writer->Flush();

  oneq::replay::ReplayTraceReader replay_reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent replay_event;
  bool saw_cycle_output = false;
  bool saw_cycle_input = false;
  bool saw_failure_marker = false;
  while (replay_reader.ReadNextEvent(&replay_event)) {
    if (replay_event.event_type == "cycle_input") {
      saw_cycle_input = true;
      EXPECT_TRUE(replay_event.has_cycle_index);
      EXPECT_EQ(replay_event.cycle_index, 77U);
    }
    if (replay_event.event_type == "cycle_output") {
      saw_cycle_output = true;
      EXPECT_TRUE(replay_event.has_cycle_index);
      EXPECT_EQ(replay_event.cycle_index, 77U);
      session::RadarCycleResult decoded_result;
      std::string decode_error;
      EXPECT_TRUE(session::DecodeCycleResultFlatbuffer(replay_event.payload_bytes, &decoded_result,
                                                       &decode_error))
          << decode_error;
      EXPECT_EQ(decoded_result.input_cycle_index, 77U);
    }
    if (replay_event.event_type == "failure_marker") {
      saw_failure_marker = true;
      oneq::replay::ReplayTraceFailure failure;
      std::string decode_error;
      EXPECT_TRUE(session::DecodeFailureMarkerFlatbuffer(replay_event.payload_bytes, &failure,
                                                         &decode_error))
          << decode_error;
      EXPECT_TRUE(failure.has_cycle_index);
      EXPECT_EQ(failure.cycle_index, 77U);
    }
  }
  EXPECT_TRUE(saw_cycle_input);
  EXPECT_TRUE(saw_cycle_output);
  EXPECT_TRUE(saw_failure_marker);
}

TEST(TraceSessionAdapterTest, RadarReplaySessionStopsAtFailureMarker) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-replay-failure");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-replay-failure-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

    session::RadarTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;

    config::RadarSessionConfig config;
    session::RadarTraceSession session(config, options);

    session::RadarCycleInput input;
    input.dt_sec = 1.0f;
    const session::RadarCycleResult result = session.StepWithResult(input);
    EXPECT_GE(result.track_output_frame.tracks.size(), 0U);

    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "AR_SIM_ASSERT";
    failure.message = "synthetic replay failure marker";
    failure.has_cycle_index = true;
    failure.cycle_index = result.track_output_frame.cycle_index;
    const std::string failure_bytes = session::EncodeFailureMarkerFlatbuffer(failure, false, 0U);
    replay_writer->WriteFailureMarker(failure, failure_bytes);
    replay_writer->Flush();
  }

  const session::RadarReplaySessionResult replay_result = session::ReplayRadarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.has_failure_marker);
  EXPECT_TRUE(replay_result.reached_failure_marker);
  EXPECT_EQ(replay_result.playback.failure_marker_count, 1U);
  EXPECT_EQ(replay_result.failure_marker_data.error_code, "AR_SIM_ASSERT");
  EXPECT_EQ(replay_result.failure_marker_data.message, "synthetic replay failure marker");
  EXPECT_TRUE(replay_result.failure_marker_data.has_cycle_index);
}

TEST(TraceSessionAdapterTest, RadarTraceSessionStepWritesResultFailureMarker) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-step-validation-failure");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-step-validation-failure-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  session::RadarTraceSessionOptions options;
  options.replay_writer = replay_writer;
  options.trace_config_on_construct = true;
  session::RadarTraceSession session(config::RadarSessionConfig(), options);

  session::RadarCycleInput input;
  input.cycle_index = 91U;
  input.dt_sec = -1.0f;
  (void)session.Step(input);
  replay_writer->Flush();

  oneq::replay::ReplayTraceReader replay_reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent replay_event;
  bool saw_result_output = false;
  bool saw_failure_marker = false;
  while (replay_reader.ReadNextEvent(&replay_event)) {
    if (replay_event.event_type == "cycle_output") {
      EXPECT_EQ(replay_event.payload_type, "RadarCycleResult");
      session::RadarCycleResult decoded_result;
      std::string decode_error;
      ASSERT_TRUE(session::DecodeCycleResultFlatbuffer(replay_event.payload_bytes, &decoded_result,
                                                       &decode_error))
          << decode_error;
      saw_result_output = true;
      EXPECT_TRUE(decoded_result.has_validation_error);
      EXPECT_EQ(decoded_result.input_cycle_index, 91U);
    }
    if (replay_event.event_type == "failure_marker") {
      saw_failure_marker = true;
    }
  }
  EXPECT_TRUE(saw_result_output);
  EXPECT_TRUE(saw_failure_marker);
}

TEST(TraceSessionAdapterTest, RadarTraceSessionConsecutiveStepWithResultDoesNotEmitWarning) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-consecutive-input");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-consecutive-input-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  session::RadarTraceSessionOptions options;
  options.replay_writer = replay_writer;
  options.trace_config_on_construct = true;
  session::RadarTraceSession session(config::RadarSessionConfig(), options);

  session::RadarCycleInput input1;
  input1.cycle_index = 1U;
  input1.dt_sec = 1.0f;
  const session::RadarCycleResult result1 = session.StepWithResult(input1);
  EXPECT_GE(result1.track_output_frame.tracks.size(), 0U);

  session::RadarCycleInput input2;
  input2.cycle_index = 2U;
  input2.dt_sec = 1.0f;
  const session::RadarCycleResult result2 = session.StepWithResult(input2);
  EXPECT_GE(result2.track_output_frame.tracks.size(), 0U);

  replay_writer->Flush();

  oneq::replay::ReplayTraceReader replay_reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent replay_event;
  std::uint32_t input_count = 0U;
  std::uint32_t output_count = 0U;
  std::uint32_t warning_count = 0U;
  while (replay_reader.ReadNextEvent(&replay_event)) {
    if (replay_event.event_type == "cycle_input") {
      ++input_count;
    }
    if (replay_event.event_type == "cycle_output") {
      ++output_count;
    }
    if (replay_event.event_type == "warning") {
      ++warning_count;
      EXPECT_EQ(replay_event.payload_type, "ConsecutiveCycleInputWarning");
    }
  }
  EXPECT_EQ(input_count, 2U);
  EXPECT_EQ(output_count, 2U);
  EXPECT_EQ(warning_count, 0U);
}

TEST(TraceSessionAdapterTest, RadarReplaySessionRejectsTrailingCycleInput) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-trailing-input");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-trailing-input-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);

  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "airborne_radar";
  config_event.event_type = "session_config";
  config_event.payload_type = "RadarSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes =
      session::EncodeSessionConfigFlatbuffer(config::RadarSessionConfig());
  writer.WriteEvent(config_event);

  session::RadarCycleInput input;
  input.cycle_index = 3U;
  input.dt_sec = 1.0f;
  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "airborne_radar";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "RadarCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = session::EncodeCycleInputFlatbuffer(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);
  writer.Flush();

  const session::RadarReplaySessionResult replay_result = session::ReplayRadarTrace(trace_dir);

  EXPECT_FALSE(replay_result.ok);
  EXPECT_NE(replay_result.first_error.find("pending cycle_input"), std::string::npos);
}

}  // namespace tests
}  // namespace airborne_radar

namespace electronic_surveillance_radar {
namespace tests {

TEST(TraceSessionAdapterTest, EsrTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-esr-trace");
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::FlatbufferFileTraceSink(trace_path, false));

  config::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;

  session::EsrTraceSession session(config, session::EsrTraceSessionOptions{sink, true});

  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const session::EsrCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.output_frame.observation_output.observations.size(), 0U);

  const std::vector<std::uint8_t> content = ReadBinaryFile(trace_path);
  bool saw_config = false;
  bool saw_input = false;
  bool saw_output = false;
  std::size_t offset = 0U;
  while (offset + 4U <= content.size()) {
    const std::uint32_t payload_size = static_cast<std::uint32_t>(content[offset]) |
                                       (static_cast<std::uint32_t>(content[offset + 1]) << 8U) |
                                       (static_cast<std::uint32_t>(content[offset + 2]) << 16U) |
                                       (static_cast<std::uint32_t>(content[offset + 3]) << 24U);
    ASSERT_LE(offset + 4U + payload_size, content.size());
    const flexbuffers::Reference root =
        flexbuffers::GetRoot(content.data() + offset + 4U, payload_size);
    const flexbuffers::Map map = root.AsMap();
    const std::string module = map["module"].AsString().str();
    const std::string phase = map["phase"].AsString().str();
    if (module == "electronic_surveillance_radar") {
      if (phase == "config") saw_config = true;
      if (phase == "input") saw_input = true;
      if (phase == "output") saw_output = true;
    }
    offset += 4U + payload_size;
  }
  EXPECT_TRUE(saw_config);
  EXPECT_TRUE(saw_input);
  EXPECT_TRUE(saw_output);

  std::remove(trace_path.c_str());
}

TEST(TraceSessionAdapterTest, EsrTraceSessionUsesInputCycleIndexForValidationFailures) {
  const std::string trace_dir = MakeTempTracePath("oneq-esr-validation-failure");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "esr-validation-failure-test";
  manifest.module = "electronic_surveillance_radar";
  manifest.scenario_id = "unit-test";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  config::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;

  session::EsrTraceSessionOptions options;
  options.replay_writer = replay_writer;
  options.trace_config_on_construct = true;
  session::EsrTraceSession session(config, options);

  session::EsrCycleInput valid_input;
  valid_input.cycle_index = 1U;
  valid_input.dt_sec = 1.0f;
  const session::EsrCycleResult valid_result = session.StepWithResult(valid_input);
  ASSERT_TRUE(valid_result.executed_this_cycle);

  session::EsrCycleInput invalid_input;
  invalid_input.cycle_index = 77U;
  invalid_input.dt_sec = -1.0f;
  const session::EsrOutputFrame invalid_output = session.Step(invalid_input);
  EXPECT_EQ(invalid_output.cycle_index, valid_result.output_frame.cycle_index);
  replay_writer->Flush();

  oneq::replay::ReplayTraceReader replay_reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent replay_event;
  bool saw_failed_cycle_output = false;
  bool saw_failure_marker = false;
  while (replay_reader.ReadNextEvent(&replay_event)) {
    if (replay_event.event_type == "cycle_output") {
      session::EsrCycleResult decoded_result;
      ASSERT_TRUE(session::DecodeEsrCycleResult(replay_event.payload_bytes, &decoded_result));
      if (decoded_result.has_validation_error) {
        saw_failed_cycle_output = true;
        EXPECT_TRUE(replay_event.has_cycle_index);
        EXPECT_EQ(replay_event.cycle_index, 77U);
        EXPECT_EQ(decoded_result.input_cycle_index, 77U);
      }
    }
    if (replay_event.event_type == "failure_marker") {
      saw_failure_marker = true;
      EXPECT_TRUE(replay_event.has_cycle_index);
      EXPECT_EQ(replay_event.cycle_index, 77U);
    }
  }
  EXPECT_TRUE(saw_failed_cycle_output);
  EXPECT_TRUE(saw_failure_marker);
}

}  // namespace tests
}  // namespace electronic_surveillance_radar

namespace electro_optical_sensor {
namespace tests {

TEST(TraceSessionAdapterTest, EosTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-eos-trace");
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::FlatbufferFileTraceSink(trace_path, false));

  config::EosSessionConfig config;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;

  ::electro_optical_sensor::session::EosTraceSession session(
      config, ::electro_optical_sensor::session::EosTraceSessionOptions{sink, true});

  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.output_frame.detections.size(), 0U);

  const std::vector<std::uint8_t> content = ReadBinaryFile(trace_path);
  bool saw_config = false;
  bool saw_input = false;
  bool saw_output = false;
  std::size_t offset = 0U;
  while (offset + 4U <= content.size()) {
    const std::uint32_t payload_size = static_cast<std::uint32_t>(content[offset]) |
                                       (static_cast<std::uint32_t>(content[offset + 1]) << 8U) |
                                       (static_cast<std::uint32_t>(content[offset + 2]) << 16U) |
                                       (static_cast<std::uint32_t>(content[offset + 3]) << 24U);
    ASSERT_LE(offset + 4U + payload_size, content.size());
    const flexbuffers::Reference root =
        flexbuffers::GetRoot(content.data() + offset + 4U, payload_size);
    const flexbuffers::Map map = root.AsMap();
    const std::string module = map["module"].AsString().str();
    const std::string phase = map["phase"].AsString().str();
    if (module == "electro_optical_sensor") {
      if (phase == "config") saw_config = true;
      if (phase == "input") saw_input = true;
      if (phase == "output") saw_output = true;
    }
    offset += 4U + payload_size;
  }
  EXPECT_TRUE(saw_config);
  EXPECT_TRUE(saw_input);
  EXPECT_TRUE(saw_output);

  std::remove(trace_path.c_str());
}

TEST(TraceSessionAdapterTest, EosTraceSessionUsesInputCycleIndexForValidationFailures) {
  const std::string trace_dir = MakeTempTracePath("oneq-eos-validation-failure");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-validation-failure-test";
  manifest.module = "electro_optical_sensor";
  manifest.scenario_id = "unit-test";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  config::EosSessionConfig config;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;

  session::EosTraceSessionOptions options;
  options.replay_writer = replay_writer;
  options.trace_config_on_construct = true;
  session::EosTraceSession session(config, options);

  session::EosCycleInput valid_input;
  valid_input.cycle_index = 1U;
  valid_input.dt_sec = 1.0f;
  const session::EosCycleResult valid_result = session.StepWithResult(valid_input);
  ASSERT_TRUE(valid_result.executed_this_cycle);

  session::EosCycleInput invalid_input;
  invalid_input.cycle_index = 77U;
  invalid_input.dt_sec = -1.0f;
  const session::EosOutputFrame invalid_output = session.Step(invalid_input);
  EXPECT_EQ(invalid_output.cycle_index, valid_result.output_frame.cycle_index);
  replay_writer->Flush();

  oneq::replay::ReplayTraceReader replay_reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent replay_event;
  bool saw_failed_cycle_output = false;
  bool saw_failure_marker = false;
  while (replay_reader.ReadNextEvent(&replay_event)) {
    if (replay_event.event_type == "cycle_output") {
      session::EosCycleResult decoded_result;
      ASSERT_TRUE(session::DecodeEosCycleResult(replay_event.payload_bytes, &decoded_result));
      if (decoded_result.has_validation_error) {
        saw_failed_cycle_output = true;
        EXPECT_TRUE(replay_event.has_cycle_index);
        EXPECT_EQ(replay_event.cycle_index, 77U);
        EXPECT_EQ(decoded_result.input_cycle_index, 77U);
      }
    }
    if (replay_event.event_type == "failure_marker") {
      saw_failure_marker = true;
      EXPECT_TRUE(replay_event.has_cycle_index);
      EXPECT_EQ(replay_event.cycle_index, 77U);
    }
  }
  EXPECT_TRUE(saw_failed_cycle_output);
  EXPECT_TRUE(saw_failure_marker);
}

}  // namespace tests
}  // namespace electro_optical_sensor

namespace oneq {
namespace trace {
namespace tests {

TEST(TraceSessionAdapterTest, FlatbufferFileTraceSinkWritesBinaryFrame) {
  const std::string trace_path = MakeTempTracePath("oneq-flatbuffer-trace");
  std::shared_ptr<TraceSink> sink(new FlatbufferFileTraceSink(trace_path, false));
  sink->Record("platform_module", "input", "{\"value\":1}");

  const std::vector<std::uint8_t> content = ReadBinaryFile(trace_path);
  ExpectFlatbufferRecord(content, "platform_module", "input");

  std::remove(trace_path.c_str());
}

}  // namespace tests
}  // namespace trace
}  // namespace oneq
