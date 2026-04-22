/**
 * @file trace_session_adapter_test.cpp
 * @brief 验证三模块 TraceSession 中间层能够落盘记录 config/input/output。
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <flatbuffers/flexbuffers.h>
#endif

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarReplaySession.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "1q/electro_optical_sensor/model/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

namespace {

std::string MakeTempTracePath(const char* prefix) {
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
  stream << prefix << "-" << std::time(nullptr) << "-" << std::rand() << ".jsonl";
  return stream.str();
}

std::string ReadFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::vector<std::uint8_t> ReadBinaryFile(const std::string& path) {
  std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
  return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(input)),
                                   std::istreambuf_iterator<char>());
}

void ExpectCommonTracePhases(const std::string& content, const std::string& module_name) {
  EXPECT_NE(content.find("\"module\":\"" + module_name + "\""), std::string::npos);
  EXPECT_NE(content.find("\"phase\":\"config\""), std::string::npos);
  EXPECT_NE(content.find("\"phase\":\"input\""), std::string::npos);
  EXPECT_NE(content.find("\"phase\":\"output\""), std::string::npos);
}

#if defined(_WIN32)
void ExpectFlatbufferRecord(const std::vector<std::uint8_t>& content,
                            const std::string& module_name, const std::string& phase_name) {
  ASSERT_GE(content.size(), 4U);

  const std::uint32_t payload_size = static_cast<std::uint32_t>(content[0]) |
                                     (static_cast<std::uint32_t>(content[1]) << 8U) |
                                     (static_cast<std::uint32_t>(content[2]) << 16U) |
                                     (static_cast<std::uint32_t>(content[3]) << 24U);
  ASSERT_EQ(content.size(), static_cast<std::size_t>(payload_size) + 4U);

  const flexbuffers::Reference root =
      flexbuffers::GetRoot(content.data() + 4U, payload_size);
  const flexbuffers::Map map = root.AsMap();

  EXPECT_EQ(map["module"].AsString().str(), module_name);
  EXPECT_EQ(map["phase"].AsString().str(), phase_name);
  EXPECT_GT(map["timestamp_ms"].AsInt64(), 0);
  EXPECT_FALSE(map["payload_json"].AsString().str().empty());
}
#endif

}  // namespace

namespace airborne_radar {
namespace tests {

TEST(TraceSessionAdapterTest, RadarTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-radar-trace");
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::JsonlFileTraceSink(trace_path, false));

  session::RadarSessionConfig config = config::presets::MakeDefaultRadarSessionConfig();

  session::RadarTraceSession session(config, session::RadarTraceSessionOptions{sink, true});
  session::RadarCycleInput input;
  input.dt_sec = 1.0f;

  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.track_output_frame.published_track_count, 0U);

  const std::string content = ReadFile(trace_path);
  ExpectCommonTracePhases(content, "airborne_radar");

  std::remove(trace_path.c_str());
}

TEST(TraceSessionAdapterTest, RadarTraceSessionWritesReplayEventsWithFullInput) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-replay-trace");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "radar-replay-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "unit-test";

  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  session::RadarSessionConfig config = config::presets::MakeDefaultRadarSessionConfig();
  session::RadarTraceSessionOptions options;
  options.replay_writer = replay_writer;
  options.trace_config_on_construct = true;

  session::RadarTraceSession session(config, options);
  session::RadarCycleInput input;
  input.dt_sec = 1.0f;

  model::TargetFeature target;
  target.external_target_id = 2001U;
  target.current_track_velocity_x = 120.0f;
  target.current_track_velocity_y = 0.0f;
  target.current_track_velocity_z = 0.0f;
  target.current_track_speed = 120.0f;
  target.current_track_rcs = 1.5f;
  target.range_m = 1500.0f;
  target.has_cartesian_position = true;
  target.position_x = 1500.0f;
  target.position_y = 50.0f;
  target.position_z = 100.0f;
  input.target_features.push_back(target);

  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.track_output_frame.published_track_count, 0U);

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
    }
  }
  EXPECT_TRUE(saw_flatbuffer_session_config);
  EXPECT_TRUE(saw_flatbuffer_input);
}

TEST(TraceSessionAdapterTest, RadarReplaySessionReplaysTraceAndComparesOutput) {
  const std::string trace_dir = MakeTempTracePath("oneq-radar-replay-run");

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

    session::RadarSessionConfig config;
    config.policy.lifecycle.confirm_hits = 1U;
    config.policy.lifecycle.max_miss_before_lost = 1U;
    config.policy.tracking.enable_kalman_filter = false;
    config.mission.orientation.scan_center_deg.az_deg = 12.5f;
    config.hardware.detection.min_detection_margin_db = -25.0f;
    session::RadarTraceSession session(config, options);

    config::RadarRuntimeConfigPatch runtime_patch;
    runtime_patch.has_policy = true;
    runtime_patch.policy = config.policy;
    runtime_patch.policy.tracking.kalman_measurement_noise_std = 7.5f;
    runtime_patch.has_scan_center_deg = true;
    runtime_patch.scan_center_deg.az_deg = 4.0f;
    runtime_patch.scan_center_deg.el_deg = -1.0f;
    runtime_patch.has_commanded_beamwidth_enabled = true;
    runtime_patch.commanded_beamwidth_enabled = true;
    runtime_patch.has_environment_runtime_config = true;
    runtime_patch.environment_runtime_config.has_jamming_sensitivity_profile = true;
    runtime_patch.environment_runtime_config.jamming_sensitivity_profile =
        environment::JammingSensitivityProfile::kStrict;
    runtime_patch.environment_runtime_config.has_scenario_config = true;
    runtime_patch.environment_runtime_config.scenario_config.atmospheric_physics
        .enable_physical_model = true;
    runtime_patch.environment_runtime_config.scenario_config.atmospheric_physics
        .relative_humidity = 0.4f;
    session.ApplyRuntimeConfig(runtime_patch);

    session::RadarCycleInput input;
    input.dt_sec = 1.0f;

    model::TargetFeature target;
    target.external_target_id = 2002U;
    target.current_track_velocity_x = 80.0f;
    target.current_track_velocity_y = 1.0f;
    target.current_track_velocity_z = 0.0f;
    target.current_track_speed = 80.006f;
    target.current_track_rcs = 2.0f;
    target.range_m = 2000.0f;
    target.has_cartesian_position = true;
    target.position_x = 2000.0f;
    target.position_y = 0.0f;
    target.position_z = 150.0f;
    input.target_features.push_back(target);

    environment::EnvironmentSceneState scene_state;
    scene_state.atmospheric_physics.enable_physical_model = true;
    scene_state.atmospheric_physics.relative_humidity = 0.65f;
    environment::JammerEmitterState jammer;
    jammer.technique = model::JammingTechnique::kNoiseSuppression;
    jammer.power_db = 24.0f;
    jammer.js_db = 7.0f;
    jammer.has_direction_deg = true;
    jammer.azimuth_deg = 18.0f;
    jammer.elevation_deg = 2.0f;
    scene_state.jammer_emitters.push_back(jammer);

    const session::RadarCycleResult result = session.StepWithResult(input, scene_state);
    EXPECT_GE(result.track_output_frame.published_track_count, 0U);
    replay_writer->Flush();
  }

  const std::string content = ReadFile(trace_dir + "/events/000000.events.jsonl");
  EXPECT_NE(content.find("\"event_type\":\"runtime_config_patch\""), std::string::npos);
  EXPECT_NE(content.find("\"has_policy\":true"), std::string::npos);
  EXPECT_NE(content.find("\"kalman_measurement_noise_std\":7.5"), std::string::npos);
  EXPECT_NE(content.find("\"has_environment_runtime_config\":true"),
            std::string::npos);
  EXPECT_NE(content.find("\"has_jamming_sensitivity_profile\":true"),
            std::string::npos);
  EXPECT_NE(content.find("\"has_scan_center_deg\":true"), std::string::npos);
  EXPECT_NE(content.find("\"event_type\":\"scene_state\""), std::string::npos);
  EXPECT_NE(content.find("\"payload_encoding\":\"flatbuffers\""), std::string::npos);

  oneq::replay::ReplayTraceReader replay_reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent replay_event;
  bool saw_session_config = false;
  bool saw_scene_state = false;
  while (replay_reader.ReadNextEvent(&replay_event)) {
    if (replay_event.event_type == "session_config") {
      saw_session_config = true;
      EXPECT_EQ(replay_event.payload_type, "RadarSessionConfig");
      EXPECT_EQ(replay_event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(replay_event.payload_bytes.empty());
      EXPECT_TRUE(replay_event.payload_hash_matches);
    }
    if (replay_event.event_type == "scene_state") {
      saw_scene_state = true;
      EXPECT_EQ(replay_event.payload_type, "EnvironmentSceneState");
      EXPECT_EQ(replay_event.payload_encoding, "flatbuffers");
      EXPECT_FALSE(replay_event.payload_bytes.empty());
      EXPECT_TRUE(replay_event.payload_hash_matches);
    }
  }
  EXPECT_TRUE(saw_session_config);
  EXPECT_TRUE(saw_scene_state);

  const session::RadarReplaySessionResult replay_result =
      session::ReplayRadarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.applied_scene_state_count, 1U);
  EXPECT_EQ(replay_result.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
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

    session::RadarSessionConfig config;
    session::RadarTraceSession session(config, options);

    session::RadarCycleInput input;
    input.dt_sec = 1.0f;
    const session::RadarCycleResult result = session.StepWithResult(input);
    EXPECT_GE(result.track_output_frame.published_track_count, 0U);

    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "AR_SIM_ASSERT";
    failure.message = "synthetic replay failure marker";
    failure.has_cycle_index = true;
    failure.cycle_index = result.track_output_frame.cycle_index;
    replay_writer->WriteFailureMarker(failure);
    replay_writer->Flush();
  }

  const session::RadarReplaySessionResult replay_result =
      session::ReplayRadarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.has_failure_marker);
  EXPECT_TRUE(replay_result.reached_failure_marker);
  EXPECT_EQ(replay_result.playback.failure_marker_count, 1U);
  EXPECT_NE(replay_result.failure_marker_payload_json.find("AR_SIM_ASSERT"),
            std::string::npos);
}

}  // namespace tests
}  // namespace airborne_radar

namespace electronic_surveillance_radar {
namespace tests {

TEST(TraceSessionAdapterTest, EsrTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-esr-trace");
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::JsonlFileTraceSink(trace_path, false));

  session::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;

  session::EsrTraceSession session(config, session::EsrTraceSessionOptions{sink, true});

  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const session::EsrCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.output_frame.observation_output.observations.size(), 0U);

  const std::string content = ReadFile(trace_path);
  ExpectCommonTracePhases(content, "electronic_surveillance_radar");

  std::remove(trace_path.c_str());
}

}  // namespace tests
}  // namespace electronic_surveillance_radar

namespace electro_optical_sensor {
namespace tests {

TEST(TraceSessionAdapterTest, EosTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-eos-trace");
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::JsonlFileTraceSink(trace_path, false));

  session::EosSessionConfig config;
  config.policy.detection.profile = ::electro_optical_sensor::config::EosDetectionProfile::kAggressive;

  ::electro_optical_sensor::session::EosTraceSession session(
      config, ::electro_optical_sensor::session::EosTraceSessionOptions{sink, true});

  session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.output_frame.detections.size(), 0U);

  const std::string content = ReadFile(trace_path);
  ExpectCommonTracePhases(content, "electro_optical_sensor");

  std::remove(trace_path.c_str());
}

}  // namespace tests
}  // namespace electro_optical_sensor

namespace oneq {
namespace trace {
namespace tests {

TEST(TraceSessionAdapterTest, PlatformFileTraceSinkUsesPlatformBackend) {
  const std::string trace_path = MakeTempTracePath("oneq-platform-trace");
  std::shared_ptr<TraceSink> sink(new PlatformFileTraceSink(trace_path, false));
  sink->Record("platform_module", "input", "{\"value\":1}");

#if defined(_WIN32)
  const std::vector<std::uint8_t> content = ReadBinaryFile(trace_path);
  ExpectFlatbufferRecord(content, "platform_module", "input");
#else
  const std::string content = ReadFile(trace_path);
  EXPECT_NE(content.find("\"module\":\"platform_module\""), std::string::npos);
  EXPECT_NE(content.find("\"phase\":\"input\""), std::string::npos);
  EXPECT_NE(content.find("\"payload\":{\"value\":1}"), std::string::npos);
#endif

  std::remove(trace_path.c_str());
}

}  // namespace tests
}  // namespace trace
}  // namespace oneq
