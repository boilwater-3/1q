#include <gtest/gtest.h>

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include "1q/replay/ReplayTrace.h"

namespace {

std::string MakeTempTraceDir() {
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
  const std::string base = stream.str();
  if (!base.empty() && base[base.size() - 1U] != '/' && base[base.size() - 1U] != '\\') {
    stream << "/";
  }
  stream << "oneq-replay-trace-" << std::time(nullptr) << "-" << std::rand();
  return stream.str();
}

std::string JoinPath(const std::string& left, const std::string& right) {
  if (left.empty()) {
    return right;
  }
  const char tail = left[left.size() - 1U];
  if (tail == '/' || tail == '\\') {
    return left + right;
  }
#if defined(_WIN32)
  return left + "\\" + right;
#else
  return left + "/" + right;
#endif
}

std::string ReadFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void WriteFile(const std::string& path, const std::string& content) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  output << content;
}

std::string ReplaceFirst(std::string content, const std::string& from,
                         const std::string& to) {
  const std::size_t pos = content.find(from);
  if (pos != std::string::npos) {
    content.replace(pos, from.size(), to);
  }
  return content;
}

std::string MakeFlatbuffersPayloadBytes(const std::string& tag) {
  return std::string("fb:") + tag;
}

struct PlaybackDispatchState {
  std::uint64_t session_config_calls{0U};
  std::uint64_t cycle_input_calls{0U};
};

bool CountSessionConfigCallback(const oneq::replay::ReplayTraceReadEvent& event,
                                void* user_data,
                                std::string* error) {
  (void)error;
  PlaybackDispatchState* state = static_cast<PlaybackDispatchState*>(user_data);
  ++state->session_config_calls;
  return event.payload_type == "RadarSessionConfig";
}

bool CountCycleInputCallback(const oneq::replay::ReplayTraceReadEvent& event,
                             void* user_data,
                             std::string* error) {
  (void)error;
  PlaybackDispatchState* state = static_cast<PlaybackDispatchState*>(user_data);
  ++state->cycle_input_calls;
  return event.payload_type == "RadarCycleInput";
}

bool EchoOutputCallback(const oneq::replay::ReplayTraceReadEvent& event,
                        void* user_data,
                        std::string* actual_output_json,
                        std::string* error) {
  (void)user_data;
  (void)error;
  *actual_output_json = event.payload_json;
  return true;
}

bool AlwaysOkEventCallback(const oneq::replay::ReplayTraceReadEvent& event,
                           void* user_data,
                           std::string* error) {
  (void)event;
  (void)user_data;
  (void)error;
  return true;
}

bool DivergentOutputCallback(const oneq::replay::ReplayTraceReadEvent& event,
                             void* user_data,
                             std::string* actual_output_json,
                             std::string* error) {
  (void)event;
  (void)user_data;
  (void)error;
  *actual_output_json = "{\"track_count\":1}";
  return true;
}

}  // namespace

namespace oneq {
namespace replay {
namespace tests {

TEST(ReplayTraceWriterTest, WritesManifestAndReplayEventEnvelope) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "trace-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "scenario-a";
  manifest.git_commit = "abc123";
  manifest.platform = "test-platform";
  manifest.default_tolerances_json = "{\"float_abs\":0.001}";
  manifest.failure_window_event_count = 64U;

  ReplayTraceWriter writer(trace_dir, manifest, true);

  ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = "cycle_input";
  event.payload_type = "RadarCycleInput";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = MakeFlatbuffersPayloadBytes("manifest-envelope");
  event.has_cycle_index = true;
  event.cycle_index = 7U;
  event.has_sim_time_sec = true;
  event.sim_time_sec = 7.0;

  writer.WriteEvent(event);
  writer.Flush();

  const std::string manifest_content = ReadFile(JoinPath(trace_dir, "manifest.json"));
  EXPECT_NE(manifest_content.find("\"trace_id\":\"trace-test\""), std::string::npos);
  EXPECT_NE(manifest_content.find("\"scenario_id\":\"scenario-a\""), std::string::npos);
  EXPECT_NE(manifest_content.find("\"default_tolerances\":{\"float_abs\":0.001}"),
            std::string::npos);
  EXPECT_NE(manifest_content.find("\"failure_window_event_count\":64"),
            std::string::npos);

  const std::string event_path = JoinPath(JoinPath(trace_dir, "events"),
                                         "000000.events.jsonl");
  const std::string event_content = ReadFile(event_path);
  EXPECT_NE(event_content.find("\"event_type\":\"cycle_input\""), std::string::npos);
  EXPECT_NE(event_content.find("\"cycle_index\":7"), std::string::npos);
  EXPECT_NE(event_content.find("\"payload_type\":\"RadarCycleInput\""), std::string::npos);
  EXPECT_NE(event_content.find("\"payload\":null"), std::string::npos);
  EXPECT_NE(event_content.find("\"payload_base64\":\""), std::string::npos);
  EXPECT_NE(event_content.find("\"payload_hash\":\"fnv1a64:"), std::string::npos);
}

TEST(ReplayTraceWriterTest, ReaderIteratesEventsAndValidatesPayloadHash) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "reader-trace-test";
  manifest.module = "airborne_radar";
  manifest.scenario_id = "reader-scenario";

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    ReplayTraceEvent config_event;
    config_event.module = "airborne_radar";
    config_event.event_type = "session_config";
    config_event.payload_type = "RadarSessionConfig";
    config_event.payload_encoding = "flatbuffers";
    config_event.payload_bytes = MakeFlatbuffersPayloadBytes("reader-config");
    writer.WriteEvent(config_event);

    ReplayTraceEvent input_event;
    input_event.module = "airborne_radar";
    input_event.event_type = "cycle_input";
    input_event.payload_type = "RadarCycleInput";
    input_event.payload_encoding = "flatbuffers";
    input_event.payload_bytes = MakeFlatbuffersPayloadBytes("reader-input");
    input_event.has_cycle_index = true;
    input_event.cycle_index = 9U;
    input_event.has_sim_time_sec = true;
    input_event.sim_time_sec = 9.0;
    writer.WriteEvent(input_event);
    writer.Flush();
  }

  ReplayTraceReader reader(trace_dir);
  EXPECT_NE(reader.manifest_json().find("\"trace_id\":\"reader-trace-test\""),
            std::string::npos);

  ReplayTraceReadEvent event;
  ASSERT_TRUE(reader.ReadNextEvent(&event));
  EXPECT_EQ(event.sequence, 0U);
  EXPECT_EQ(event.event_type, "session_config");
  EXPECT_EQ(event.payload_type, "RadarSessionConfig");
  EXPECT_EQ(event.payload_encoding, "flatbuffers");
  EXPECT_EQ(event.payload_json, "null");
  EXPECT_EQ(event.payload_bytes, MakeFlatbuffersPayloadBytes("reader-config"));
  EXPECT_TRUE(event.payload_hash_matches);
  EXPECT_TRUE(event.previous_event_hash_matches);
  EXPECT_FALSE(event.event_hash.empty());
  EXPECT_FALSE(event.has_cycle_index);

  ASSERT_TRUE(reader.ReadNextEvent(&event));
  EXPECT_EQ(event.sequence, 1U);
  EXPECT_EQ(event.event_type, "cycle_input");
  EXPECT_EQ(event.payload_type, "RadarCycleInput");
  EXPECT_TRUE(event.has_cycle_index);
  EXPECT_EQ(event.cycle_index, 9U);
  EXPECT_TRUE(event.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(event.sim_time_sec, 9.0);
  EXPECT_EQ(event.payload_encoding, "flatbuffers");
  EXPECT_EQ(event.payload_json, "null");
  EXPECT_EQ(event.payload_bytes, MakeFlatbuffersPayloadBytes("reader-input"));
  EXPECT_TRUE(event.payload_hash_matches);
  EXPECT_TRUE(event.previous_event_hash_matches);
  EXPECT_FALSE(event.event_hash.empty());

  EXPECT_FALSE(reader.ReadNextEvent(&event));
}

TEST(ReplayTraceWriterTest, ReaderRestoresBinaryPayloadBytes) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "binary-payload-test";
  manifest.module = "airborne_radar";
  manifest.serializer_version = "replay-flatbuffers-v1";

  const std::string payload_bytes("fb\0\1", 4U);
  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "cycle_input";
    event.payload_type = "RadarCycleInput";
    event.payload_encoding = "flatbuffers";
    event.payload_json = "{}";
    event.payload_bytes = payload_bytes;
    writer.WriteEvent(event);
    writer.Flush();
  }

  const std::string event_content =
      ReadFile(JoinPath(JoinPath(trace_dir, "events"), "000000.events.jsonl"));
  EXPECT_NE(event_content.find("\"payload_encoding\":\"flatbuffers\""),
            std::string::npos);
  EXPECT_NE(event_content.find("\"payload\":null"), std::string::npos);
  EXPECT_NE(event_content.find("\"payload_base64\":\"ZmIAAQ==\""),
            std::string::npos);

  ReplayTraceReader reader(trace_dir);
  ReplayTraceReadEvent event;
  ASSERT_TRUE(reader.ReadNextEvent(&event));
  EXPECT_EQ(event.payload_encoding, "flatbuffers");
  EXPECT_EQ(event.payload_json, "null");
  EXPECT_EQ(event.payload_bytes, payload_bytes);
  EXPECT_TRUE(event.payload_hash_matches);
  EXPECT_FALSE(event.payload_hash.empty());
  EXPECT_FALSE(reader.ReadNextEvent(&event));
}

TEST(ReplayTraceWriterTest, ScanReportsEventCountAndDetectsTampering) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "scan-trace-test";
  manifest.module = "airborne_radar";

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    ReplayTraceEvent first;
    first.module = "airborne_radar";
    first.event_type = "cycle_input";
    first.payload_type = "RadarCycleInput";
    first.payload_encoding = "flatbuffers";
    first.payload_bytes = MakeFlatbuffersPayloadBytes("scan-input");
    writer.WriteEvent(first);

    ReplayTraceEvent second;
    second.module = "airborne_radar";
    second.event_type = "cycle_output";
    second.payload_type = "RadarCycleResult";
    second.payload_encoding = "flatbuffers";
    second.payload_bytes = MakeFlatbuffersPayloadBytes("scan-output");
    writer.WriteEvent(second);
    writer.Flush();
  }

  ReplayTraceScanResult scan = ScanReplayTrace(trace_dir);
  EXPECT_TRUE(scan.ok);
  EXPECT_EQ(scan.event_count, 2U);
  EXPECT_TRUE(scan.payload_hashes_ok);
  EXPECT_TRUE(scan.event_chain_ok);
  EXPECT_TRUE(scan.sequences_contiguous);
  EXPECT_TRUE(scan.first_error.empty());

  const std::string event_path = JoinPath(JoinPath(trace_dir, "events"),
                                         "000000.events.jsonl");
  const std::string tampered = ReplaceFirst(ReadFile(event_path),
                                           "\"payload_hash\":\"fnv1a64:",
                                           "\"payload_hash\":\"fnv1a64:0000000000000000");
  WriteFile(event_path, tampered);

  scan = ScanReplayTrace(trace_dir);
  EXPECT_FALSE(scan.ok);
  EXPECT_EQ(scan.event_count, 2U);
  EXPECT_FALSE(scan.payload_hashes_ok);
  EXPECT_FALSE(scan.event_chain_ok);
  EXPECT_TRUE(scan.sequences_contiguous);
  EXPECT_NE(scan.first_error.find("payload hash mismatch"), std::string::npos);
}

TEST(ReplayTraceWriterTest, WritesFailureMarkerAndLastWindowPackage) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "failure-trace-test";
  manifest.module = "airborne_radar";
  manifest.failure_window_event_count = 2U;

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    ReplayTraceEvent first;
    first.module = "airborne_radar";
    first.event_type = "cycle_input";
    first.payload_type = "RadarCycleInput";
    first.payload_encoding = "flatbuffers";
    first.payload_bytes = MakeFlatbuffersPayloadBytes("window-input");
    writer.WriteEvent(first);

    ReplayTraceEvent second;
    second.module = "airborne_radar";
    second.event_type = "cycle_output";
    second.payload_type = "RadarCycleResult";
    second.payload_encoding = "flatbuffers";
    second.payload_bytes = MakeFlatbuffersPayloadBytes("window-output");
    writer.WriteEvent(second);

    ReplayTraceFailure failure;
    failure.error_code = "ASSERTION_FAILED";
    failure.message = "track diverged";
    failure.location = "RadarTraceSession::Step";
    failure.has_cycle_index = true;
    failure.cycle_index = 1U;
    failure.has_sim_time_sec = true;
    failure.sim_time_sec = 1.0;
    failure.diagnostics_json = "{\"track_id\":17}";
    writer.WriteFailureMarker(failure, MakeFlatbuffersPayloadBytes("window-failure"));
    writer.Flush();
  }

  const std::string failure_path = JoinPath(JoinPath(trace_dir, "crash"),
                                           "failure.json");
  const std::string failure_content = ReadFile(failure_path);
  EXPECT_NE(failure_content.find("\"failure_marker_sequence\":2"), std::string::npos);
  EXPECT_NE(failure_content.find("\"last_event_sequence\":1"), std::string::npos);
  EXPECT_NE(failure_content.find("\"error_code\":\"ASSERTION_FAILED\""),
            std::string::npos);
  EXPECT_NE(failure_content.find("\"diagnostics\":{\"track_id\":17}"),
            std::string::npos);

  const std::string window_path = JoinPath(JoinPath(trace_dir, "crash"),
                                          "last-window.events.jsonl");
  const std::string window_content = ReadFile(window_path);
  EXPECT_NE(window_content.find("\"event_type\":\"cycle_output\""),
            std::string::npos);
  EXPECT_NE(window_content.find("\"event_type\":\"failure_marker\""),
            std::string::npos);
  EXPECT_NE(window_content.find("\"payload_base64\":\""), std::string::npos);

  ReplayTraceScanResult scan = ScanReplayTrace(trace_dir);
  EXPECT_TRUE(scan.ok);
  EXPECT_EQ(scan.event_count, 3U);
}

TEST(ReplayTraceWriterTest, SplitsEventChunksAndWritesCycleIndex) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "chunk-trace-test";
  manifest.module = "airborne_radar";
  manifest.event_chunk_size = 2U;

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    for (std::uint32_t i = 0U; i < 5U; ++i) {
      ReplayTraceEvent event;
      event.module = "airborne_radar";
      event.event_type = "cycle_input";
      event.payload_type = "RadarCycleInput";
      event.payload_encoding = "flatbuffers";
      std::ostringstream payload_tag;
      payload_tag << "chunk-cycle-" << i;
      event.payload_bytes = MakeFlatbuffersPayloadBytes(payload_tag.str());
      event.has_cycle_index = true;
      event.cycle_index = i;
      event.has_sim_time_sec = true;
      event.sim_time_sec = static_cast<double>(i);
      writer.WriteEvent(event);
    }
    writer.Flush();
  }

  const std::string first_chunk = ReadFile(JoinPath(JoinPath(trace_dir, "events"),
                                                   "000000.events.jsonl"));
  const std::string second_chunk = ReadFile(JoinPath(JoinPath(trace_dir, "events"),
                                                    "000001.events.jsonl"));
  const std::string third_chunk = ReadFile(JoinPath(JoinPath(trace_dir, "events"),
                                                   "000002.events.jsonl"));
  EXPECT_NE(first_chunk.find("\"sequence\":0"), std::string::npos);
  EXPECT_NE(first_chunk.find("\"sequence\":1"), std::string::npos);
  EXPECT_NE(second_chunk.find("\"sequence\":2"), std::string::npos);
  EXPECT_NE(second_chunk.find("\"sequence\":3"), std::string::npos);
  EXPECT_NE(third_chunk.find("\"sequence\":4"), std::string::npos);

  ReplayTraceReader reader(trace_dir);
  ReplayTraceReadEvent read_event;
  for (std::uint64_t expected = 0U; expected < 5U; ++expected) {
    ASSERT_TRUE(reader.ReadNextEvent(&read_event));
    EXPECT_EQ(read_event.sequence, expected);
    EXPECT_TRUE(read_event.payload_hash_matches);
    EXPECT_TRUE(read_event.previous_event_hash_matches);
  }
  EXPECT_FALSE(reader.ReadNextEvent(&read_event));

  const std::string cycle_index = ReadFile(JoinPath(JoinPath(trace_dir, "indexes"),
                                                   "cycles.idx"));
  EXPECT_NE(cycle_index.find("\"cycle_index\":4"), std::string::npos);
  EXPECT_NE(cycle_index.find("\"sequence\":4"), std::string::npos);
  EXPECT_TRUE(cycle_index.find("events\\\\000002.events.jsonl") != std::string::npos ||
              cycle_index.find("events/000002.events.jsonl") != std::string::npos);

  ReplayTraceScanResult scan = ScanReplayTrace(trace_dir);
  EXPECT_TRUE(scan.ok);
  EXPECT_EQ(scan.event_count, 5U);
}

TEST(ReplayTraceWriterTest, ChecksManifestCompatibilityBeforeReplay) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "compat-trace-test";
  manifest.module = "airborne_radar";
  manifest.schema_version = 1;
  manifest.serializer_version = "replay-flatbuffers-v1";
  manifest.git_commit = "abc123";
  manifest.git_dirty = true;

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);
    ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "session_config";
    event.payload_type = "RadarSessionConfig";
    event.payload_encoding = "flatbuffers";
    event.payload_bytes = MakeFlatbuffersPayloadBytes("compat");
    writer.WriteEvent(event);
    writer.Flush();
  }

  ReplayTraceCompatibilityExpectation expectation;
  expectation.schema_version = 1;
  expectation.serializer_version = "replay-flatbuffers-v1";
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;
  expectation.git_commit = "abc123";
  expectation.require_git_commit_match = true;

  ReplayTraceCompatibilityResult result =
      CheckReplayTraceCompatibility(trace_dir, expectation);
  EXPECT_TRUE(result.compatible);
  EXPECT_TRUE(result.schema_version_matches);
  EXPECT_TRUE(result.serializer_version_matches);
  EXPECT_TRUE(result.git_commit_matches);
  EXPECT_TRUE(result.module_matches);
  EXPECT_TRUE(result.manifest_git_dirty);
  EXPECT_EQ(result.manifest_trace_id, "compat-trace-test");
  EXPECT_EQ(result.manifest_module, "airborne_radar");
  EXPECT_NE(result.warning.find("dirty git worktree"), std::string::npos);

  expectation.schema_version = 2;
  result = CheckReplayTraceCompatibility(trace_dir, expectation);
  EXPECT_FALSE(result.compatible);
  EXPECT_FALSE(result.schema_version_matches);
  EXPECT_NE(result.first_error.find("schema mismatch"), std::string::npos);

  expectation.schema_version = 1;
  expectation.git_commit = "different";
  result = CheckReplayTraceCompatibility(trace_dir, expectation);
  EXPECT_FALSE(result.compatible);
  EXPECT_FALSE(result.git_commit_matches);
  EXPECT_NE(result.first_error.find("git commit mismatch"), std::string::npos);
}

TEST(ReplayTraceWriterTest, BuildsReplayReportForBComputerEntry) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "report-trace-test";
  manifest.module = "airborne_radar";

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    ReplayTraceEvent config;
    config.module = "airborne_radar";
    config.event_type = "session_config";
    config.payload_type = "RadarSessionConfig";
    config.payload_encoding = "flatbuffers";
    config.payload_bytes = MakeFlatbuffersPayloadBytes("report-config");
    writer.WriteEvent(config);

    ReplayTraceEvent input;
    input.module = "airborne_radar";
    input.event_type = "cycle_input";
    input.payload_type = "RadarCycleInput";
    input.payload_encoding = "flatbuffers";
    input.payload_bytes = MakeFlatbuffersPayloadBytes("report-input");
    input.has_cycle_index = true;
    input.cycle_index = 3U;
    writer.WriteEvent(input);

    ReplayTraceEvent output;
    output.module = "airborne_radar";
    output.event_type = "cycle_output";
    output.payload_type = "RadarCycleResult";
    output.payload_encoding = "flatbuffers";
    output.payload_bytes = MakeFlatbuffersPayloadBytes("report-output");
    output.has_cycle_index = true;
    output.cycle_index = 3U;
    writer.WriteEvent(output);

    ReplayTraceFailure failure;
    failure.error_code = "DIVERGED";
    failure.message = "output mismatch";
    failure.has_cycle_index = true;
    failure.cycle_index = 3U;
    writer.WriteFailureMarker(failure, MakeFlatbuffersPayloadBytes("report-failure"));
    writer.Flush();
  }

  ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;

  ReplayTraceReplayReport report = BuildReplayTraceReport(trace_dir, expectation);
  EXPECT_TRUE(report.replay_ready);
  EXPECT_TRUE(report.compatibility.compatible);
  EXPECT_TRUE(report.scan.ok);
  EXPECT_TRUE(report.has_session_config);
  EXPECT_TRUE(report.has_failure_marker);
  EXPECT_EQ(report.session_config_count, 1U);
  EXPECT_EQ(report.cycle_input_count, 1U);
  EXPECT_EQ(report.cycle_output_count, 1U);
  EXPECT_EQ(report.failure_marker_count, 1U);
  EXPECT_EQ(report.unsupported_event_count, 0U);
  EXPECT_EQ(report.first_failure_sequence, 3U);
  EXPECT_FALSE(report.first_failure_payload_json.empty());

  const std::string report_path = JoinPath(trace_dir, "replay_report.json");
  WriteReplayTraceReport(report, report_path);
  const std::string report_json = ReadFile(report_path);
  EXPECT_NE(report_json.find("\"replay_ready\":true"), std::string::npos);
  EXPECT_NE(report_json.find("\"cycle_input_count\":1"), std::string::npos);
  EXPECT_NE(report_json.find("\"first_failure_payload\":"), std::string::npos);
}

TEST(ReplayTraceWriterTest, ReplayReportRejectsMissingSessionConfig) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "missing-config-report-test";
  manifest.module = "airborne_radar";

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);
    ReplayTraceEvent input;
    input.module = "airborne_radar";
    input.event_type = "cycle_input";
    input.payload_type = "RadarCycleInput";
    input.payload_encoding = "flatbuffers";
    input.payload_bytes = MakeFlatbuffersPayloadBytes("missing-config-input");
    writer.WriteEvent(input);
    writer.Flush();
  }

  ReplayTraceCompatibilityExpectation expectation;
  expectation.module = "airborne_radar";
  expectation.require_module_match = true;

  ReplayTraceReplayReport report = BuildReplayTraceReport(trace_dir, expectation);
  EXPECT_FALSE(report.replay_ready);
  EXPECT_FALSE(report.has_session_config);
  EXPECT_NE(report.first_error.find("session_config"), std::string::npos);
}

TEST(ReplayTraceWriterTest, PlaybackDispatchesEventsAndComparesOutput) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "playback-trace-test";
  manifest.module = "airborne_radar";

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    ReplayTraceEvent config;
    config.module = "airborne_radar";
    config.event_type = "session_config";
    config.payload_type = "RadarSessionConfig";
    config.payload_encoding = "flatbuffers";
    config.payload_bytes = MakeFlatbuffersPayloadBytes("playback-config");
    writer.WriteEvent(config);

    ReplayTraceEvent input;
    input.module = "airborne_radar";
    input.event_type = "cycle_input";
    input.payload_type = "RadarCycleInput";
    input.payload_encoding = "flatbuffers";
    input.payload_bytes = MakeFlatbuffersPayloadBytes("playback-input");
    writer.WriteEvent(input);

    ReplayTraceEvent output;
    output.module = "airborne_radar";
    output.event_type = "cycle_output";
    output.payload_type = "RadarCycleResult";
    output.payload_encoding = "flatbuffers";
    output.payload_bytes = MakeFlatbuffersPayloadBytes("playback-output");
    writer.WriteEvent(output);
    writer.Flush();
  }

  PlaybackDispatchState state;
  ReplayTracePlaybackCallbacks callbacks;
  callbacks.user_data = &state;
  callbacks.on_session_config = CountSessionConfigCallback;
  callbacks.on_cycle_input = CountCycleInputCallback;
  callbacks.on_cycle_output = EchoOutputCallback;

  ReplayTracePlaybackResult result = PlaybackReplayTrace(trace_dir, callbacks);
  EXPECT_TRUE(result.ok);
  EXPECT_FALSE(result.divergence_found);
  EXPECT_EQ(result.processed_event_count, 3U);
  EXPECT_EQ(result.applied_input_count, 1U);
  EXPECT_EQ(result.compared_output_count, 1U);
  EXPECT_EQ(state.session_config_calls, 1U);
  EXPECT_EQ(state.cycle_input_calls, 1U);
}

TEST(ReplayTraceWriterTest, PlaybackStopsOnOutputDivergence) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "divergence-trace-test";
  manifest.module = "airborne_radar";

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);

    ReplayTraceEvent config;
    config.module = "airborne_radar";
    config.event_type = "session_config";
    config.payload_type = "RadarSessionConfig";
    config.payload_encoding = "flatbuffers";
    config.payload_bytes = MakeFlatbuffersPayloadBytes("divergence-config");
    writer.WriteEvent(config);

    ReplayTraceEvent input;
    input.module = "airborne_radar";
    input.event_type = "cycle_input";
    input.payload_type = "RadarCycleInput";
    input.payload_encoding = "flatbuffers";
    input.payload_bytes = MakeFlatbuffersPayloadBytes("divergence-input");
    writer.WriteEvent(input);

    ReplayTraceEvent output;
    output.module = "airborne_radar";
    output.event_type = "cycle_output";
    output.payload_type = "RadarCycleResult";
    output.payload_encoding = "flatbuffers";
    output.payload_bytes = MakeFlatbuffersPayloadBytes("divergence-output");
    writer.WriteEvent(output);
    writer.Flush();
  }

  ReplayTracePlaybackCallbacks callbacks;
  callbacks.on_session_config = AlwaysOkEventCallback;
  callbacks.on_cycle_input = AlwaysOkEventCallback;
  callbacks.on_cycle_output = DivergentOutputCallback;

  ReplayTracePlaybackOptions options;
  options.stop_on_first_divergence = true;

  ReplayTracePlaybackResult result =
      PlaybackReplayTrace(trace_dir, callbacks, options);
  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.divergence_found);
  EXPECT_EQ(result.divergence_sequence, 2U);
  EXPECT_EQ(result.expected_output_json, "null");
  EXPECT_EQ(result.actual_output_json, "{\"track_count\":1}");
  EXPECT_NE(result.first_error.find("divergence"), std::string::npos);
}

TEST(ReplayTraceWriterTest, PlaybackRejectsMissingRequiredCallback) {
  const std::string trace_dir = MakeTempTraceDir();

  ReplayTraceManifest manifest;
  manifest.trace_id = "missing-callback-trace-test";
  manifest.module = "airborne_radar";

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);
    ReplayTraceEvent config;
    config.module = "airborne_radar";
    config.event_type = "session_config";
    config.payload_type = "RadarSessionConfig";
    config.payload_encoding = "flatbuffers";
    config.payload_bytes = MakeFlatbuffersPayloadBytes("missing-callback-config");
    writer.WriteEvent(config);
    writer.Flush();
  }

  ReplayTracePlaybackCallbacks callbacks;
  ReplayTracePlaybackResult result = PlaybackReplayTrace(trace_dir, callbacks);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.first_error.find("session_config"), std::string::npos);
}

}  // namespace tests
}  // namespace replay
}  // namespace oneq
