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
  event.payload_json = "{\"dt_sec\":1.0,\"target_features\":[]}";
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
  EXPECT_NE(event_content.find("\"payload\":{\"dt_sec\":1.0,\"target_features\":[]}"),
            std::string::npos);
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
    config_event.payload_json = "{\"config\":true}";
    writer.WriteEvent(config_event);

    ReplayTraceEvent input_event;
    input_event.module = "airborne_radar";
    input_event.event_type = "cycle_input";
    input_event.payload_type = "RadarCycleInput";
    input_event.payload_json = "{\"dt_sec\":1.0,\"target_features\":[{\"external_target_id\":9}]}";
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
  EXPECT_EQ(event.payload_json, "{\"config\":true}");
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
  EXPECT_NE(event.payload_json.find("\"external_target_id\":9"), std::string::npos);
  EXPECT_TRUE(event.payload_hash_matches);
  EXPECT_TRUE(event.previous_event_hash_matches);
  EXPECT_FALSE(event.event_hash.empty());

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
    first.payload_json = "{\"dt_sec\":1.0}";
    writer.WriteEvent(first);

    ReplayTraceEvent second;
    second.module = "airborne_radar";
    second.event_type = "cycle_output";
    second.payload_type = "RadarCycleResult";
    second.payload_json = "{\"executed_this_cycle\":true}";
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
                                           "\"dt_sec\":1.0",
                                           "\"dt_sec\":2.0");
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
    first.payload_json = "{\"cycle\":1}";
    writer.WriteEvent(first);

    ReplayTraceEvent second;
    second.module = "airborne_radar";
    second.event_type = "cycle_output";
    second.payload_type = "RadarCycleResult";
    second.payload_json = "{\"cycle\":1,\"ok\":false}";
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
    writer.WriteFailureMarker(failure);
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
  EXPECT_EQ(window_content.find("\"cycle\":1}"), std::string::npos);
  EXPECT_NE(window_content.find("\"event_type\":\"cycle_output\""),
            std::string::npos);
  EXPECT_NE(window_content.find("\"event_type\":\"failure_marker\""),
            std::string::npos);
  EXPECT_NE(window_content.find("\"last_event_sequence\":1"), std::string::npos);

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
      std::ostringstream payload;
      payload << "{\"cycle\":" << i << "}";
      event.payload_json = payload.str();
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
  manifest.serializer_version = "replay-json-v1";
  manifest.git_commit = "abc123";
  manifest.git_dirty = true;

  {
    ReplayTraceWriter writer(trace_dir, manifest, true);
    ReplayTraceEvent event;
    event.module = "airborne_radar";
    event.event_type = "session_config";
    event.payload_type = "RadarSessionConfig";
    event.payload_json = "{\"config\":true}";
    writer.WriteEvent(event);
    writer.Flush();
  }

  ReplayTraceCompatibilityExpectation expectation;
  expectation.schema_version = 1;
  expectation.serializer_version = "replay-json-v1";
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

}  // namespace tests
}  // namespace replay
}  // namespace oneq
