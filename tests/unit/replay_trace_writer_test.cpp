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

}  // namespace tests
}  // namespace replay
}  // namespace oneq
