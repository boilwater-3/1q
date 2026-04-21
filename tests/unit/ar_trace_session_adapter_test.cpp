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

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"
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
