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

#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "electro_optical_sensor/session/EosReplayFlatbufferCodec.h"

namespace electro_optical_sensor {
namespace tests {
namespace {

std::string MakeTempDir(const char* prefix) {
  std::ostringstream stream;
  stream << "/tmp/" << prefix << "-" << std::time(nullptr) << "-"
         << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "-"
         << std::rand();
  return stream.str();
}

std::vector<std::uint8_t> ReadBinaryFile(const std::string& path) {
  std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
  return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(input)),
                                   std::istreambuf_iterator<char>());
}

TEST(EosTraceSessionAdapterTest, WritesConfigInputAndOutputToTraceSink) {
  const std::string trace_path = MakeTempDir("oneq-eos-trace");
  const std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::FlatbufferFileTraceSink(trace_path, false));

  config::EosSessionConfig config;
  config.policy.detection.minimum_snr_db = 4.5f;
  session::EosTraceSessionOptions options;
  options.sink = sink;
  session::EosTraceSession traced(config, options);
  session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1f;  // 合法步长：受 53c56e21 收紧的 dt_sec <= 10/frame_rate_hz 上界约束（30Hz → ≈0.333s）。
  (void)traced.StepWithResult(input);

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
    const flexbuffers::Map map =
        flexbuffers::GetRoot(content.data() + offset + 4U, payload_size).AsMap();
    if (map["module"].AsString().str() == "electro_optical_sensor") {
      const std::string phase = map["phase"].AsString().str();
      saw_config = saw_config || phase == "config";
      saw_input = saw_input || phase == "input";
      saw_output = saw_output || phase == "output";
    }
    offset += 4U + payload_size;
  }
  EXPECT_TRUE(saw_config);
  EXPECT_TRUE(saw_input);
  EXPECT_TRUE(saw_output);

  std::remove(trace_path.c_str());
}

TEST(EosTraceSessionAdapterTest, ValidationFailureUsesInputCycleIndex) {
  const std::string trace_dir = MakeTempDir("oneq-eos-validation");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "eos-validation";
  manifest.module = "electro_optical_sensor";
  const auto writer = std::shared_ptr<oneq::replay::ReplayTraceWriter>(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
  session::EosTraceSessionOptions options;
  options.replay_writer = writer;
  session::EosTraceSession traced(config::EosSessionConfig{}, options);
  session::EosCycleInput input;
  input.cycle_index = 77U;
  input.dt_sec = -1.0f;
  (void)traced.StepWithResult(input);
  ASSERT_EQ(writer->Flush(), oneq::replay::ReplayTraceWriteStatus::kSuccess);

  oneq::replay::ReplayTraceReader reader(trace_dir);
  oneq::replay::ReplayTraceReadEvent event;
  bool saw_rejected_output = false;
  while (reader.ReadNextEvent(&event) == oneq::replay::ReplayTraceReadStatus::kEvent) {
    if (event.event_type != "cycle_output") {
      continue;
    }
    session::EosCycleResult result;
    ASSERT_TRUE(session::DecodeEosCycleResult(event.payload_bytes, &result));
    if (::electro_optical_sensor::session::HasValidationError(result.issues)) {
      saw_rejected_output = true;
      EXPECT_TRUE(event.has_cycle_index);
      EXPECT_EQ(event.cycle_index, 77U);
      EXPECT_EQ(result.input_cycle_index, 77U);
    }
  }
  EXPECT_TRUE(saw_rejected_output);
}

}  // namespace
}  // namespace tests
}  // namespace electro_optical_sensor
