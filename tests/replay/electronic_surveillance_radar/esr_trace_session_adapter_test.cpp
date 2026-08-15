#include <flatbuffers/flexbuffers.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"
#include "support/oneq_test_temp_dir.h"

namespace electronic_surveillance_radar {
namespace tests {
namespace {

std::string MakeTempPath(const char* prefix) {
  std::ostringstream stream;
  stream << oneq_test::TempDir() << prefix << "-" << std::time(nullptr) << "-"
         << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "-"
         << std::rand();
  return stream.str();
}

std::vector<std::uint8_t> ReadBinaryFile(const std::string& path) {
  std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
  return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(input)),
                                   std::istreambuf_iterator<char>());
}

TEST(EsrTraceSessionAdapterTest, WritesConfigInputAndOutputToTraceSink) {
  const std::string trace_path = MakeTempPath("oneq-esr-trace");
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::FlatbufferFileTraceSink(trace_path, false));
  config::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  session::EsrTraceSession traced(config, session::EsrTraceSessionOptions{sink, true});
  session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  (void)traced.StepWithResult(input);

  const std::vector<std::uint8_t> content = ReadBinaryFile(trace_path);
  bool saw_config = false;
  bool saw_input = false;
  bool saw_output = false;
  std::size_t offset = 0U;
  while (offset + 4U <= content.size()) {
    const std::uint32_t size = static_cast<std::uint32_t>(content[offset]) |
                               (static_cast<std::uint32_t>(content[offset + 1]) << 8U) |
                               (static_cast<std::uint32_t>(content[offset + 2]) << 16U) |
                               (static_cast<std::uint32_t>(content[offset + 3]) << 24U);
    ASSERT_LE(offset + 4U + size, content.size());
    const flexbuffers::Map map = flexbuffers::GetRoot(content.data() + offset + 4U, size).AsMap();
    if (map["module"].AsString().str() == "electronic_surveillance_radar") {
      const std::string phase = map["phase"].AsString().str();
      saw_config = saw_config || phase == "config";
      saw_input = saw_input || phase == "input";
      saw_output = saw_output || phase == "output";
    }
    offset += 4U + size;
  }
  EXPECT_TRUE(saw_config);
  EXPECT_TRUE(saw_input);
  EXPECT_TRUE(saw_output);
  std::remove(trace_path.c_str());
}

TEST(EsrTraceSessionAdapterTest, ValidationFailureUsesInputCycleIndex) {
  const std::string trace_dir = MakeTempPath("oneq-esr-validation");
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "esr-validation";
  manifest.module = "electronic_surveillance_radar";
  const auto writer = std::shared_ptr<oneq::replay::ReplayTraceWriter>(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
  session::EsrTraceSessionOptions options;
  options.replay_writer = writer;
  session::EsrTraceSession traced(config::EsrSessionConfig{}, options);
  session::EsrCycleInput input;
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
    session::EsrCycleResult result;
    ASSERT_TRUE(session::DecodeEsrCycleResult(event.payload_bytes, &result));
    if (session::HasValidationError(result.issues)) {
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
}  // namespace electronic_surveillance_radar
