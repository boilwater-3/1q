#include <flatbuffers/flexbuffers.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "1q/trace/TraceSink.h"

namespace oneq {
namespace trace {
namespace tests {
namespace {

std::string MakeTempPath(const char* prefix) {
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

TEST(TraceSinkAdapterTest, FlatbufferFileTraceSinkWritesBinaryFrame) {
  const std::string trace_path = MakeTempPath("oneq-flatbuffer-trace");
  FlatbufferFileTraceSink sink(trace_path, false);
  sink.Record("platform_module", "input", "{\"value\":1}");
  const std::vector<std::uint8_t> content = ReadBinaryFile(trace_path);
  ASSERT_GE(content.size(), 4U);
  const std::uint32_t size = static_cast<std::uint32_t>(content[0]) |
                             (static_cast<std::uint32_t>(content[1]) << 8U) |
                             (static_cast<std::uint32_t>(content[2]) << 16U) |
                             (static_cast<std::uint32_t>(content[3]) << 24U);
  ASSERT_EQ(content.size(), static_cast<std::size_t>(size) + 4U);
  const flexbuffers::Map map = flexbuffers::GetRoot(content.data() + 4U, size).AsMap();
  EXPECT_EQ(map["module"].AsString().str(), "platform_module");
  EXPECT_EQ(map["phase"].AsString().str(), "input");
  EXPECT_FALSE(map["payload_json"].AsString().str().empty());
  std::remove(trace_path.c_str());
}

TEST(TraceSinkAdapterTest, OpenFailureDoesNotThrow) {
  const std::string trace_path = MakeTempPath("oneq-missing-trace-parent") + "/trace.bin";
  FlatbufferFileTraceSink sink(trace_path, false);
  EXPECT_FALSE(sink.is_open());
  sink.Record("platform_module", "input", "{\"value\":1}");
  EXPECT_TRUE(ReadBinaryFile(trace_path).empty());
}

}  // namespace
}  // namespace tests
}  // namespace trace
}  // namespace oneq
