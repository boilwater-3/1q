#include "1q/trace/TraceSink.h"

#include <flatbuffers/flexbuffers.h>

#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "common/logging/ProjectLog.h"
#include "common/trace/TimeUtils.h"

namespace oneq {
namespace trace {

TraceSink::~TraceSink() = default;

FlatbufferFileTraceSink::FlatbufferFileTraceSink(std::string file_path, bool append)
    : file_path_(std::move(file_path)),
      output_(file_path_.c_str(), append ? (std::ios::out | std::ios::app | std::ios::binary)
                                         : (std::ios::out | std::ios::trunc | std::ios::binary)) {
  if (!output_.is_open()) {
    // 中译：无法打开 flatbuffer trace 文件：{}。
    // 标识：trace 记录初始化失败——文件路径不可写时后续记录全部无效，
    //       排查路径与权限。
    PROJECT_LOG_ERROR("failed to open flatbuffer trace file: {}", file_path_);
  }
}

void FlatbufferFileTraceSink::Record(const std::string& module, const std::string& phase,
                                     const std::string& payload_json) {
  const std::int64_t timestamp_ms = oneq::common::trace::CurrentTimestampMs();

  flexbuffers::Builder builder;
  builder.Map([&]() {
    builder.Int("timestamp_ms", timestamp_ms);
    builder.String("module", module);
    builder.String("phase", phase);
    builder.String("payload_json", payload_json);
  });
  builder.Finish();

  const std::vector<std::uint8_t>& payload = builder.GetBuffer();
  if (payload.size() > 0xFFFFFFFFull) {
    // 中译：flatbuffer trace 帧过大（{} 字节），本帧被丢弃。
    // 标识：帧大小保护——超过 4GiB 长度字段上限的帧不写入，
    //       防止头字段溢出。
    PROJECT_LOG_ERROR("flatbuffer trace frame is too large: {} bytes", payload.size());
    return;
  }

  const std::uint32_t frame_size = static_cast<std::uint32_t>(payload.size());
  const std::uint8_t header[4] = {
      static_cast<std::uint8_t>(frame_size & 0xFFu),
      static_cast<std::uint8_t>((frame_size >> 8) & 0xFFu),
      static_cast<std::uint8_t>((frame_size >> 16) & 0xFFu),
      static_cast<std::uint8_t>((frame_size >> 24) & 0xFFu),
  };

  std::lock_guard<std::mutex> lock(mutex_);
  if (!output_.is_open()) {
    // 中译：flatbuffer trace 文件未打开：{}。
    // 标识：写入前置检查失败——文件不可用时放弃本帧记录。
    PROJECT_LOG_ERROR("flatbuffer trace file is not open: {}", file_path_);
    return;
  }
  output_.write(reinterpret_cast<const char*>(header), sizeof(header));
  output_.write(reinterpret_cast<const char*>(payload.data()),
                static_cast<std::streamsize>(payload.size()));
  output_.flush();
  if (output_.fail() || output_.bad()) {
    // 中译：写入 flatbuffer trace 帧失败：{}。
    // 标识：IO 失败——写入出错时记录错误，供排查磁盘/空间问题。
    PROJECT_LOG_ERROR("failed to write flatbuffer trace frame: {}", file_path_);
  }
}

const std::string& FlatbufferFileTraceSink::file_path() const { return file_path_; }

bool FlatbufferFileTraceSink::is_open() const { return output_.is_open(); }

}  // namespace trace
}  // namespace oneq
