#include "1q/common/trace/TraceSink.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <stdexcept>

#include "common/trace/JsonFormatUtils.h"

namespace oneq {
namespace common {
namespace trace {

TraceSink::~TraceSink() = default;

struct JsonlFileTraceSink::Impl {
  explicit Impl(std::string path, bool append)
      : file_path(std::move(path)),
        output(file_path.c_str(), append ? (std::ios::out | std::ios::app)
                                         : (std::ios::out | std::ios::trunc)) {
    if (!output.is_open()) {
      throw std::runtime_error("failed to open trace file: " + file_path);
    }
  }

  std::string file_path;
  std::ofstream output;
  std::mutex mutex;
};

JsonlFileTraceSink::JsonlFileTraceSink(std::string file_path, bool append)
    : impl_(new Impl(std::move(file_path), append)) {}

JsonlFileTraceSink::~JsonlFileTraceSink() = default;

void JsonlFileTraceSink::Record(const std::string& module, const std::string& phase,
                                const std::string& payload_json) {
  const std::int64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch())
                                        .count();

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->output << "{"
                << "\"timestamp_ms\":" << timestamp_ms << ","
                << "\"module\":" << internal::QuoteString(module) << ","
                << "\"phase\":" << internal::QuoteString(phase) << ","
                << "\"payload\":" << payload_json << "}"
                << '\n';
  impl_->output.flush();
}

const std::string& JsonlFileTraceSink::file_path() const { return impl_->file_path; }

}  // namespace trace
}  // namespace common
}  // namespace oneq
