#include "1q/replay/ReplayTrace.h"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include "common/trace/JsonFormatUtils.h"

namespace oneq {
namespace replay {
namespace {

std::int64_t CurrentTimestampMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool IsPathSeparator(char value) { return value == '/' || value == '\\'; }

void CreateDirectoryIfMissing(const std::string& path) {
  if (path.empty()) {
    return;
  }

#if defined(_WIN32)
  const int result = _mkdir(path.c_str());
#else
  const int result = mkdir(path.c_str(), 0777);
#endif
  if (result != 0 && errno != EEXIST) {
    throw std::runtime_error("failed to create replay trace directory: " + path);
  }
}

void CreateDirectoryRecursive(const std::string& path) {
  std::string current;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const char c = path[i];
    current.push_back(c);

    if (i == 1 && c == ':') {
      continue;
    }
    if (IsPathSeparator(c)) {
      if (current.size() == 3U && current[1] == ':') {
        continue;
      }
      CreateDirectoryIfMissing(current);
    }
  }
  CreateDirectoryIfMissing(path);
}

std::string JoinPath(const std::string& left, const std::string& right) {
  if (left.empty()) {
    return right;
  }
  if (IsPathSeparator(left[left.size() - 1U])) {
    return left + right;
  }
#if defined(_WIN32)
  return left + "\\" + right;
#else
  return left + "/" + right;
#endif
}

std::string HashString(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ull;
  for (std::size_t i = 0; i < value.size(); ++i) {
    hash ^= static_cast<unsigned char>(value[i]);
    hash *= 1099511628211ull;
  }

  std::ostringstream stream;
  stream << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << hash;
  return stream.str();
}

void WriteJsonStringField(std::ostream& output, const char* name, const std::string& value,
                          bool trailing_comma) {
  output << "\"" << name << "\":"
         << trace::internal::QuoteString(value);
  if (trailing_comma) {
    output << ",";
  }
}

void WriteJsonRawField(std::ostream& output, const char* name, const std::string& value,
                       bool trailing_comma) {
  output << "\"" << name << "\":" << (value.empty() ? "{}" : value);
  if (trailing_comma) {
    output << ",";
  }
}

void WriteManifestFile(const std::string& path, const ReplayTraceManifest& manifest) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open replay manifest: " + path);
  }

  output << "{";
  WriteJsonStringField(output, "trace_id", manifest.trace_id, true);
  output << "\"created_wall_time_ms\":" << CurrentTimestampMs() << ",";
  WriteJsonStringField(output, "module", manifest.module, true);
  WriteJsonStringField(output, "scenario_id", manifest.scenario_id, true);
  output << "\"schema_version\":" << manifest.schema_version << ",";
  WriteJsonStringField(output, "serializer_version", manifest.serializer_version, true);
  WriteJsonStringField(output, "git_commit", manifest.git_commit, true);
  output << "\"git_dirty\":" << trace::internal::BoolToJson(manifest.git_dirty) << ",";
  WriteJsonStringField(output, "build_type", manifest.build_type, true);
  WriteJsonStringField(output, "compiler", manifest.compiler, true);
  WriteJsonStringField(output, "compiler_version", manifest.compiler_version, true);
  WriteJsonStringField(output, "platform", manifest.platform, true);
  WriteJsonStringField(output, "cpu_arch", manifest.cpu_arch, true);
  WriteJsonStringField(output, "library_version", manifest.library_version, true);
  WriteJsonRawField(output, "dependency_versions", manifest.dependency_versions_json, true);
  WriteJsonStringField(output, "float_policy", manifest.float_policy, true);
  WriteJsonRawField(output, "default_tolerances", manifest.default_tolerances_json, true);
  output << "\"checkpoint_interval_cycles\":" << manifest.checkpoint_interval_cycles << ",";
  output << "\"event_chunk_size\":" << manifest.event_chunk_size;
  output << "}\n";
}

}  // namespace

struct ReplayTraceWriter::Impl {
  Impl(std::string path, ReplayTraceManifest manifest_value, bool overwrite)
      : trace_dir(std::move(path)), manifest(std::move(manifest_value)) {
    CreateDirectoryRecursive(trace_dir);
    CreateDirectoryRecursive(JoinPath(trace_dir, "events"));
    CreateDirectoryRecursive(JoinPath(trace_dir, "checkpoints"));
    CreateDirectoryRecursive(JoinPath(trace_dir, "crash"));
    CreateDirectoryRecursive(JoinPath(trace_dir, "indexes"));

    WriteManifestFile(JoinPath(trace_dir, "manifest.json"), manifest);

    const std::string event_path = JoinPath(JoinPath(trace_dir, "events"),
                                           "000000.events.jsonl");
    events.open(event_path.c_str(),
                overwrite ? (std::ios::out | std::ios::trunc)
                          : (std::ios::out | std::ios::app));
    if (!events.is_open()) {
      throw std::runtime_error("failed to open replay event file: " + event_path);
    }
  }

  std::string trace_dir;
  ReplayTraceManifest manifest;
  std::ofstream events;
  std::uint64_t next_sequence{0U};
  std::string previous_event_hash{};
};

ReplayTraceWriter::ReplayTraceWriter(std::string trace_dir, ReplayTraceManifest manifest,
                                     bool overwrite)
    : impl_(new Impl(std::move(trace_dir), std::move(manifest), overwrite)) {}

ReplayTraceWriter::~ReplayTraceWriter() = default;

void ReplayTraceWriter::WriteEvent(const ReplayTraceEvent& event) {
  const std::string payload_hash = HashString(event.payload_json);
  const std::string previous_hash = impl_->previous_event_hash;

  std::ostringstream line;
  line << "{";
  line << "\"schema_version\":" << impl_->manifest.schema_version << ",";
  WriteJsonStringField(line, "trace_id", impl_->manifest.trace_id, true);
  line << "\"sequence\":" << impl_->next_sequence << ",";
  WriteJsonStringField(line, "module", event.module, true);
  WriteJsonStringField(line, "event_type", event.event_type, true);
  if (event.has_cycle_index) {
    line << "\"cycle_index\":" << event.cycle_index << ",";
  } else {
    line << "\"cycle_index\":null,";
  }
  if (event.has_sim_time_sec) {
    line << "\"sim_time_sec\":" << event.sim_time_sec << ",";
  } else {
    line << "\"sim_time_sec\":null,";
  }
  line << "\"wall_time_ms\":" << CurrentTimestampMs() << ",";
  WriteJsonStringField(line, "payload_type", event.payload_type, true);
  WriteJsonStringField(line, "payload_encoding", event.payload_encoding, true);
  WriteJsonRawField(line, "payload", event.payload_json, true);
  WriteJsonStringField(line, "payload_hash", payload_hash, true);
  WriteJsonStringField(line, "previous_event_hash", previous_hash, false);
  line << "}";

  const std::string serialized = line.str();
  impl_->events << serialized << '\n';
  impl_->events.flush();
  impl_->previous_event_hash = HashString(serialized);
  ++impl_->next_sequence;
}

void ReplayTraceWriter::Flush() { impl_->events.flush(); }

const std::string& ReplayTraceWriter::trace_dir() const { return impl_->trace_dir; }

const ReplayTraceManifest& ReplayTraceWriter::manifest() const { return impl_->manifest; }

}  // namespace replay
}  // namespace oneq
