#include "1q/replay/ReplayTrace.h"

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <deque>
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

std::string EventChunkFileName(std::uint32_t chunk_index) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << chunk_index << ".events.jsonl";
  return stream.str();
}

std::string EventChunkRelativePath(std::uint32_t chunk_index) {
  return JoinPath("events", EventChunkFileName(chunk_index));
}

std::string EventChunkPath(const std::string& trace_dir, std::uint32_t chunk_index) {
  return JoinPath(trace_dir, EventChunkRelativePath(chunk_index));
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

const char* Base64Alphabet() {
  return "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}

std::string Base64Encode(const std::string& bytes) {
  std::string output;
  std::uint32_t accumulator = 0U;
  int bits = -6;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    accumulator = (accumulator << 8) |
                  static_cast<unsigned char>(bytes[i]);
    bits += 8;
    while (bits >= 0) {
      output.push_back(Base64Alphabet()[(accumulator >> bits) & 0x3FU]);
      bits -= 6;
    }
  }
  if (bits > -6) {
    output.push_back(Base64Alphabet()[((accumulator << 8) >> (bits + 8)) & 0x3FU]);
  }
  while ((output.size() % 4U) != 0U) {
    output.push_back('=');
  }
  return output;
}

int Base64Value(char value) {
  if (value >= 'A' && value <= 'Z') {
    return value - 'A';
  }
  if (value >= 'a' && value <= 'z') {
    return value - 'a' + 26;
  }
  if (value >= '0' && value <= '9') {
    return value - '0' + 52;
  }
  if (value == '+') {
    return 62;
  }
  if (value == '/') {
    return 63;
  }
  return -1;
}

std::string Base64Decode(const std::string& text) {
  std::string output;
  std::uint32_t accumulator = 0U;
  int bits = -8;
  for (std::size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '=') {
      break;
    }
    const int value = Base64Value(c);
    if (value < 0) {
      continue;
    }
    accumulator = (accumulator << 6) | static_cast<std::uint32_t>(value);
    bits += 6;
    if (bits >= 0) {
      output.push_back(static_cast<char>((accumulator >> bits) & 0xFFU));
      bits -= 8;
    }
  }
  return output;
}

const std::string& PayloadBytesForHash(const ReplayTraceEvent& event) {
  if (event.payload_encoding != "json" && !event.payload_bytes.empty()) {
    return event.payload_bytes;
  }
  return event.payload_json;
}

const std::string& PayloadBytesForHash(const ReplayTraceReadEvent& event) {
  if (event.payload_encoding != "json" && !event.payload_bytes.empty()) {
    return event.payload_bytes;
  }
  return event.payload_json;
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
  output << "\"event_chunk_size\":" << manifest.event_chunk_size << ",";
  output << "\"failure_window_event_count\":" << manifest.failure_window_event_count;
  output << "}\n";
}

void WriteOptionalUInt32Field(std::ostream& output, const char* name, bool has_value,
                              std::uint32_t value, bool trailing_comma) {
  output << "\"" << name << "\":";
  if (has_value) {
    output << value;
  } else {
    output << "null";
  }
  if (trailing_comma) {
    output << ",";
  }
}

void WriteOptionalUInt64Field(std::ostream& output, const char* name, bool has_value,
                              std::uint64_t value, bool trailing_comma) {
  output << "\"" << name << "\":";
  if (has_value) {
    output << value;
  } else {
    output << "null";
  }
  if (trailing_comma) {
    output << ",";
  }
}

void WriteOptionalDoubleField(std::ostream& output, const char* name, bool has_value,
                              double value, bool trailing_comma) {
  output << "\"" << name << "\":";
  if (has_value) {
    output << value;
  } else {
    output << "null";
  }
  if (trailing_comma) {
    output << ",";
  }
}

std::string BuildFailurePayloadJson(const ReplayTraceFailure& failure,
                                    bool has_last_event_sequence,
                                    std::uint64_t last_event_sequence) {
  std::ostringstream payload;
  payload << "{";
  WriteJsonStringField(payload, "error_code", failure.error_code, true);
  WriteJsonStringField(payload, "message", failure.message, true);
  WriteJsonStringField(payload, "location", failure.location, true);
  WriteOptionalUInt32Field(payload, "cycle_index", failure.has_cycle_index,
                           failure.cycle_index, true);
  WriteOptionalDoubleField(payload, "sim_time_sec", failure.has_sim_time_sec,
                           failure.sim_time_sec, true);
  WriteOptionalUInt64Field(payload, "last_event_sequence", has_last_event_sequence,
                           last_event_sequence, true);
  WriteJsonRawField(payload, "diagnostics", failure.diagnostics_json, false);
  payload << "}";
  return payload.str();
}

void WriteFailureFile(const std::string& path, const ReplayTraceManifest& manifest,
                      const ReplayTraceFailure& failure,
                      std::uint64_t failure_marker_sequence,
                      bool has_last_event_sequence,
                      std::uint64_t last_event_sequence) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open replay failure file: " + path);
  }

  output << "{";
  output << "\"schema_version\":" << manifest.schema_version << ",";
  WriteJsonStringField(output, "trace_id", manifest.trace_id, true);
  WriteJsonStringField(output, "module", manifest.module, true);
  output << "\"created_wall_time_ms\":" << CurrentTimestampMs() << ",";
  output << "\"failure_marker_sequence\":" << failure_marker_sequence << ",";
  WriteOptionalUInt64Field(output, "last_event_sequence", has_last_event_sequence,
                           last_event_sequence, true);
  WriteJsonStringField(output, "error_code", failure.error_code, true);
  WriteJsonStringField(output, "message", failure.message, true);
  WriteJsonStringField(output, "location", failure.location, true);
  WriteOptionalUInt32Field(output, "cycle_index", failure.has_cycle_index,
                           failure.cycle_index, true);
  WriteOptionalDoubleField(output, "sim_time_sec", failure.has_sim_time_sec,
                           failure.sim_time_sec, true);
  WriteJsonRawField(output, "diagnostics", failure.diagnostics_json, false);
  output << "}\n";
}

void WriteLastWindowFile(const std::string& path,
                         const std::deque<std::string>& last_window) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open replay last-window file: " + path);
  }

  for (std::deque<std::string>::const_iterator it = last_window.begin();
       it != last_window.end(); ++it) {
    output << *it << '\n';
  }
}

void WriteCycleIndexLine(std::ostream& output, const ReplayTraceEvent& event,
                         std::uint64_t sequence, std::uint32_t chunk_index,
                         std::uint64_t byte_offset) {
  if (!event.has_cycle_index) {
    return;
  }

  output << "{";
  output << "\"cycle_index\":" << event.cycle_index << ",";
  output << "\"sequence\":" << sequence << ",";
  WriteJsonStringField(output, "event_file", EventChunkRelativePath(chunk_index), true);
  output << "\"byte_offset\":" << byte_offset << ",";
  WriteJsonStringField(output, "event_type", event.event_type, true);
  WriteOptionalDoubleField(output, "sim_time_sec", event.has_sim_time_sec,
                           event.sim_time_sec, false);
  output << "}\n";
}

void WriteReportFile(const std::string& path, const ReplayTraceReplayReport& report) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    throw std::runtime_error("failed to open replay report file: " + path);
  }

  output << "{";
  output << "\"replay_ready\":" << trace::internal::BoolToJson(report.replay_ready)
         << ",";
  WriteJsonStringField(output, "first_error", report.first_error, true);
  WriteJsonStringField(output, "warning", report.warning, true);

  output << "\"compatibility\":{";
  output << "\"compatible\":"
         << trace::internal::BoolToJson(report.compatibility.compatible) << ",";
  output << "\"schema_version_matches\":"
         << trace::internal::BoolToJson(report.compatibility.schema_version_matches)
         << ",";
  output << "\"serializer_version_matches\":"
         << trace::internal::BoolToJson(
                report.compatibility.serializer_version_matches)
         << ",";
  output << "\"git_commit_matches\":"
         << trace::internal::BoolToJson(report.compatibility.git_commit_matches)
         << ",";
  output << "\"module_matches\":"
         << trace::internal::BoolToJson(report.compatibility.module_matches)
         << ",";
  output << "\"manifest_git_dirty\":"
         << trace::internal::BoolToJson(report.compatibility.manifest_git_dirty)
         << ",";
  WriteJsonStringField(output, "manifest_trace_id",
                       report.compatibility.manifest_trace_id, true);
  WriteJsonStringField(output, "manifest_module",
                       report.compatibility.manifest_module, true);
  output << "\"manifest_schema_version\":"
         << report.compatibility.manifest_schema_version << ",";
  WriteJsonStringField(output, "manifest_serializer_version",
                       report.compatibility.manifest_serializer_version, true);
  WriteJsonStringField(output, "manifest_git_commit",
                       report.compatibility.manifest_git_commit, true);
  WriteJsonStringField(output, "first_error", report.compatibility.first_error,
                       true);
  WriteJsonStringField(output, "warning", report.compatibility.warning, false);
  output << "},";

  output << "\"scan\":{";
  output << "\"ok\":" << trace::internal::BoolToJson(report.scan.ok) << ",";
  output << "\"event_count\":" << report.scan.event_count << ",";
  output << "\"payload_hashes_ok\":"
         << trace::internal::BoolToJson(report.scan.payload_hashes_ok) << ",";
  output << "\"event_chain_ok\":"
         << trace::internal::BoolToJson(report.scan.event_chain_ok) << ",";
  output << "\"sequences_contiguous\":"
         << trace::internal::BoolToJson(report.scan.sequences_contiguous) << ",";
  WriteJsonStringField(output, "first_error", report.scan.first_error, false);
  output << "},";

  output << "\"events\":{";
  output << "\"has_session_config\":"
         << trace::internal::BoolToJson(report.has_session_config) << ",";
  output << "\"has_failure_marker\":"
         << trace::internal::BoolToJson(report.has_failure_marker) << ",";
  output << "\"session_config_count\":" << report.session_config_count << ",";
  output << "\"cycle_input_count\":" << report.cycle_input_count << ",";
  output << "\"scene_state_count\":" << report.scene_state_count << ",";
  output << "\"runtime_config_patch_count\":"
         << report.runtime_config_patch_count << ",";
  output << "\"cycle_output_count\":" << report.cycle_output_count << ",";
  output << "\"failure_marker_count\":" << report.failure_marker_count << ",";
  output << "\"unsupported_event_count\":" << report.unsupported_event_count
         << ",";
  output << "\"first_failure_sequence\":" << report.first_failure_sequence << ",";
  WriteJsonRawField(output, "first_failure_payload",
                    report.first_failure_payload_json, false);
  output << "}";
  output << "}\n";
}

bool InvokeReplayCallback(ReplayTraceEventCallback callback,
                          const ReplayTraceReadEvent& event,
                          void* user_data,
                          const char* missing_callback_error,
                          bool required,
                          std::string* error) {
  if (!callback) {
    if (required) {
      *error = missing_callback_error;
      return false;
    }
    return true;
  }
  return callback(event, user_data, error);
}

std::string ReadWholeFile(const std::string& path) {
  std::ifstream input(path.c_str(), std::ios::in);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open replay trace file: " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::size_t FindFieldValueStart(const std::string& json, const std::string& field_name) {
  const std::string key = "\"" + field_name + "\":";
  const std::size_t key_pos = json.find(key);
  if (key_pos == std::string::npos) {
    return std::string::npos;
  }
  return key_pos + key.size();
}

std::string ExtractStringField(const std::string& json, const std::string& field_name) {
  std::size_t value_pos = FindFieldValueStart(json, field_name);
  if (value_pos == std::string::npos || value_pos >= json.size() || json[value_pos] != '"') {
    return "";
  }
  ++value_pos;

  std::ostringstream output;
  bool escaping = false;
  for (std::size_t i = value_pos; i < json.size(); ++i) {
    const char c = json[i];
    if (escaping) {
      switch (c) {
        case 'n':
          output << '\n';
          break;
        case 'r':
          output << '\r';
          break;
        case 't':
          output << '\t';
          break;
        default:
          output << c;
          break;
      }
      escaping = false;
      continue;
    }
    if (c == '\\') {
      escaping = true;
      continue;
    }
    if (c == '"') {
      break;
    }
    output << c;
  }
  return output.str();
}

std::uint64_t ExtractUInt64Field(const std::string& json, const std::string& field_name) {
  const std::size_t value_pos = FindFieldValueStart(json, field_name);
  if (value_pos == std::string::npos) {
    return 0U;
  }
  return static_cast<std::uint64_t>(std::strtoull(json.c_str() + value_pos, nullptr, 10));
}

std::int32_t ExtractInt32Field(const std::string& json, const std::string& field_name) {
  const std::size_t value_pos = FindFieldValueStart(json, field_name);
  if (value_pos == std::string::npos) {
    return 0;
  }
  return static_cast<std::int32_t>(std::strtol(json.c_str() + value_pos, nullptr, 10));
}

bool ExtractBoolField(const std::string& json, const std::string& field_name) {
  const std::size_t value_pos = FindFieldValueStart(json, field_name);
  if (value_pos == std::string::npos) {
    return false;
  }
  return json.compare(value_pos, 4U, "true") == 0;
}

bool ExtractNullableUInt32Field(const std::string& json, const std::string& field_name,
                                std::uint32_t* value) {
  const std::size_t value_pos = FindFieldValueStart(json, field_name);
  if (value_pos == std::string::npos || json.compare(value_pos, 4U, "null") == 0) {
    return false;
  }
  *value = static_cast<std::uint32_t>(std::strtoul(json.c_str() + value_pos, nullptr, 10));
  return true;
}

bool ExtractNullableDoubleField(const std::string& json, const std::string& field_name,
                                double* value) {
  const std::size_t value_pos = FindFieldValueStart(json, field_name);
  if (value_pos == std::string::npos || json.compare(value_pos, 4U, "null") == 0) {
    return false;
  }
  *value = std::strtod(json.c_str() + value_pos, nullptr);
  return true;
}

std::string ExtractRawJsonValue(const std::string& json, const std::string& field_name) {
  const std::size_t value_pos = FindFieldValueStart(json, field_name);
  if (value_pos == std::string::npos || value_pos >= json.size()) {
    return "";
  }

  const char opening = json[value_pos];
  char closing = '\0';
  if (opening == '{') {
    closing = '}';
  } else if (opening == '[') {
    closing = ']';
  } else {
    std::size_t end = value_pos;
    while (end < json.size() && json[end] != ',') {
      ++end;
    }
    return json.substr(value_pos, end - value_pos);
  }

  int depth = 0;
  bool in_string = false;
  bool escaping = false;
  for (std::size_t i = value_pos; i < json.size(); ++i) {
    const char c = json[i];
    if (escaping) {
      escaping = false;
      continue;
    }
    if (c == '\\') {
      escaping = in_string;
      continue;
    }
    if (c == '"') {
      in_string = !in_string;
      continue;
    }
    if (in_string) {
      continue;
    }
    if (c == opening) {
      ++depth;
    } else if (c == closing) {
      --depth;
      if (depth == 0) {
        return json.substr(value_pos, i - value_pos + 1U);
      }
    }
  }
  return "";
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

    OpenEventChunk(0U, overwrite);

    const std::string cycle_index_path = JoinPath(JoinPath(trace_dir, "indexes"),
                                                 "cycles.idx");
    cycles_index.open(cycle_index_path.c_str(),
                      overwrite ? (std::ios::out | std::ios::trunc)
                                : (std::ios::out | std::ios::app));
    if (!cycles_index.is_open()) {
      throw std::runtime_error("failed to open replay cycle index: " + cycle_index_path);
    }
  }

  void OpenEventChunk(std::uint32_t chunk_index, bool truncate) {
    if (events.is_open()) {
      events.close();
    }

    const std::string event_path = EventChunkPath(trace_dir, chunk_index);
    events.open(event_path.c_str(),
                truncate ? (std::ios::out | std::ios::trunc)
                         : (std::ios::out | std::ios::app));
    if (!events.is_open()) {
      throw std::runtime_error("failed to open replay event file: " + event_path);
    }
    current_chunk_index = chunk_index;
  }

  void RotateEventChunkIfNeeded() {
    if (manifest.event_chunk_size == 0U || next_sequence == 0U) {
      return;
    }
    if ((next_sequence % manifest.event_chunk_size) == 0U) {
      OpenEventChunk(current_chunk_index + 1U, true);
    }
  }

  std::string trace_dir;
  ReplayTraceManifest manifest;
  std::ofstream events;
  std::ofstream cycles_index;
  std::uint64_t next_sequence{0U};
  std::uint32_t current_chunk_index{0U};
  std::string previous_event_hash{};
  std::deque<std::string> last_window;
};

ReplayTraceWriter::ReplayTraceWriter(std::string trace_dir, ReplayTraceManifest manifest,
                                     bool overwrite)
    : impl_(new Impl(std::move(trace_dir), std::move(manifest), overwrite)) {}

ReplayTraceWriter::~ReplayTraceWriter() = default;

void ReplayTraceWriter::WriteEvent(const ReplayTraceEvent& event) {
  impl_->RotateEventChunkIfNeeded();

  const std::string payload_hash = HashString(PayloadBytesForHash(event));
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
  if (event.payload_encoding == "json" || event.payload_bytes.empty()) {
    WriteJsonRawField(line, "payload", event.payload_json, true);
  } else {
    line << "\"payload\":null,";
    WriteJsonStringField(line, "payload_base64", Base64Encode(event.payload_bytes), true);
  }
  WriteJsonStringField(line, "payload_hash", payload_hash, true);
  WriteJsonStringField(line, "previous_event_hash", previous_hash, false);
  line << "}";

  const std::string serialized = line.str();
  const std::ostream::pos_type position = impl_->events.tellp();
  std::uint64_t byte_offset = 0U;
  if (position != std::ostream::pos_type(-1)) {
    byte_offset = static_cast<std::uint64_t>(position);
  }
  impl_->events << serialized << '\n';
  impl_->events.flush();
  WriteCycleIndexLine(impl_->cycles_index, event, impl_->next_sequence,
                      impl_->current_chunk_index, byte_offset);
  impl_->cycles_index.flush();
  impl_->previous_event_hash = HashString(serialized);
  ++impl_->next_sequence;

  const std::uint32_t window_limit = impl_->manifest.failure_window_event_count;
  if (window_limit > 0U) {
    while (impl_->last_window.size() >= window_limit) {
      impl_->last_window.pop_front();
    }
    impl_->last_window.push_back(serialized);
  }
}

void ReplayTraceWriter::WriteFailureMarker(const ReplayTraceFailure& failure) {
  const std::uint64_t failure_marker_sequence = impl_->next_sequence;
  const bool has_last_event_sequence = impl_->next_sequence > 0U;
  const std::uint64_t last_event_sequence =
      has_last_event_sequence ? (impl_->next_sequence - 1U) : 0U;

  ReplayTraceEvent event;
  event.module = impl_->manifest.module;
  event.event_type = "failure_marker";
  event.payload_type = "ReplayTraceFailure";
  event.payload_json = BuildFailurePayloadJson(failure, has_last_event_sequence,
                                               last_event_sequence);
  event.has_cycle_index = failure.has_cycle_index;
  event.cycle_index = failure.cycle_index;
  event.has_sim_time_sec = failure.has_sim_time_sec;
  event.sim_time_sec = failure.sim_time_sec;
  WriteEvent(event);

  const std::string crash_dir = JoinPath(impl_->trace_dir, "crash");
  WriteFailureFile(JoinPath(crash_dir, "failure.json"), impl_->manifest, failure,
                   failure_marker_sequence, has_last_event_sequence,
                   last_event_sequence);
  WriteLastWindowFile(JoinPath(crash_dir, "last-window.events.jsonl"),
                      impl_->last_window);
}

void ReplayTraceWriter::Flush() {
  impl_->events.flush();
  impl_->cycles_index.flush();
}

const std::string& ReplayTraceWriter::trace_dir() const { return impl_->trace_dir; }

const ReplayTraceManifest& ReplayTraceWriter::manifest() const { return impl_->manifest; }

struct ReplayTraceReader::Impl {
  explicit Impl(std::string path)
      : trace_dir(std::move(path)),
        manifest_json(ReadWholeFile(JoinPath(trace_dir, "manifest.json"))) {
    if (!OpenEventChunk(0U)) {
      throw std::runtime_error("failed to open replay event file: " +
                               EventChunkPath(trace_dir, 0U));
    }
  }

  bool OpenEventChunk(std::uint32_t chunk_index) {
    if (events.is_open()) {
      events.close();
    }

    const std::string event_path = EventChunkPath(trace_dir, chunk_index);
    events.open(event_path.c_str(), std::ios::in);
    if (!events.is_open()) {
      return false;
    }
    current_chunk_index = chunk_index;
    return true;
  }

  bool ReadNextLine(std::string* line) {
    bool keep_reading = true;
    while (keep_reading) {
      if (std::getline(events, *line)) {
        return true;
      }
      if (!OpenEventChunk(current_chunk_index + 1U)) {
        return false;
      }
    }
    return false;
  }

  std::string trace_dir;
  std::string manifest_json;
  std::ifstream events;
  std::uint32_t current_chunk_index{0U};
  std::string previous_event_hash;
};

ReplayTraceReader::ReplayTraceReader(std::string trace_dir)
    : impl_(new Impl(std::move(trace_dir))) {}

ReplayTraceReader::~ReplayTraceReader() = default;

const std::string& ReplayTraceReader::trace_dir() const { return impl_->trace_dir; }

const std::string& ReplayTraceReader::manifest_json() const { return impl_->manifest_json; }

bool ReplayTraceReader::ReadNextEvent(ReplayTraceReadEvent* event) {
  if (event == nullptr) {
    return false;
  }

  std::string line;
  if (!impl_->ReadNextLine(&line)) {
    return false;
  }

  ReplayTraceReadEvent parsed;
  parsed.raw_event_json = line;
  parsed.schema_version = ExtractInt32Field(line, "schema_version");
  parsed.trace_id = ExtractStringField(line, "trace_id");
  parsed.sequence = ExtractUInt64Field(line, "sequence");
  parsed.module = ExtractStringField(line, "module");
  parsed.event_type = ExtractStringField(line, "event_type");
  parsed.has_cycle_index = ExtractNullableUInt32Field(line, "cycle_index",
                                                      &parsed.cycle_index);
  parsed.has_sim_time_sec = ExtractNullableDoubleField(line, "sim_time_sec",
                                                       &parsed.sim_time_sec);
  parsed.payload_type = ExtractStringField(line, "payload_type");
  parsed.payload_encoding = ExtractStringField(line, "payload_encoding");
  parsed.payload_json = ExtractRawJsonValue(line, "payload");
  const std::string payload_base64 = ExtractStringField(line, "payload_base64");
  if (!payload_base64.empty()) {
    parsed.payload_bytes = Base64Decode(payload_base64);
  }
  parsed.payload_hash = ExtractStringField(line, "payload_hash");
  parsed.previous_event_hash = ExtractStringField(line, "previous_event_hash");
  parsed.payload_hash_matches =
      (HashString(PayloadBytesForHash(parsed)) == parsed.payload_hash);
  parsed.event_hash = HashString(line);
  parsed.previous_event_hash_matches =
      (parsed.previous_event_hash == impl_->previous_event_hash);
  impl_->previous_event_hash = parsed.event_hash;

  *event = parsed;
  return true;
}

ReplayTraceScanResult ScanReplayTrace(const std::string& trace_dir) {
  ReplayTraceReader reader(trace_dir);
  ReplayTraceScanResult result;

  std::uint64_t expected_sequence = 0U;
  ReplayTraceReadEvent event;
  while (reader.ReadNextEvent(&event)) {
    ++result.event_count;

    if (event.sequence != expected_sequence) {
      result.sequences_contiguous = false;
      if (result.first_error.empty()) {
        std::ostringstream message;
        message << "expected event sequence " << expected_sequence << " but found "
                << event.sequence;
        result.first_error = message.str();
      }
    }

    if (!event.payload_hash_matches) {
      result.payload_hashes_ok = false;
      if (result.first_error.empty()) {
        std::ostringstream message;
        message << "payload hash mismatch at sequence " << event.sequence;
        result.first_error = message.str();
      }
    }

    if (!event.previous_event_hash_matches) {
      result.event_chain_ok = false;
      if (result.first_error.empty()) {
        std::ostringstream message;
        message << "event chain mismatch at sequence " << event.sequence;
        result.first_error = message.str();
      }
    }

    expected_sequence = event.sequence + 1U;
  }

  result.ok = result.payload_hashes_ok && result.event_chain_ok &&
              result.sequences_contiguous;
  return result;
}

ReplayTraceCompatibilityResult CheckReplayTraceCompatibility(
    const std::string& trace_dir,
    const ReplayTraceCompatibilityExpectation& expectation) {
  ReplayTraceReader reader(trace_dir);
  const std::string& manifest_json = reader.manifest_json();

  ReplayTraceCompatibilityResult result;
  result.manifest_trace_id = ExtractStringField(manifest_json, "trace_id");
  result.manifest_module = ExtractStringField(manifest_json, "module");
  result.manifest_schema_version = ExtractInt32Field(manifest_json, "schema_version");
  result.manifest_serializer_version =
      ExtractStringField(manifest_json, "serializer_version");
  result.manifest_git_commit = ExtractStringField(manifest_json, "git_commit");
  result.manifest_git_dirty = ExtractBoolField(manifest_json, "git_dirty");

  result.schema_version_matches =
      (result.manifest_schema_version == expectation.schema_version);
  result.serializer_version_matches =
      (result.manifest_serializer_version == expectation.serializer_version);
  result.git_commit_matches =
      !expectation.require_git_commit_match ||
      expectation.git_commit.empty() ||
      (result.manifest_git_commit == expectation.git_commit);
  result.module_matches =
      !expectation.require_module_match ||
      expectation.module.empty() ||
      (result.manifest_module == expectation.module);

  if (!result.schema_version_matches && result.first_error.empty()) {
    std::ostringstream message;
    message << "replay schema mismatch: expected " << expectation.schema_version
            << " but found " << result.manifest_schema_version;
    result.first_error = message.str();
  }
  if (!result.serializer_version_matches && result.first_error.empty()) {
    result.first_error = "replay serializer version mismatch";
  }
  if (!result.git_commit_matches && result.first_error.empty()) {
    result.first_error = "replay git commit mismatch";
  }
  if (!result.module_matches && result.first_error.empty()) {
    result.first_error = "replay module mismatch";
  }

  if (result.manifest_git_dirty) {
    result.warning = "replay trace was captured from a dirty git worktree";
  }

  result.compatible = result.schema_version_matches &&
                      result.serializer_version_matches &&
                      result.git_commit_matches &&
                      result.module_matches;
  return result;
}

ReplayTraceReplayReport BuildReplayTraceReport(
    const std::string& trace_dir,
    const ReplayTraceCompatibilityExpectation& expectation) {
  ReplayTraceReplayReport report;
  report.compatibility = CheckReplayTraceCompatibility(trace_dir, expectation);
  report.scan = ScanReplayTrace(trace_dir);

  ReplayTraceReader reader(trace_dir);
  ReplayTraceReadEvent event;
  while (reader.ReadNextEvent(&event)) {
    if (event.event_type == "session_config") {
      ++report.session_config_count;
      report.has_session_config = true;
    } else if (event.event_type == "cycle_input") {
      ++report.cycle_input_count;
    } else if (event.event_type == "scene_state") {
      ++report.scene_state_count;
    } else if (event.event_type == "runtime_config_patch") {
      ++report.runtime_config_patch_count;
    } else if (event.event_type == "cycle_output") {
      ++report.cycle_output_count;
    } else if (event.event_type == "failure_marker") {
      ++report.failure_marker_count;
      if (!report.has_failure_marker) {
        report.has_failure_marker = true;
        report.first_failure_sequence = event.sequence;
        report.first_failure_payload_json = event.payload_json;
      }
    } else {
      ++report.unsupported_event_count;
    }
  }

  if (!report.compatibility.compatible && report.first_error.empty()) {
    report.first_error = report.compatibility.first_error;
  }
  if (!report.scan.ok && report.first_error.empty()) {
    report.first_error = report.scan.first_error;
  }
  if (!report.has_session_config && report.first_error.empty()) {
    report.first_error = "replay trace does not contain a session_config event";
  }

  if (!report.compatibility.warning.empty()) {
    report.warning = report.compatibility.warning;
  }
  if (report.unsupported_event_count > 0U && report.warning.empty()) {
    report.warning = "replay trace contains unsupported event types";
  }

  report.replay_ready = report.compatibility.compatible &&
                        report.scan.ok &&
                        report.has_session_config &&
                        report.unsupported_event_count == 0U;
  return report;
}

void WriteReplayTraceReport(const ReplayTraceReplayReport& report,
                            const std::string& report_path) {
  WriteReportFile(report_path, report);
}

ReplayTracePlaybackResult PlaybackReplayTrace(
    const std::string& trace_dir,
    const ReplayTracePlaybackCallbacks& callbacks,
    const ReplayTracePlaybackOptions& options) {
  ReplayTracePlaybackResult result;
  ReplayTraceReader reader(trace_dir);

  ReplayTraceReadEvent event;
  while (reader.ReadNextEvent(&event)) {
    ++result.processed_event_count;

    std::string callback_error;
    bool callback_ok = true;
    if (event.event_type == "session_config") {
      callback_ok = InvokeReplayCallback(
          callbacks.on_session_config, event, callbacks.user_data,
          "missing session_config replay callback", true, &callback_error);
    } else if (event.event_type == "cycle_input") {
      callback_ok = InvokeReplayCallback(
          callbacks.on_cycle_input, event, callbacks.user_data,
          "missing cycle_input replay callback", true, &callback_error);
      if (callback_ok) {
        ++result.applied_input_count;
      }
    } else if (event.event_type == "scene_state") {
      callback_ok = InvokeReplayCallback(
          callbacks.on_scene_state, event, callbacks.user_data,
          "missing scene_state replay callback", false, &callback_error);
      if (callback_ok && callbacks.on_scene_state) {
        ++result.applied_scene_state_count;
      }
    } else if (event.event_type == "runtime_config_patch") {
      callback_ok = InvokeReplayCallback(
          callbacks.on_runtime_config_patch, event, callbacks.user_data,
          "missing runtime_config_patch replay callback", true, &callback_error);
      if (callback_ok) {
        ++result.applied_runtime_patch_count;
      }
    } else if (event.event_type == "cycle_output") {
      if (!callbacks.on_cycle_output) {
        if (options.require_output_callback) {
          callback_ok = false;
          callback_error = "missing cycle_output replay callback";
        } else {
          ++result.skipped_output_count;
        }
      } else {
        std::string actual_output_json;
        callback_ok = callbacks.on_cycle_output(event, callbacks.user_data,
                                                &actual_output_json,
                                                &callback_error);
        if (callback_ok) {
          ++result.compared_output_count;
          if (actual_output_json != event.payload_json) {
            result.divergence_found = true;
            result.divergence_sequence = event.sequence;
            result.expected_output_json = event.payload_json;
            result.actual_output_json = actual_output_json;
            result.ok = false;
            if (result.first_error.empty()) {
              result.first_error = "replay output divergence";
            }
            if (options.stop_on_first_divergence) {
              return result;
            }
          }
        }
      }
    } else if (event.event_type == "failure_marker") {
      ++result.failure_marker_count;
      callback_ok = InvokeReplayCallback(
          callbacks.on_failure_marker, event, callbacks.user_data,
          "missing failure_marker replay callback", false, &callback_error);
      if (callback_ok && options.stop_on_failure_marker) {
        return result;
      }
    }

    if (!callback_ok) {
      result.ok = false;
      if (result.first_error.empty()) {
        result.first_error = callback_error.empty() ? "replay callback failed"
                                                    : callback_error;
      }
      return result;
    }
  }

  return result;
}

}  // namespace replay
}  // namespace oneq
