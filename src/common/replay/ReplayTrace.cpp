#include "1q/replay/ReplayTrace.h"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#if ONEQ_HAVE_ZLIB
#include <zlib.h>
#endif

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include "common/logging/ProjectLog.h"
#include "common/trace/JsonFormatUtils.h"
#include "common/trace/TimeUtils.h"

namespace oneq {
namespace replay {
namespace {

constexpr std::uint64_t kMaxReplayTraceFileBytes = 0xFFFFFFFFull;

bool IsPathSeparator(char value) { return value == '/' || value == '\\'; }

bool CreateDirectoryIfMissing(const std::string& path, std::string* error) {
  if (path.empty()) {
    return true;
  }

#if defined(_WIN32)
  const int result = _mkdir(path.c_str());
#else
  const int result = mkdir(path.c_str(), 0777);
#endif
  if (result != 0 && errno != EEXIST) {
    if (error != nullptr) {
      *error = "failed to create replay trace directory: " + path;
    }
    return false;
  }
  return true;
}

bool CreateDirectoryRecursive(const std::string& path, std::string* error) {
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
      if (!CreateDirectoryIfMissing(current, error)) {
        return false;
      }
    }
  }
  return CreateDirectoryIfMissing(path, error);
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

std::string EventChunkGzFileName(std::uint32_t chunk_index) {
  return EventChunkFileName(chunk_index) + ".gz";
}

std::string EventChunkRelativePath(std::uint32_t chunk_index) {
  return JoinPath("events", EventChunkFileName(chunk_index));
}

std::string EventChunkGzRelativePath(std::uint32_t chunk_index) {
  return JoinPath("events", EventChunkGzFileName(chunk_index));
}

std::string EventChunkPath(const std::string& trace_dir, std::uint32_t chunk_index) {
  return JoinPath(trace_dir, EventChunkRelativePath(chunk_index));
}

std::string EventChunkGzPath(const std::string& trace_dir, std::uint32_t chunk_index) {
  return JoinPath(trace_dir, EventChunkGzRelativePath(chunk_index));
}

bool FileExists(const std::string& path) {
  std::ifstream f(path.c_str());
  return f.good();
}

bool HasExistingReplayTraceArtifacts(const std::string& trace_dir) {
  return FileExists(JoinPath(trace_dir, "manifest.json")) ||
         FileExists(EventChunkPath(trace_dir, 0U)) ||
         FileExists(EventChunkGzPath(trace_dir, 0U)) ||
         FileExists(JoinPath(JoinPath(trace_dir, "indexes"), "cycles.idx"));
}

#if ONEQ_HAVE_ZLIB
// 将 src_path 内容压缩写入 dst_gz_path，成功后删除 src_path。
// 返回 false 表示压缩失败，error 写入原因。
bool GzipCompressFile(const std::string& src_path, const std::string& dst_gz_path,
                      std::string* error) {
  // 读取原文件
  std::ifstream src(src_path.c_str(), std::ios::in | std::ios::binary);
  if (!src.is_open()) {
    if (error != nullptr) {
      *error = "gzip: failed to open source: " + src_path;
    }
    return false;
  }
  const std::string contents((std::istreambuf_iterator<char>(src)),
                             std::istreambuf_iterator<char>());
  src.close();

  // 写入 .gz
  gzFile gz = gzopen(dst_gz_path.c_str(), "wb");
  if (gz == Z_NULL) {
    if (error != nullptr) {
      *error = "gzip: failed to create gz file: " + dst_gz_path;
    }
    return false;
  }
  if (!contents.empty()) {
    const int written = gzwrite(gz, contents.data(), static_cast<unsigned int>(contents.size()));
    if (written <= 0) {
      gzclose(gz);
      if (error != nullptr) {
        *error = "gzip: write failed: " + dst_gz_path;
      }
      return false;
    }
  }
  gzclose(gz);

  // 删除原始文件
  std::remove(src_path.c_str());
  return true;
}

// 从 gzFile 中读取一行（不含换行符）。返回 false 表示 EOF 或错误。
bool GzipReadLine(gzFile gz, std::string* line) {
  line->clear();
  char buf[4096];
  bool got_any = false;
  while (true) {
    const char* result = gzgets(gz, buf, static_cast<int>(sizeof(buf)));
    if (result == Z_NULL) {
      break;
    }
    got_any = true;
    const std::string chunk(result);
    if (!chunk.empty() && chunk[chunk.size() - 1U] == '\n') {
      line->append(chunk, 0U, chunk.size() - 1U);
      return true;
    }
    *line += chunk;
  }
  return got_any && !line->empty();
}
#endif  // ONEQ_HAVE_ZLIB

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
    accumulator = (accumulator << 8) | static_cast<unsigned char>(bytes[i]);
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
  return event.payload_bytes;
}

const std::string& PayloadBytesForHash(const ReplayTraceReadEvent& event) {
  return event.payload_bytes;
}

void WriteJsonStringField(std::ostream& output, const char* name, const std::string& value,
                          bool trailing_comma) {
  output << "\"" << name << "\":" << oneq::common::trace::QuoteString(value);
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

bool WriteManifestFile(const std::string& path, const ReplayTraceManifest& manifest,
                       std::string* error) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    if (error != nullptr) {
      *error = "failed to open replay manifest: " + path;
    }
    return false;
  }

  output << "{";
  WriteJsonStringField(output, "trace_id", manifest.trace_id, true);
  output << "\"created_wall_time_ms\":" << oneq::common::trace::CurrentTimestampMs() << ",";
  WriteJsonStringField(output, "module", manifest.module, true);
  WriteJsonStringField(output, "scenario_id", manifest.scenario_id, true);
  output << "\"schema_version\":" << manifest.schema_version << ",";
  WriteJsonStringField(output, "serializer_version", manifest.serializer_version, true);
  WriteJsonStringField(output, "git_commit", manifest.git_commit, true);
  output << "\"git_dirty\":" << oneq::common::trace::BoolToJson(manifest.git_dirty) << ",";
  WriteJsonStringField(output, "build_type", manifest.build_type, true);
  WriteJsonStringField(output, "compiler", manifest.compiler, true);
  WriteJsonStringField(output, "compiler_version", manifest.compiler_version, true);
  WriteJsonStringField(output, "platform", manifest.platform, true);
  WriteJsonStringField(output, "cpu_arch", manifest.cpu_arch, true);
  WriteJsonStringField(output, "library_version", manifest.library_version, true);
  WriteJsonRawField(output, "dependency_versions", manifest.dependency_versions_payload, true);
  WriteJsonStringField(output, "float_policy", manifest.float_policy, true);
  WriteJsonRawField(output, "default_tolerances", manifest.default_tolerances_payload, true);
  output << "\"compress_closed_chunks\":"
         << oneq::common::trace::BoolToJson(manifest.compress_closed_chunks) << ",";
  output << "\"checkpoint_interval_cycles\":" << manifest.checkpoint_interval_cycles << ",";
  output << "\"event_chunk_size\":" << manifest.event_chunk_size << ",";
  output << "\"failure_window_event_count\":" << manifest.failure_window_event_count;
  output << "}\n";
  if (output.fail() || output.bad()) {
    if (error != nullptr) {
      *error = "failed to write replay manifest: " + path;
    }
    return false;
  }
  return true;
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

void WriteOptionalDoubleField(std::ostream& output, const char* name, bool has_value, double value,
                              bool trailing_comma) {
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

bool WriteFailureFile(const std::string& path, const ReplayTraceManifest& manifest,
                      const ReplayTraceFailure& failure, std::uint64_t failure_marker_sequence,
                      bool has_last_event_sequence, std::uint64_t last_event_sequence,
                      std::string* error) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    if (error != nullptr) {
      *error = "failed to open replay failure file: " + path;
    }
    return false;
  }

  output << "{";
  output << "\"schema_version\":" << manifest.schema_version << ",";
  WriteJsonStringField(output, "trace_id", manifest.trace_id, true);
  WriteJsonStringField(output, "module", manifest.module, true);
  output << "\"created_wall_time_ms\":" << oneq::common::trace::CurrentTimestampMs() << ",";
  output << "\"failure_marker_sequence\":" << failure_marker_sequence << ",";
  WriteOptionalUInt64Field(output, "last_event_sequence", has_last_event_sequence,
                           last_event_sequence, true);
  WriteJsonStringField(output, "error_code", failure.error_code, true);
  WriteJsonStringField(output, "message", failure.message, true);
  WriteJsonStringField(output, "location", failure.location, true);
  WriteOptionalUInt32Field(output, "cycle_index", failure.has_cycle_index, failure.cycle_index,
                           true);
  WriteOptionalDoubleField(output, "sim_time_sec", failure.has_sim_time_sec, failure.sim_time_sec,
                           true);
  WriteJsonRawField(output, "diagnostics", failure.diagnostics_payload, false);
  output << "}\n";
  if (output.fail() || output.bad()) {
    if (error != nullptr) {
      *error = "failed to write replay failure file: " + path;
    }
    return false;
  }
  return true;
}

bool WriteLastWindowFile(const std::string& path, const std::deque<std::string>& last_window,
                         std::string* error) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    if (error != nullptr) {
      *error = "failed to open replay last-window file: " + path;
    }
    return false;
  }

  for (std::deque<std::string>::const_iterator it = last_window.begin(); it != last_window.end();
       ++it) {
    output << *it << '\n';
  }
  if (output.fail() || output.bad()) {
    if (error != nullptr) {
      *error = "failed to write replay last-window file: " + path;
    }
    return false;
  }
  return true;
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
  WriteOptionalDoubleField(output, "sim_time_sec", event.has_sim_time_sec, event.sim_time_sec,
                           false);
  output << "}\n";
}

bool WriteReportFile(const std::string& path, const ReplayTraceReplayReport& report,
                     std::string* error) {
  std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    if (error != nullptr) {
      *error = "failed to open replay report file: " + path;
    }
    return false;
  }

  output << "{";
  output << "\"replay_ready\":" << oneq::common::trace::BoolToJson(report.replay_ready) << ",";
  WriteJsonStringField(output, "first_error", report.first_error, true);
  WriteJsonStringField(output, "warning", report.warning, true);

  output << "\"compatibility\":{";
  output << "\"compatible\":" << oneq::common::trace::BoolToJson(report.compatibility.compatible)
         << ",";
  output << "\"schema_version_matches\":"
         << oneq::common::trace::BoolToJson(report.compatibility.schema_version_matches) << ",";
  output << "\"serializer_version_matches\":"
         << oneq::common::trace::BoolToJson(report.compatibility.serializer_version_matches) << ",";
  output << "\"git_commit_matches\":"
         << oneq::common::trace::BoolToJson(report.compatibility.git_commit_matches) << ",";
  output << "\"module_matches\":"
         << oneq::common::trace::BoolToJson(report.compatibility.module_matches) << ",";
  output << "\"manifest_git_dirty\":"
         << oneq::common::trace::BoolToJson(report.compatibility.manifest_git_dirty) << ",";
  WriteJsonStringField(output, "manifest_trace_id", report.compatibility.manifest_trace_id, true);
  WriteJsonStringField(output, "manifest_module", report.compatibility.manifest_module, true);
  output << "\"manifest_schema_version\":" << report.compatibility.manifest_schema_version << ",";
  WriteJsonStringField(output, "manifest_serializer_version",
                       report.compatibility.manifest_serializer_version, true);
  WriteJsonStringField(output, "manifest_git_commit", report.compatibility.manifest_git_commit,
                       true);
  WriteJsonStringField(output, "first_error", report.compatibility.first_error, true);
  WriteJsonStringField(output, "warning", report.compatibility.warning, false);
  output << "},";

  output << "\"scan\":{";
  output << "\"ok\":" << oneq::common::trace::BoolToJson(report.scan.ok) << ",";
  output << "\"event_count\":" << report.scan.event_count << ",";
  output << "\"payload_hashes_ok\":" << oneq::common::trace::BoolToJson(report.scan.payload_hashes_ok)
         << ",";
  output << "\"event_chain_ok\":" << oneq::common::trace::BoolToJson(report.scan.event_chain_ok) << ",";
  output << "\"sequences_contiguous\":"
         << oneq::common::trace::BoolToJson(report.scan.sequences_contiguous) << ",";
  WriteJsonStringField(output, "first_error", report.scan.first_error, false);
  output << "},";

  output << "\"events\":{";
  output << "\"has_session_config\":" << oneq::common::trace::BoolToJson(report.has_session_config)
         << ",";
  output << "\"has_failure_marker\":" << oneq::common::trace::BoolToJson(report.has_failure_marker)
         << ",";
  output << "\"session_config_count\":" << report.session_config_count << ",";
  output << "\"cycle_input_count\":" << report.cycle_input_count << ",";
  output << "\"scene_state_count\":" << report.scene_state_count << ",";
  output << "\"runtime_config_patch_count\":" << report.runtime_config_patch_count << ",";
  output << "\"cycle_output_count\":" << report.cycle_output_count << ",";
  output << "\"failure_marker_count\":" << report.failure_marker_count << ",";
  output << "\"warning_event_count\":" << report.warning_event_count << ",";
  output << "\"unsupported_event_count\":" << report.unsupported_event_count << ",";
  output << "\"first_failure_sequence\":" << report.first_failure_sequence << ",";
  WriteJsonStringField(output, "first_failure_payload_base64", report.first_failure_payload_base64,
                       true);
  WriteJsonStringField(output, "first_failure_payload_encoding",
                       report.first_failure_payload_encoding, true);
  WriteJsonStringField(output, "first_failure_payload_type", report.first_failure_payload_type,
                       false);
  output << "}";
  output << "}\n";
  if (output.fail() || output.bad()) {
    if (error != nullptr) {
      *error = "failed to write replay report file: " + path;
    }
    return false;
  }
  return true;
}

bool InvokeReplayCallback(ReplayTraceEventCallback callback, const ReplayTraceReadEvent& event,
                          void* user_data, const char* missing_callback_error, bool required,
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

bool ReadWholeFile(const std::string& path, std::string* content, std::string* error) {
  std::ifstream input(path.c_str(), std::ios::in);
  if (!input.is_open()) {
    if (error != nullptr) {
      *error = "failed to open replay trace file: " + path;
    }
    return false;
  }
  input.seekg(0, std::ios::end);
  const std::ifstream::pos_type size = input.tellg();
  if (size != std::ifstream::pos_type(-1) &&
      static_cast<std::uint64_t>(size) > kMaxReplayTraceFileBytes) {
    if (error != nullptr) {
      *error = "replay trace file is too large: " + path;
    }
    return false;
  }
  input.seekg(0, std::ios::beg);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (input.bad()) {
    if (error != nullptr) {
      *error = "failed to read replay trace file: " + path;
    }
    return false;
  }
  if (content != nullptr) {
    *content = buffer.str();
  }
  return true;
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
    if (!overwrite && HasExistingReplayTraceArtifacts(trace_dir)) {
      first_error = "replay trace already exists and overwrite is disabled: " + trace_dir;
      writable = false;
      PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
      return;
    }
    if (!CreateDirectoryRecursive(trace_dir, &first_error) ||
        !CreateDirectoryRecursive(JoinPath(trace_dir, "events"), &first_error) ||
        !CreateDirectoryRecursive(JoinPath(trace_dir, "checkpoints"), &first_error) ||
        !CreateDirectoryRecursive(JoinPath(trace_dir, "crash"), &first_error) ||
        !CreateDirectoryRecursive(JoinPath(trace_dir, "indexes"), &first_error)) {
      writable = false;
      PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
      return;
    }

    if (!WriteManifestFile(JoinPath(trace_dir, "manifest.json"), manifest, &first_error)) {
      writable = false;
      PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
      return;
    }

    if (!OpenEventChunk(0U, overwrite)) {
      writable = false;
      PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
      return;
    }

    const std::string cycle_index_path = JoinPath(JoinPath(trace_dir, "indexes"), "cycles.idx");
    cycles_index.open(cycle_index_path.c_str(), overwrite ? (std::ios::out | std::ios::trunc)
                                                          : (std::ios::out | std::ios::app));
    if (!cycles_index.is_open()) {
      first_error = "failed to open replay cycle index: " + cycle_index_path;
      writable = false;
      PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
    }
  }

  bool OpenEventChunk(std::uint32_t chunk_index, bool truncate) {
    if (events.is_open()) {
      events.close();
    }

    const std::string event_path = EventChunkPath(trace_dir, chunk_index);
    events.open(event_path.c_str(),
                truncate ? (std::ios::out | std::ios::trunc) : (std::ios::out | std::ios::app));
    if (!events.is_open()) {
      first_error = "failed to open replay event file: " + event_path;
      writable = false;
      return false;
    }
    current_chunk_index = chunk_index;
    return true;
  }

  bool RotateEventChunkIfNeeded() {
    if (!writable || manifest.event_chunk_size == 0U || next_sequence == 0U) {
      return writable;
    }
    if ((next_sequence % manifest.event_chunk_size) != 0U) {
      return true;
    }

    // Seal the current chunk before opening the next one.
    const std::uint32_t sealed_index = current_chunk_index;
    if (!OpenEventChunk(current_chunk_index + 1U, true)) {
      PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
      return false;
    }
#if ONEQ_HAVE_ZLIB
    if (manifest.compress_closed_chunks) {
      const std::string plain_path = EventChunkPath(trace_dir, sealed_index);
      const std::string gz_path = EventChunkGzPath(trace_dir, sealed_index);
      if (!GzipCompressFile(plain_path, gz_path, &first_error)) {
        writable = false;
        PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
        return false;
      }
    }
#endif
    return true;
  }

  bool CheckWritable() {
    if (!writable) {
      if (!first_error.empty()) {
        PROJECT_LOG_ERROR("ReplayTraceWriter is not writable: {}", first_error);
      }
      return false;
    }
    if (!events.is_open() || !cycles_index.is_open()) {
      first_error = "replay trace output stream is not open";
      writable = false;
      PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
      return false;
    }
    return true;
  }

  bool MarkWriteFailure(const std::string& message) {
    first_error = message;
    writable = false;
    PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", first_error);
    return false;
  }

  bool WriteLine(const std::string& serialized, const ReplayTraceEvent& event) {
    if (!CheckWritable()) {
      return false;
    }
    const std::ostream::pos_type position = events.tellp();
    std::uint64_t byte_offset = 0U;
    if (position != std::ostream::pos_type(-1)) {
      byte_offset = static_cast<std::uint64_t>(position);
    }
    events << serialized << '\n';
    events.flush();
    if (events.fail() || events.bad()) {
      return MarkWriteFailure("failed to write replay event file");
    }
    WriteCycleIndexLine(cycles_index, event, next_sequence, current_chunk_index, byte_offset);
    cycles_index.flush();
    if (cycles_index.fail() || cycles_index.bad()) {
      return MarkWriteFailure("failed to write replay cycle index");
    }
    return true;
  }

  bool Flush() {
    if (!CheckWritable()) {
      return false;
    }
    events.flush();
    cycles_index.flush();
    if (events.fail() || events.bad() || cycles_index.fail() || cycles_index.bad()) {
      return MarkWriteFailure("failed to flush replay trace streams");
    }
    return true;
  }

  void RememberWindow(const std::string& serialized) {
    const std::uint32_t window_limit = manifest.failure_window_event_count;
    if (window_limit == 0U) {
      return;
    }
    while (last_window.size() >= window_limit) {
      last_window.pop_front();
    }
    last_window.push_back(serialized);
  }

  std::string trace_dir;
  ReplayTraceManifest manifest;
  std::ofstream events;
  std::ofstream cycles_index;
  bool writable{true};
  std::string first_error{};
  std::uint64_t next_sequence{0U};
  std::uint32_t current_chunk_index{0U};
  std::string previous_event_hash{};
  std::deque<std::string> last_window;
};

ReplayTraceWriter::ReplayTraceWriter(std::string trace_dir, ReplayTraceManifest manifest,
                                     bool overwrite)
    : impl_(new Impl(std::move(trace_dir), std::move(manifest), overwrite)) {}

ReplayTraceWriter::~ReplayTraceWriter() = default;

ReplayTraceWriteStatus ReplayTraceWriter::WriteEvent(const ReplayTraceEvent& event) {
  if (!impl_->RotateEventChunkIfNeeded()) {
    return ReplayTraceWriteStatus::kError;
  }
  if (!impl_->CheckWritable()) {
    return ReplayTraceWriteStatus::kError;
  }

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
  line << "\"wall_time_ms\":" << oneq::common::trace::CurrentTimestampMs() << ",";
  WriteJsonStringField(line, "payload_type", event.payload_type, true);
  WriteJsonStringField(line, "payload_encoding", event.payload_encoding, true);
  line << "\"payload\":null,";
  WriteJsonStringField(line, "payload_base64", Base64Encode(event.payload_bytes), true);
  WriteJsonStringField(line, "payload_hash", payload_hash, true);
  WriteJsonStringField(line, "previous_event_hash", previous_hash, false);
  line << "}";

  const std::string serialized = line.str();
  if (!impl_->WriteLine(serialized, event)) {
    return ReplayTraceWriteStatus::kError;
  }
  impl_->previous_event_hash = HashString(serialized);
  ++impl_->next_sequence;
  impl_->RememberWindow(serialized);
  return ReplayTraceWriteStatus::kSuccess;
}

ReplayTraceWriteStatus ReplayTraceWriter::WriteFailureMarker(
    const ReplayTraceFailure& failure) {
  return WriteFailureMarker(failure, "");
}

ReplayTraceWriteStatus ReplayTraceWriter::WriteFailureMarker(
    const ReplayTraceFailure& failure, const std::string& payload_bytes) {
  const std::uint64_t failure_marker_sequence = impl_->next_sequence;
  const bool has_last_event_sequence = impl_->next_sequence > 0U;
  const std::uint64_t last_event_sequence =
      has_last_event_sequence ? (impl_->next_sequence - 1U) : 0U;

  ReplayTraceEvent event;
  event.module = impl_->manifest.module;
  event.event_type = "failure_marker";
  event.payload_type = "ReplayTraceFailure";
  event.payload_encoding = "flatbuffers";
  event.payload_bytes = payload_bytes;
  event.has_cycle_index = failure.has_cycle_index;
  event.cycle_index = failure.cycle_index;
  event.has_sim_time_sec = failure.has_sim_time_sec;
  event.sim_time_sec = failure.sim_time_sec;
  if (WriteEvent(event) == ReplayTraceWriteStatus::kError) {
    return ReplayTraceWriteStatus::kError;
  }

  const std::string crash_dir = JoinPath(impl_->trace_dir, "crash");
  if (!impl_->writable) {
    return ReplayTraceWriteStatus::kError;
  }
  if (!WriteFailureFile(JoinPath(crash_dir, "failure.json"), impl_->manifest, failure,
                        failure_marker_sequence, has_last_event_sequence, last_event_sequence,
                        &impl_->first_error)) {
    impl_->writable = false;
    PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", impl_->first_error);
    return ReplayTraceWriteStatus::kError;
  }
  if (!WriteLastWindowFile(JoinPath(crash_dir, "last-window.events.jsonl"), impl_->last_window,
                           &impl_->first_error)) {
    impl_->writable = false;
    PROJECT_LOG_ERROR("ReplayTraceWriter disabled: {}", impl_->first_error);
    return ReplayTraceWriteStatus::kError;
  }
  return ReplayTraceWriteStatus::kSuccess;
}

ReplayTraceWriteStatus ReplayTraceWriter::Flush() {
  return impl_->Flush() ? ReplayTraceWriteStatus::kSuccess : ReplayTraceWriteStatus::kError;
}

bool ReplayTraceWriter::ok() const { return impl_->writable; }

const std::string& ReplayTraceWriter::first_error() const { return impl_->first_error; }

const std::string& ReplayTraceWriter::trace_dir() const { return impl_->trace_dir; }

const ReplayTraceManifest& ReplayTraceWriter::manifest() const { return impl_->manifest; }

struct ReplayTraceReader::Impl {
  explicit Impl(std::string path) : trace_dir(std::move(path)) {
    if (!ReadWholeFile(JoinPath(trace_dir, "manifest.json"), &manifest_json, &first_error)) {
      readable = false;
      PROJECT_LOG_ERROR("ReplayTraceReader disabled: {}", first_error);
      return;
    }
    if (!OpenEventChunk(0U)) {
      first_error = "failed to open replay event file: " + EventChunkPath(trace_dir, 0U);
      readable = false;
      PROJECT_LOG_ERROR("ReplayTraceReader disabled: {}", first_error);
    }
  }

  ~Impl() {
#if ONEQ_HAVE_ZLIB
    if (gz_events != Z_NULL) {
      gzclose(gz_events);
      gz_events = Z_NULL;
    }
#endif
  }

  bool OpenEventChunk(std::uint32_t chunk_index) {
    if (events.is_open()) {
      events.close();
    }
#if ONEQ_HAVE_ZLIB
    if (gz_events != Z_NULL) {
      gzclose(gz_events);
      gz_events = Z_NULL;
    }
    // Prefer compressed chunk if present.
    const std::string gz_path = EventChunkGzPath(trace_dir, chunk_index);
    if (FileExists(gz_path)) {
      gz_events = gzopen(gz_path.c_str(), "rb");
      if (gz_events != Z_NULL) {
        current_chunk_index = chunk_index;
        return true;
      }
    }
#endif
    const std::string event_path = EventChunkPath(trace_dir, chunk_index);
    events.open(event_path.c_str(), std::ios::in);
    if (!events.is_open()) {
      return false;
    }
    current_chunk_index = chunk_index;
    return true;
  }

  ReplayTraceReadStatus MarkReadFailure(const std::string& message) {
    if (first_error.empty()) {
      first_error = message;
    }
    readable = false;
    PROJECT_LOG_ERROR("ReplayTraceReader disabled: {}", first_error);
    return ReplayTraceReadStatus::kError;
  }

  ReplayTraceReadStatus ReadNextLine(std::string* line) {
    if (!readable) {
      return ReplayTraceReadStatus::kError;
    }
    while (true) {
#if ONEQ_HAVE_ZLIB
      if (gz_events != Z_NULL) {
        if (GzipReadLine(gz_events, line)) {
          return ReplayTraceReadStatus::kEvent;
        }
        int gzip_error = Z_OK;
        const char* gzip_message = gzerror(gz_events, &gzip_error);
        if (gzip_error != Z_OK && gzip_error != Z_STREAM_END) {
          return MarkReadFailure(std::string("failed to read replay gzip event file: ") +
                                 (gzip_message == nullptr ? "unknown gzip error" : gzip_message));
        }
        if (!OpenEventChunk(current_chunk_index + 1U)) {
          return ReplayTraceReadStatus::kEndOfTrace;
        }
        continue;
      }
#endif
      if (std::getline(events, *line)) {
        return ReplayTraceReadStatus::kEvent;
      }
      if (events.bad() || (events.fail() && !events.eof())) {
        return MarkReadFailure("failed to read replay event file: " +
                               EventChunkPath(trace_dir, current_chunk_index));
      }
      if (!OpenEventChunk(current_chunk_index + 1U)) {
        return ReplayTraceReadStatus::kEndOfTrace;
      }
    }
  }

  std::string trace_dir;
  std::string manifest_json;
  bool readable{true};
  std::string first_error{};
  std::ifstream events;
#if ONEQ_HAVE_ZLIB
  gzFile gz_events{Z_NULL};
#endif
  std::uint32_t current_chunk_index{0U};
  std::string previous_event_hash;
};

ReplayTraceReader::ReplayTraceReader(std::string trace_dir)
    : impl_(new Impl(std::move(trace_dir))) {}

ReplayTraceReader::~ReplayTraceReader() = default;

const std::string& ReplayTraceReader::trace_dir() const { return impl_->trace_dir; }

const std::string& ReplayTraceReader::manifest_json() const { return impl_->manifest_json; }

ReplayTraceReadStatus ReplayTraceReader::ReadNextEvent(ReplayTraceReadEvent* event) {
  if (event == nullptr) {
    return impl_->MarkReadFailure("replay trace event output is null");
  }

  std::string line;
  const ReplayTraceReadStatus status = impl_->ReadNextLine(&line);
  if (status != ReplayTraceReadStatus::kEvent) {
    return status;
  }

  ReplayTraceReadEvent parsed;
  parsed.raw_event_json = line;
  parsed.schema_version = ExtractInt32Field(line, "schema_version");
  parsed.trace_id = ExtractStringField(line, "trace_id");
  parsed.sequence = ExtractUInt64Field(line, "sequence");
  parsed.module = ExtractStringField(line, "module");
  parsed.event_type = ExtractStringField(line, "event_type");
  parsed.has_cycle_index = ExtractNullableUInt32Field(line, "cycle_index", &parsed.cycle_index);
  parsed.has_sim_time_sec = ExtractNullableDoubleField(line, "sim_time_sec", &parsed.sim_time_sec);
  parsed.payload_type = ExtractStringField(line, "payload_type");
  parsed.payload_encoding = ExtractStringField(line, "payload_encoding");
  parsed.payload_inline = ExtractRawJsonValue(line, "payload");
  const std::string payload_base64 = ExtractStringField(line, "payload_base64");
  if (!payload_base64.empty()) {
    parsed.payload_bytes = Base64Decode(payload_base64);
  }
  parsed.payload_hash = ExtractStringField(line, "payload_hash");
  parsed.previous_event_hash = ExtractStringField(line, "previous_event_hash");
  parsed.payload_hash_matches = (HashString(PayloadBytesForHash(parsed)) == parsed.payload_hash);
  parsed.event_hash = HashString(line);
  parsed.previous_event_hash_matches = (parsed.previous_event_hash == impl_->previous_event_hash);
  impl_->previous_event_hash = parsed.event_hash;

  *event = parsed;
  return ReplayTraceReadStatus::kEvent;
}

bool ReplayTraceReader::ok() const { return impl_->readable; }

const std::string& ReplayTraceReader::first_error() const { return impl_->first_error; }

ReplayTraceScanResult ScanReplayTrace(const std::string& trace_dir) {
  ReplayTraceReader reader(trace_dir);
  ReplayTraceScanResult result;

  std::uint64_t expected_sequence = 0U;
  ReplayTraceReadEvent event;
  ReplayTraceReadStatus read_status = ReplayTraceReadStatus::kEndOfTrace;
  while ((read_status = reader.ReadNextEvent(&event)) == ReplayTraceReadStatus::kEvent) {
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

  if (read_status == ReplayTraceReadStatus::kError) {
    result.first_error = reader.first_error();
  }

  result.ok = read_status != ReplayTraceReadStatus::kError && result.payload_hashes_ok &&
              result.event_chain_ok && result.sequences_contiguous;
  return result;
}

ReplayTraceCompatibilityResult CheckReplayTraceCompatibility(
    const std::string& trace_dir, const ReplayTraceCompatibilityExpectation& expectation) {
  ReplayTraceReader reader(trace_dir);
  const std::string& manifest_json = reader.manifest_json();

  ReplayTraceCompatibilityResult result;
  result.manifest_trace_id = ExtractStringField(manifest_json, "trace_id");
  result.manifest_module = ExtractStringField(manifest_json, "module");
  result.manifest_schema_version = ExtractInt32Field(manifest_json, "schema_version");
  result.manifest_serializer_version = ExtractStringField(manifest_json, "serializer_version");
  result.manifest_git_commit = ExtractStringField(manifest_json, "git_commit");
  result.manifest_git_dirty = ExtractBoolField(manifest_json, "git_dirty");

  result.schema_version_matches = (result.manifest_schema_version == expectation.schema_version);
  result.serializer_version_matches =
      (result.manifest_serializer_version == expectation.serializer_version);
  result.git_commit_matches = !expectation.require_git_commit_match ||
                              expectation.git_commit.empty() ||
                              (result.manifest_git_commit == expectation.git_commit);
  result.module_matches = !expectation.require_module_match || expectation.module.empty() ||
                          (result.manifest_module == expectation.module);

  if (!result.schema_version_matches && result.first_error.empty()) {
    std::ostringstream message;
    message << "replay schema mismatch: expected " << expectation.schema_version << " but found "
            << result.manifest_schema_version;
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

  result.compatible = result.schema_version_matches && result.serializer_version_matches &&
                      result.git_commit_matches && result.module_matches;
  return result;
}

ReplayTraceReplayReport BuildReplayTraceReport(
    const std::string& trace_dir, const ReplayTraceCompatibilityExpectation& expectation) {
  ReplayTraceReplayReport report;
  report.compatibility = CheckReplayTraceCompatibility(trace_dir, expectation);
  report.scan = ScanReplayTrace(trace_dir);

  ReplayTraceReader reader(trace_dir);
  ReplayTraceReadEvent event;
  ReplayTraceReadStatus read_status = ReplayTraceReadStatus::kEndOfTrace;
  while ((read_status = reader.ReadNextEvent(&event)) == ReplayTraceReadStatus::kEvent) {
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
        report.first_failure_payload_base64 = Base64Encode(event.payload_bytes);
        report.first_failure_payload_encoding = event.payload_encoding;
        report.first_failure_payload_type = event.payload_type;
      }
    } else if (event.event_type == "warning") {
      ++report.warning_event_count;
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
  if (report.warning_event_count > 0U && report.warning.empty()) {
    report.warning = "replay trace contains warning events";
  }
  if (report.unsupported_event_count > 0U && report.warning.empty()) {
    report.warning = "replay trace contains unsupported event types";
  }

  report.replay_ready = report.compatibility.compatible && report.scan.ok &&
                        report.has_session_config && report.unsupported_event_count == 0U;
  return report;
}

void WriteReplayTraceReport(const ReplayTraceReplayReport& report, const std::string& report_path) {
  std::string error;
  if (!WriteReportFile(report_path, report, &error)) {
    PROJECT_LOG_ERROR("failed to write replay trace report: {}", error);
  }
}

ReplayTracePlaybackResult PlaybackReplayTrace(const std::string& trace_dir,
                                              const ReplayTracePlaybackCallbacks& callbacks,
                                              const ReplayTracePlaybackOptions& options) {
  ReplayTracePlaybackResult result;
  ReplayTraceReader reader(trace_dir);

  ReplayTraceReadEvent event;
  ReplayTraceReadStatus read_status = ReplayTraceReadStatus::kEndOfTrace;
  while ((read_status = reader.ReadNextEvent(&event)) == ReplayTraceReadStatus::kEvent) {
    ++result.processed_event_count;

    std::string callback_error;
    bool callback_ok = true;
    if (event.event_type == "session_config") {
      callback_ok =
          InvokeReplayCallback(callbacks.on_session_config, event, callbacks.user_data,
                               "missing session_config replay callback", true, &callback_error);
    } else if (event.event_type == "cycle_input") {
      callback_ok =
          InvokeReplayCallback(callbacks.on_cycle_input, event, callbacks.user_data,
                               "missing cycle_input replay callback", true, &callback_error);
      if (callback_ok) {
        ++result.applied_input_count;
      }
    } else if (event.event_type == "scene_state") {
      callback_ok =
          InvokeReplayCallback(callbacks.on_scene_state, event, callbacks.user_data,
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
        std::string actual_output_payload;
        const ReplayTraceOutputStatus status =
            callbacks.on_cycle_output(event, callbacks.user_data, &actual_output_payload,
                                      &callback_error);
        if (status == ReplayTraceOutputStatus::kOtherFailure) {
          // 解码失败 / cycle 执行失败 / 入口 payload 类型不匹配：非分叉，记为回放失败。
          callback_ok = false;
        } else {
          // kHandledByModule 与 kDivergence 都表示模块已逐字段比较（一致或不一致），
          // 计入 compared_output_count，口径与旧的 generic 路径对齐。
          ++result.compared_output_count;
          // Generic divergence check（向后兼容）：
          // - kHandledByModule 且 actual_output_payload 非空时，与 event.payload_inline 比较。
          // - kDivergence 直接置分叉，使用模块携带的 actual_output_payload。
          const bool divergence =
              (status == ReplayTraceOutputStatus::kDivergence) ||
              (!actual_output_payload.empty() && actual_output_payload != event.payload_inline);
          if (divergence) {
            result.divergence_found = true;
            if (result.divergence_sequence == 0U) {
              result.divergence_sequence = event.sequence;
              result.expected_output_payload = event.payload_inline;
              result.actual_output_payload = actual_output_payload;
              if (result.first_error.empty()) {
                result.first_error = callback_error.empty()
                                         ? std::string("replay cycle_output divergence")
                                         : callback_error;
              }
            }
            result.ok = false;
            if (options.stop_on_first_divergence) {
              return result;
            }
          }
        }
      }
    } else if (event.event_type == "failure_marker") {
      ++result.failure_marker_count;
      callback_ok =
          InvokeReplayCallback(callbacks.on_failure_marker, event, callbacks.user_data,
                               "missing failure_marker replay callback", false, &callback_error);
      if (callback_ok && options.stop_on_failure_marker) {
        return result;
      }
    }

    if (!callback_ok) {
      result.ok = false;
      if (result.first_error.empty()) {
        result.first_error = callback_error.empty() ? "replay callback failed" : callback_error;
      }
      return result;
    }
  }

  if (read_status == ReplayTraceReadStatus::kError) {
    result.ok = false;
    if (result.first_error.empty()) {
      result.first_error = reader.first_error();
    }
  }

  return result;
}

}  // namespace replay
}  // namespace oneq
