/**
 * @file ReplayTrace.h
 * @brief Defines the replay trace writer used for reproducible simulations.
 */

#ifndef ONEQ_REPLAY_REPLAY_TRACE_H_
#define ONEQ_REPLAY_REPLAY_TRACE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "1q/api.hpp"

namespace oneq {
namespace replay {

struct ONEQ_API ReplayTraceManifest {
  std::string trace_id{};
  std::string module{};
  std::string scenario_id{};
  int schema_version{1};
  std::string serializer_version{"replay-json-v1"};
  std::string git_commit{};
  bool git_dirty{false};
  std::string build_type{};
  std::string compiler{};
  std::string compiler_version{};
  std::string platform{};
  std::string cpu_arch{};
  std::string library_version{};
  std::string dependency_versions_json{"{}"};
  std::string float_policy{};
  std::string default_tolerances_json{"{}"};
  std::uint32_t checkpoint_interval_cycles{0U};
  std::uint32_t event_chunk_size{10000U};
  std::uint32_t failure_window_event_count{128U};
};

struct ONEQ_API ReplayTraceEvent {
  std::string module{};
  std::string event_type{};
  std::string payload_type{};
  std::string payload_encoding{"json"};
  std::string payload_json{"{}"};
  bool has_cycle_index{false};
  std::uint32_t cycle_index{0U};
  bool has_sim_time_sec{false};
  double sim_time_sec{0.0};
};

struct ONEQ_API ReplayTraceReadEvent {
  std::string raw_event_json{};
  std::int32_t schema_version{0};
  std::string trace_id{};
  std::uint64_t sequence{0U};
  std::string module{};
  std::string event_type{};
  bool has_cycle_index{false};
  std::uint32_t cycle_index{0U};
  bool has_sim_time_sec{false};
  double sim_time_sec{0.0};
  std::string payload_type{};
  std::string payload_encoding{};
  std::string payload_json{};
  std::string payload_hash{};
  std::string previous_event_hash{};
  bool payload_hash_matches{false};
  std::string event_hash{};
  bool previous_event_hash_matches{false};
};

struct ONEQ_API ReplayTraceScanResult {
  std::uint64_t event_count{0U};
  bool payload_hashes_ok{true};
  bool event_chain_ok{true};
  bool sequences_contiguous{true};
  bool ok{true};
  std::string first_error{};
};

struct ONEQ_API ReplayTraceCompatibilityExpectation {
  std::int32_t schema_version{1};
  std::string serializer_version{"replay-json-v1"};
  std::string git_commit{};
  bool require_git_commit_match{false};
  std::string module{};
  bool require_module_match{false};
};

struct ONEQ_API ReplayTraceCompatibilityResult {
  bool compatible{true};
  bool schema_version_matches{true};
  bool serializer_version_matches{true};
  bool git_commit_matches{true};
  bool module_matches{true};
  bool manifest_git_dirty{false};
  std::string manifest_trace_id{};
  std::string manifest_module{};
  std::int32_t manifest_schema_version{0};
  std::string manifest_serializer_version{};
  std::string manifest_git_commit{};
  std::string first_error{};
  std::string warning{};
};

struct ONEQ_API ReplayTraceFailure {
  std::string error_code{};
  std::string message{};
  std::string location{};
  bool has_cycle_index{false};
  std::uint32_t cycle_index{0U};
  bool has_sim_time_sec{false};
  double sim_time_sec{0.0};
  std::string diagnostics_json{"{}"};
};

class ONEQ_API ReplayTraceWriter final {
 public:
  ReplayTraceWriter(std::string trace_dir, ReplayTraceManifest manifest,
                    bool overwrite = false);
  ~ReplayTraceWriter();

  ReplayTraceWriter(const ReplayTraceWriter&) = delete;
  ReplayTraceWriter& operator=(const ReplayTraceWriter&) = delete;

  void WriteEvent(const ReplayTraceEvent& event);
  void WriteFailureMarker(const ReplayTraceFailure& failure);
  void Flush();

  const std::string& trace_dir() const;
  const ReplayTraceManifest& manifest() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class ONEQ_API ReplayTraceReader final {
 public:
  explicit ReplayTraceReader(std::string trace_dir);
  ~ReplayTraceReader();

  ReplayTraceReader(const ReplayTraceReader&) = delete;
  ReplayTraceReader& operator=(const ReplayTraceReader&) = delete;

  const std::string& trace_dir() const;
  const std::string& manifest_json() const;
  bool ReadNextEvent(ReplayTraceReadEvent* event);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

ONEQ_API ReplayTraceScanResult ScanReplayTrace(const std::string& trace_dir);
ONEQ_API ReplayTraceCompatibilityResult CheckReplayTraceCompatibility(
    const std::string& trace_dir,
    const ReplayTraceCompatibilityExpectation& expectation);

}  // namespace replay
}  // namespace oneq

#endif  // ONEQ_REPLAY_REPLAY_TRACE_H_
