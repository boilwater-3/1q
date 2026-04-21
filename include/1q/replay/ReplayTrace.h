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

struct ONEQ_API ReplayTraceReplayReport {
  ReplayTraceCompatibilityResult compatibility{};
  ReplayTraceScanResult scan{};
  bool replay_ready{false};
  bool has_session_config{false};
  bool has_failure_marker{false};
  std::uint64_t session_config_count{0U};
  std::uint64_t cycle_input_count{0U};
  std::uint64_t scene_state_count{0U};
  std::uint64_t runtime_config_patch_count{0U};
  std::uint64_t cycle_output_count{0U};
  std::uint64_t failure_marker_count{0U};
  std::uint64_t unsupported_event_count{0U};
  std::uint64_t first_failure_sequence{0U};
  std::string first_failure_payload_json{};
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

typedef bool (*ReplayTraceEventCallback)(const ReplayTraceReadEvent& event,
                                         void* user_data,
                                         std::string* error);
typedef bool (*ReplayTraceOutputCallback)(const ReplayTraceReadEvent& event,
                                          void* user_data,
                                          std::string* actual_output_json,
                                          std::string* error);

struct ONEQ_API ReplayTracePlaybackCallbacks {
  void* user_data{nullptr};
  ReplayTraceEventCallback on_session_config{nullptr};
  ReplayTraceEventCallback on_cycle_input{nullptr};
  ReplayTraceEventCallback on_scene_state{nullptr};
  ReplayTraceEventCallback on_runtime_config_patch{nullptr};
  ReplayTraceOutputCallback on_cycle_output{nullptr};
  ReplayTraceEventCallback on_failure_marker{nullptr};
};

struct ONEQ_API ReplayTracePlaybackOptions {
  bool stop_on_first_divergence{true};
  bool stop_on_failure_marker{false};
  bool require_output_callback{false};
};

struct ONEQ_API ReplayTracePlaybackResult {
  bool ok{true};
  std::uint64_t processed_event_count{0U};
  std::uint64_t applied_input_count{0U};
  std::uint64_t applied_scene_state_count{0U};
  std::uint64_t applied_runtime_patch_count{0U};
  std::uint64_t compared_output_count{0U};
  std::uint64_t skipped_output_count{0U};
  std::uint64_t failure_marker_count{0U};
  bool divergence_found{false};
  std::uint64_t divergence_sequence{0U};
  std::string expected_output_json{};
  std::string actual_output_json{};
  std::string first_error{};
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
ONEQ_API ReplayTraceReplayReport BuildReplayTraceReport(
    const std::string& trace_dir,
    const ReplayTraceCompatibilityExpectation& expectation);
ONEQ_API void WriteReplayTraceReport(const ReplayTraceReplayReport& report,
                                     const std::string& report_path);
ONEQ_API ReplayTracePlaybackResult PlaybackReplayTrace(
    const std::string& trace_dir,
    const ReplayTracePlaybackCallbacks& callbacks,
    const ReplayTracePlaybackOptions& options = ReplayTracePlaybackOptions{});

}  // namespace replay
}  // namespace oneq

#endif  // ONEQ_REPLAY_REPLAY_TRACE_H_
