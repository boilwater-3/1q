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
  std::string serializer_version{"replay-flatbuffers-v1"};
  std::string git_commit{};
  bool git_dirty{false};
  std::string build_type{};
  std::string compiler{};
  std::string compiler_version{};
  std::string platform{};
  std::string cpu_arch{};
  std::string library_version{};
  std::string dependency_versions_payload{"{}"};
  std::string float_policy{};
  std::string default_tolerances_payload{"{}"};
  std::uint32_t checkpoint_interval_cycles{0U};
  std::uint32_t event_chunk_size{10000U};
  std::uint32_t failure_window_event_count{128U};
  bool compress_closed_chunks{false}; /**< 若为 true，Writer 关闭 chunk 后将其压缩为 .gz
                                         并删除原文件；Reader 自动透明解压 */
};

struct ONEQ_API ReplayTraceEvent {
  std::string module{};
  std::string event_type{};
  std::string payload_type{};
  std::string payload_encoding{"flatbuffers"};
  std::string payload_inline{"{}"};
  std::string payload_bytes{};
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
  std::string payload_inline{};
  std::string payload_bytes{};
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
  std::string serializer_version{"replay-flatbuffers-v1"};
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
  std::uint64_t warning_event_count{0U};
  std::uint64_t unsupported_event_count{0U};
  std::uint64_t first_failure_sequence{0U};
  std::string first_failure_payload_base64{};
  std::string first_failure_payload_encoding{};
  std::string first_failure_payload_type{};
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
  std::string diagnostics_payload{"{}"};
};

using ReplayTraceEventCallback = bool (*)(const ReplayTraceReadEvent& event, void* user_data,
                                          std::string* error);

/**
 * @brief cycle_output 回调返回的结构化比较状态。
 *
 * 替代旧的 bool 返回值，让模块在回调中显式区分三类结果，避免回放框架靠解析
 * `error` 文本字符串来判断是否发生输出分叉（参见
 * `docs/review/batch_validation_consumer_friction.md` §1）。
 *
 * - `kHandledByModule`：模块已逐字段比较，输出一致。`actual_output_payload` 应留空
 *   （表示比较由模块内部完成），或填入模块自行生成的诊断 JSON。
 * - `kDivergence`：模块检测到输出分叉（逐字段比较不一致）。模块应同时通过
 *   `actual_output_payload` 携带实际输出摘要，供消费方结构化读取。
 * - `kOtherFailure`：解码失败、入口 payload 类型不匹配、cycle 执行失败等。
 *   这类失败**不**视为输出分叉，仅记为回放失败。
 */
enum class ReplayTraceOutputStatus {
  kHandledByModule = 0, /**< 模块已比较且输出一致 */
  kDivergence = 1,      /**< 模块检测到输出分叉 */
  kOtherFailure = 2     /**< 解码/执行/类型不匹配等失败，非分叉 */
};

using ReplayTraceOutputCallback = ReplayTraceOutputStatus (*)(const ReplayTraceReadEvent& event,
                                                              void* user_data,
                                                              std::string* actual_output_payload,
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
  std::uint64_t processed_event_count{0U}; /**< 已读取的 trace event 数，使用 64-bit 以支持长回放 */
  std::uint64_t applied_input_count{0U};   /**< 已应用的 cycle_input 数，使用 64-bit 以支持长回放 */
  std::uint64_t applied_scene_state_count{0U};   /**< 已应用的 scene_state 数 */
  std::uint64_t applied_runtime_patch_count{0U}; /**< 已应用的 runtime_config_patch 数 */
  std::uint64_t compared_output_count{
      0U};                                /**< 已比较的 cycle_output 数，使用 64-bit 以支持长回放 */
  std::uint64_t skipped_output_count{0U}; /**< 未要求输出回调时跳过的 cycle_output 数 */
  std::uint64_t failure_marker_count{0U}; /**< 已遇到的 failure_marker 数 */
  bool divergence_found{false};
  std::uint64_t divergence_sequence{0U};
  std::string expected_output_payload{};
  std::string actual_output_payload{};
  std::string first_error{};
};

class ONEQ_API ReplayTraceWriter final {
 public:
  ReplayTraceWriter(std::string trace_dir, ReplayTraceManifest manifest, bool overwrite = false);
  ~ReplayTraceWriter();

  ReplayTraceWriter(const ReplayTraceWriter&) = delete;
  ReplayTraceWriter& operator=(const ReplayTraceWriter&) = delete;

  void WriteEvent(const ReplayTraceEvent& event);
  void WriteFailureMarker(const ReplayTraceFailure& failure);
  void WriteFailureMarker(const ReplayTraceFailure& failure, const std::string& payload_bytes);
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
    const std::string& trace_dir, const ReplayTraceCompatibilityExpectation& expectation);
ONEQ_API ReplayTraceReplayReport BuildReplayTraceReport(
    const std::string& trace_dir, const ReplayTraceCompatibilityExpectation& expectation);
ONEQ_API void WriteReplayTraceReport(const ReplayTraceReplayReport& report,
                                     const std::string& report_path);
ONEQ_API ReplayTracePlaybackResult
PlaybackReplayTrace(const std::string& trace_dir, const ReplayTracePlaybackCallbacks& callbacks,
                    const ReplayTracePlaybackOptions& options = ReplayTracePlaybackOptions{});

}  // namespace replay
}  // namespace oneq

#endif  // ONEQ_REPLAY_REPLAY_TRACE_H_
