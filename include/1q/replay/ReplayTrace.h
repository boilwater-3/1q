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

class ONEQ_API ReplayTraceWriter final {
 public:
  ReplayTraceWriter(std::string trace_dir, ReplayTraceManifest manifest,
                    bool overwrite = false);
  ~ReplayTraceWriter();

  ReplayTraceWriter(const ReplayTraceWriter&) = delete;
  ReplayTraceWriter& operator=(const ReplayTraceWriter&) = delete;

  void WriteEvent(const ReplayTraceEvent& event);
  void Flush();

  const std::string& trace_dir() const;
  const ReplayTraceManifest& manifest() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace replay
}  // namespace oneq

#endif  // ONEQ_REPLAY_REPLAY_TRACE_H_
