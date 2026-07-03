/**
 * @file ArTraceSession.h
 * @brief AR module primary trace session types.
 *
 * Primary header for trace sessions.
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACE_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace oneq {
namespace trace {
class TraceSink;
}
}  // namespace oneq

namespace airborne_radar {
namespace session {

/**
 * @brief ArTraceSessionOptions 描述记录包装器配置。
 * @note `sink` 产出调试/观测记录，不能直接回放；`replay_writer` 产出可被
 *       `ReplayArTrace()` 消费的 replay trace 目录。需要可复现实验时应同时配置
 *       `replay_writer`。
 */
struct ONEQ_API ArTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{};
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true}; /**< 构造时是否记录配置 */

  ArTraceSessionOptions() = default;
  ArTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  ArTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config,
                        std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief ArTraceSession 作为 ArSession 的独立中间层记录包装器。
 */
class ONEQ_API ArTraceSession {
 public:
  explicit ArTraceSession(const config::ArSessionConfig& config = {},
                          ArTraceSessionOptions options = {});

  ArTraceSession(ArTraceSession&& other) noexcept;
  ArTraceSession& operator=(ArTraceSession&& other) noexcept;

  ArTraceSession(const ArTraceSession&) = delete;
  ArTraceSession& operator=(const ArTraceSession&) = delete;

  ~ArTraceSession();

  TrackOutputFrame Step(const ArCycleInput& input);
  ArCycleResult StepWithResult(const ArCycleInput& input);

  void ApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch);

  const std::vector<session::ArCommand>& GetSubmittedCommands() const;
  bool HasLatestControlProfile() const;
  const session::ArControlProfile& GetLatestControlProfile() const;
  session::AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  ArSession& session();
  const ArSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACE_SESSION_H_
