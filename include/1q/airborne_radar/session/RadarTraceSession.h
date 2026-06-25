/**
 * @file RadarTraceSession.h
 * @brief 为机载雷达提供独立的中间层记录包装器。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_TRACE_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/session/RadarSession.h"
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
 * @brief RadarTraceSessionOptions 描述记录包装器配置。
 */
struct ONEQ_API RadarTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{};
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true}; /**< 构造时是否记录配置 */

  RadarTraceSessionOptions() = default;
  RadarTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink,
                           bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  RadarTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink,
                           bool trace_config,
                           std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief RadarTraceSession 作为 RadarSession 的独立中间层记录包装器。
 */
class ONEQ_API RadarTraceSession {
 public:
  explicit RadarTraceSession(const config::RadarSessionConfig& config = {},
                             RadarTraceSessionOptions options = {});

  RadarTraceSession(RadarTraceSession&& other) noexcept;
  RadarTraceSession& operator=(RadarTraceSession&& other) noexcept;

  RadarTraceSession(const RadarTraceSession&) = delete;
  RadarTraceSession& operator=(const RadarTraceSession&) = delete;

  ~RadarTraceSession();

  TrackOutputFrame Step(const RadarCycleInput& input);
  RadarCycleResult StepWithResult(const RadarCycleInput& input);

  void ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch);

  const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const;
  bool HasLatestControlProfile() const;
  const extension::control::RadarControlProfile& GetLatestControlProfile() const;
  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  RadarSession& session();
  const RadarSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_TRACE_SESSION_H_
