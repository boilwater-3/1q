/**
 * @file SbirsTraceSession.h
 * @brief 为 SBIRS-inspired 模块提供独立的记录包装器。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_TRACE_SESSION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_TRACE_SESSION_H_

#include <memory>
#include <utility>

#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
namespace trace {
class TraceSink;
}
}  // namespace oneq

namespace sbirs_sensor {
namespace session {

struct ONEQ_API SbirsTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{};
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true};

  SbirsTraceSessionOptions() = default;
  SbirsTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  SbirsTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config,
                           std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

class ONEQ_API SbirsTraceSession {
 public:
  explicit SbirsTraceSession(config::SbirsSessionConfig config = {},
                             SbirsTraceSessionOptions options = {});
  ~SbirsTraceSession();

  SbirsTraceSession(const SbirsTraceSession&) = delete;
  SbirsTraceSession& operator=(const SbirsTraceSession&) = delete;
  SbirsTraceSession(SbirsTraceSession&&) noexcept;
  SbirsTraceSession& operator=(SbirsTraceSession&&) noexcept;

  SbirsOutputFrame Step(const SbirsCycleInput& input);
  SbirsCycleResult StepWithResult(const SbirsCycleInput& input);
  void ApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);

  SbirsSession& session();
  const SbirsSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_TRACE_SESSION_H_
