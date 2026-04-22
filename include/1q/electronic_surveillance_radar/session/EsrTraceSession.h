/**
 * @file EsrTraceSession.h
 * @brief 为电子侦察模块提供独立的中间层记录包装器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_TRACE_SESSION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/trace/TraceSink.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrTraceSessionOptions 描述记录包装器配置。
 */
struct ONEQ_API EsrTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{}; /**< 记录输出 sink */
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true};                   /**< 构造时是否记录配置 */

  EsrTraceSessionOptions() = default;
  EsrTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink,
                         bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  EsrTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink,
                         bool trace_config,
                         std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief EsrTraceSession 作为 EsrSession 的独立中间层记录包装器。
 */
class ONEQ_API EsrTraceSession {
 public:
  explicit EsrTraceSession(session::EsrSessionConfig config = {},
                           EsrTraceSessionOptions options = {});

  output::EsrOutputFrame Step(const session::EsrCycleInput& input);
  session::EsrCycleResult StepWithResult(const session::EsrCycleInput& input);
  void ApplyRuntimeConfig(const session::EsrRuntimeConfigPatch& patch);

  session::EsrSession& session();
  const session::EsrSession& session() const;

 private:
  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes) const;
  void WriteReplayEvent(const std::string& event_type, const std::string& payload_type,
                        const std::string& payload_bytes, std::uint32_t cycle_index) const;

  session::EsrSession session_;
  std::shared_ptr<oneq::trace::TraceSink> sink_;
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer_;
  bool pending_input_written_{false};
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_TRACE_SESSION_H_
