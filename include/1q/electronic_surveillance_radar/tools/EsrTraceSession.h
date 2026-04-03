/**
 * @file EsrTraceSession.h
 * @brief 为电子侦察模块提供独立的中间层记录包装器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_TOOLS_ESR_TRACE_SESSION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_TOOLS_ESR_TRACE_SESSION_H_

#include <memory>
#include <string>
#include <utility>

#include "1q/common/trace/TraceSink.h"
#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace tools {

/**
 * @brief EsrTraceSessionOptions 描述记录包装器配置。
 */
struct ONEQ_API EsrTraceSessionOptions {
  std::shared_ptr<oneq::common::trace::TraceSink> sink{}; /**< 记录输出 sink */
  bool trace_config_on_construct{true};                   /**< 构造时是否记录配置 */

  EsrTraceSessionOptions() = default;
  EsrTraceSessionOptions(std::shared_ptr<oneq::common::trace::TraceSink> trace_sink,
                         bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
};

/**
 * @brief EsrTraceSession 作为 EsrSession 的独立中间层记录包装器。
 */
class ONEQ_API EsrTraceSession {
 public:
  explicit EsrTraceSession(core::session::EsrSessionConfig config = {},
                           EsrTraceSessionOptions options = {});

  common::EsrOutputFrame Step(const core::context::EsrCycleInput& input);
  core::session::EsrCycleResult StepWithResult(const core::context::EsrCycleInput& input);
  void ApplyRuntimeConfig(const core::session::EsrRuntimeConfigPatch& patch);

  core::session::EsrSession& session();
  const core::session::EsrSession& session() const;

 private:
  void Record(const std::string& phase, const std::string& payload_json) const;

  core::session::EsrSession session_;
  std::shared_ptr<oneq::common::trace::TraceSink> sink_;
};

}  // namespace tools
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_TOOLS_ESR_TRACE_SESSION_H_
