/**
 * @file EosTraceSession.h
 * @brief 为光电传感器模块提供独立的中间层记录包装器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_TOOLS_EOS_TRACE_SESSION_H_
#define ELECTRO_OPTICAL_SENSOR_TOOLS_EOS_TRACE_SESSION_H_

#include <memory>
#include <string>
#include <utility>

#include "1q/common/trace/TraceSink.h"
#include "1q/electro_optical_sensor/core/session/EosSession.h"

namespace electro_optical_sensor {
namespace tools {

/**
 * @brief EosTraceSessionOptions 描述记录包装器配置。
 */
struct ONEQ_API EosTraceSessionOptions {
  std::shared_ptr<oneq::common::trace::TraceSink> sink{}; /**< 记录输出 sink */
  bool trace_config_on_construct{true};                   /**< 构造时是否记录配置 */

  EosTraceSessionOptions() = default;
  EosTraceSessionOptions(std::shared_ptr<oneq::common::trace::TraceSink> trace_sink,
                         bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
};

/**
 * @brief EosTraceSession 作为 EosSession 的独立中间层记录包装器。
 */
class ONEQ_API EosTraceSession {
 public:
  explicit EosTraceSession(core::session::EosSessionConfig config = {},
                           EosTraceSessionOptions options = {});

  common::EosOutputFrame Step(const core::context::EosCycleInput& input);
  core::session::EosCycleResult StepWithResult(const core::context::EosCycleInput& input);
  void ApplyRuntimeConfig(const core::session::EosRuntimeConfigPatch& patch);

  core::session::EosSession& session();
  const core::session::EosSession& session() const;

 private:
  void Record(const std::string& phase, const std::string& payload_json) const;

  core::session::EosSession session_;
  std::shared_ptr<oneq::common::trace::TraceSink> sink_;
};

}  // namespace tools
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_TOOLS_EOS_TRACE_SESSION_H_
