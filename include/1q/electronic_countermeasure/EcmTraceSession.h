/**
 * @file EcmTraceSession.h
 * @brief 定义 ECM 调试 trace 与可回放事件包装器。
 */

#ifndef ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_TRACE_SESSION_H_
#define ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_TRACE_SESSION_H_

#include <memory>

#include "1q/electronic_countermeasure/EcmSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
namespace trace {
class TraceSink;
}
}  // namespace oneq

namespace electronic_countermeasure {
namespace session {

/** @brief ECM trace/replay 记录端配置。 */
struct ONEQ_API EcmTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{};
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true};
};

/** @brief 包装 EcmSession 并记录配置、补丁、周期输入和周期结果。 */
class ONEQ_API EcmTraceSession {
 public:
  explicit EcmTraceSession(config::EcmSessionConfig config = {},
                           EcmTraceSessionOptions options = {});
  ~EcmTraceSession();
  EcmTraceSession(const EcmTraceSession&) = delete;
  EcmTraceSession& operator=(const EcmTraceSession&) = delete;
  EcmTraceSession(EcmTraceSession&&) noexcept;
  EcmTraceSession& operator=(EcmTraceSession&&) noexcept;

  EcmCycleResult StepWithResult(const EcmCycleInput& input);
  EcmRuntimeConfigApplyResult ApplyRuntimeConfig(
      const config::EcmRuntimeConfigPatch& patch);
  EcmSession& session();
  const EcmSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_TRACE_SESSION_H_
