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

/**
 * @brief trace 记录包装器的配置选项。
 * @note `sink` 与 `replay_writer` 均为可选；提供 `replay_writer` 时会同时记录可回放 trace。
 */
struct ONEQ_API SbirsTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{};                /**< trace 输出 sink */
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{}; /**< replay trace writer */
  bool trace_config_on_construct{true};                          /**< 构造时是否记录配置快照 */

  SbirsTraceSessionOptions() = default;
  SbirsTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  SbirsTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config,
                           std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief 在 `SbirsSession` 之上叠加 trace/replay 记录能力的包装会话。
 * @note 该类不可拷贝但可移动，内部持有实现 (PIMPL)；实例本身非线程安全。
 */
class ONEQ_API SbirsTraceSession {
 public:
  /**
   * @brief 构造 trace 包装会话。
   * @param[in] config 会话初始化配置
   * @param[in] options trace/replay 选项
   */
  explicit SbirsTraceSession(config::SbirsSessionConfig config = {},
                             SbirsTraceSessionOptions options = {});
  ~SbirsTraceSession();

  SbirsTraceSession(const SbirsTraceSession&) = delete;
  SbirsTraceSession& operator=(const SbirsTraceSession&) = delete;
  SbirsTraceSession(SbirsTraceSession&&) noexcept;
  SbirsTraceSession& operator=(SbirsTraceSession&&) noexcept;

  /**
   * @brief 执行一个周期并记录 trace，返回原始系统输出帧。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsOutputFrame`
   */
  SbirsOutputFrame Step(const SbirsCycleInput& input);
  /**
   * @brief 执行一个周期并记录 trace，返回结构化执行结果。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsCycleResult`
   */
  SbirsCycleResult StepWithResult(const SbirsCycleInput& input);
  /**
   * @brief 尝试提交运行期配置补丁并记录到 trace。
   * @param[in] patch 运行期配置补丁
   * @return patch 有效且产生更新返回 true；patch 无效或无可更新项返回 false
   */
  bool TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);

  /** @brief 访问被包装的底层会话（可写）。 */
  SbirsSession& session();
  /** @brief 访问被包装的底层会话（只读）。 */
  const SbirsSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_TRACE_SESSION_H_
