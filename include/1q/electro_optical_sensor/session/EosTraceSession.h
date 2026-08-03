/**
 * @file EosTraceSession.h
 * @brief 为光电传感器模块提供独立的中间层记录包装器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_TRACE_SESSION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "1q/electro_optical_sensor/session/EosSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
namespace trace {
class TraceSink;
}
}  // namespace oneq

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosTraceSessionOptions 描述记录包装器配置。
 * @note `sink` 产出调试/观测记录，不能直接回放；`replay_writer` 产出可被
 *       `ReplayEosTrace()` 消费的 replay trace 目录。需要可复现实验时应同时配置
 *       `replay_writer`。
 */
struct ONEQ_API EosTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{}; /**< 记录输出 sink */
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true}; /**< 构造时是否记录配置 */

  EosTraceSessionOptions() = default;
  EosTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  EosTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config,
                         std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief EosTraceSession 作为 EosSession 的独立中间层记录包装器。
 */
class ONEQ_API EosTraceSession {
 public:
  explicit EosTraceSession(config::EosSessionConfig config = {},
                           EosTraceSessionOptions options = {});
  ~EosTraceSession();

  EosTraceSession(const EosTraceSession&) = delete;
  EosTraceSession& operator=(const EosTraceSession&) = delete;
  EosTraceSession(EosTraceSession&&) noexcept;
  EosTraceSession& operator=(EosTraceSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧（输出便捷入口）。
   * @note 若需要区分本周期是否执行或是否复用上一输出，请使用
   *       `StepWithResult()` 读取结构化状态字段。
   */
  EosOutputFrame Step(const EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @note 返回值包含 `executed_this_cycle` / `abort_reason` 等状态语义。
   */
  EosCycleResult StepWithResult(const EosCycleInput& input);

  /**
   * @brief 尝试应用运行期可变配置补丁并同步记录到 trace/replay 通道。
   * @param[in] patch 运行期配置补丁。
   * @note 启用 replay_writer 时先 apply 后写记录，保证回放时配置变更先于执行可重放。
   * @return 补丁被接受并应用成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch);

  /** @brief 获取被包装的内部会话引用。 */
  EosSession& session();
  /** @brief 获取被包装的内部会话只读引用。 */
  const EosSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_TRACE_SESSION_H_
