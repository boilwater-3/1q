/**
 * @file EsrTraceSession.h
 * @brief 为电子侦察模块提供独立的中间层记录包装器。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_TRACE_SESSION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
namespace trace {
class TraceSink;
}
}  // namespace oneq

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrTraceSessionOptions 描述记录包装器配置。
 * @note `sink` 产出调试/观测记录，不能直接回放；`replay_writer` 产出可被
 *       `ReplayEsrTrace()` 消费的 replay trace 目录。需要可复现实验时应同时配置
 *       `replay_writer`。
 */
struct ONEQ_API EsrTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{}; /**< 记录输出 sink */
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true}; /**< 构造时是否记录配置 */

  EsrTraceSessionOptions() = default;
  EsrTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  EsrTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config,
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
  explicit EsrTraceSession(config::EsrSessionConfig config = {},
                           EsrTraceSessionOptions options = {});
  ~EsrTraceSession();

  EsrTraceSession(const EsrTraceSession&) = delete;
  EsrTraceSession& operator=(const EsrTraceSession&) = delete;
  EsrTraceSession(EsrTraceSession&&) noexcept;
  EsrTraceSession& operator=(EsrTraceSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧，同时向已配置 sink/replay_writer 记录。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  EsrOutputFrame Step(const EsrCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果，同时向已配置 sink/replay_writer 记录。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult StepWithResult(const EsrCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁，并转发到内部 EsrSession。
   * @param[in] patch 运行期补丁。
   */
  void ApplyRuntimeConfig(const config::EsrRuntimeConfigPatch& patch);

  /**
   * @brief 获取内部 EsrSession 的可变引用。
   * @return 内部会话引用。
   */
  session::EsrSession& session();

  /**
   * @brief 获取内部 EsrSession 的只读引用。
   * @return 内部会话常量引用。
   */
  const session::EsrSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_TRACE_SESSION_H_
