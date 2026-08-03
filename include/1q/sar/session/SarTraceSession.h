/**
 * @file SarTraceSession.h
 * @brief 定义 SAR trace 会话占位门面。
 */

#ifndef ONEQ_SAR_SESSION_SAR_TRACE_SESSION_H_
#define ONEQ_SAR_SESSION_SAR_TRACE_SESSION_H_

#include <memory>
#include <utility>

#include "1q/sar/session/SarSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
namespace trace {
class TraceSink;
}
}  // namespace oneq

namespace sar {
namespace session {

/**
 * @brief SAR trace 会话配置。
 * @note `sink` 产出调试/观测记录，不能直接回放；`replay_writer` 产出可被
 *       `ReplaySarTrace()` 消费的 replay trace 目录。需要可复现实验时应同时配置
 *       `replay_writer`。
 */
struct ONEQ_API SarTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{};
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true};

  SarTraceSessionOptions() = default;
  SarTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  SarTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config,
                         std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief SAR trace 会话。
 */
class ONEQ_API SarTraceSession {
 public:
  SarTraceSession();
  explicit SarTraceSession(SarSession session);
  explicit SarTraceSession(config::SarSessionConfig config, SarTraceSessionOptions options = {});
  ~SarTraceSession();

  SarTraceSession(const SarTraceSession&) = delete;
  SarTraceSession& operator=(const SarTraceSession&) = delete;
  SarTraceSession(SarTraceSession&&) noexcept;
  SarTraceSession& operator=(SarTraceSession&&) noexcept;

  /**
   * @brief 执行单周期并返回聚合结果（同时写入 trace/replay 记录）。
   * @param[in] input 单周期输入载荷。
   * @return 单周期聚合结果。
   */
  SarCycleResult StepWithResult(const SarCycleInput& input);
  /**
   * @brief 执行单周期并返回输出帧（同时写入 trace/replay 记录）。
   * @param[in] input 单周期输入载荷。
   * @return 单周期输出帧。
   */
  SarOutputFrame Step(const SarCycleInput& input);
  /**
   * @brief 尝试应用运行期可变配置补丁（透传至内部 SarSession）。
   * @param[in] patch 运行期配置补丁。
   * @return 补丁成功应用时返回 true；补丁无效或被拒绝时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);

  /**
   * @brief 访问内部 SarSession 引用。
   * @return 内部会话引用。
   */
  SarSession& session();
  /**
   * @brief 以只读方式访问内部 SarSession 引用。
   * @return 内部会话 const 引用。
   */
  const SarSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_TRACE_SESSION_H_
