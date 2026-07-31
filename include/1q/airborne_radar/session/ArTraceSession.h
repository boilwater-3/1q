/**
 * @file ArTraceSession.h
 * @brief 机载雷达 trace 记录会话类型。
 *
 * trace 记录包装会话（调试/观测记录与 replay trace 写出）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACE_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACE_SESSION_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArSession.h"
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
 * @brief ArTraceSessionOptions 描述记录包装器配置。
 * @note `sink` 产出调试/观测记录，不能直接回放；`replay_writer` 产出可被
 *       `ReplayArTrace()` 消费的 replay trace 目录。需要可复现实验时应同时配置
 *       `replay_writer`。
 */
struct ONEQ_API ArTraceSessionOptions {
  std::shared_ptr<oneq::trace::TraceSink> sink{};
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool trace_config_on_construct{true}; /**< 构造时是否记录配置 */

  ArTraceSessionOptions() = default;
  ArTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config)
      : sink(std::move(trace_sink)), trace_config_on_construct(trace_config) {}
  ArTraceSessionOptions(std::shared_ptr<oneq::trace::TraceSink> trace_sink, bool trace_config,
                        std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_trace_writer)
      : sink(std::move(trace_sink)),
        replay_writer(std::move(replay_trace_writer)),
        trace_config_on_construct(trace_config) {}
};

/**
 * @brief ArTraceSession 作为 ArSession 的独立中间层记录包装器。
 */
class ONEQ_API ArTraceSession {
 public:
  /**
   * @brief 用四域配置与记录选项构造 trace 包装会话。
   * @param[in] config 四域会话配置，透传给内部 ArSession。
   * @param[in] options 记录器配置（sink / replay_writer / 是否记录配置）。
   */
  explicit ArTraceSession(const config::ArSessionConfig& config = {},
                          ArTraceSessionOptions options = {});

  ArTraceSession(ArTraceSession&& other) noexcept;
  ArTraceSession& operator=(ArTraceSession&& other) noexcept;

  ArTraceSession(const ArTraceSession&) = delete;
  ArTraceSession& operator=(const ArTraceSession&) = delete;

  ~ArTraceSession();

  /** @brief 记录并执行一个单周期，返回当前周期轨迹帧。 */
  TrackOutputFrame Step(const ArCycleInput& input);

  /** @brief 记录并执行一个单周期，返回结构化聚合结果。 */
  ArCycleResult StepWithResult(const ArCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁并记录（透传给内部 ArSession）。
   * @param[in] patch 运行期可变配置补丁。
   */
  void ApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch);

  /** @brief 尝试应用运行期配置补丁；接受或拒绝结果均写入 replay trace。 */
  bool TryApplyRuntimeConfig(const config::ArRuntimeConfigPatch& patch);

  /** @brief 提交外部 profile 覆盖（整包替换值，绕过 TacticalProposal 管线与 hold/cooldown）。 */
  session::ExternalDecisionSubmitStatus SubmitExternalDecision(
      session::ExternalDecisionOverride override_decision);

  /** @brief 获取被包装的底层 ArSession（只读）。 */
  const ArSession& session() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACE_SESSION_H_
