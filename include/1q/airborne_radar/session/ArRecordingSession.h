/**
 * @file ArRecordingSession.h
 * @brief 机载雷达 Replay 记录包装会话。
 *
 * 包装 `ArSession`，拦截 Step / runtime patch / 外部决策并写入
 * `ReplayTraceWriter`。未配置 writer 时只透传底层会话。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_RECORDING_SESSION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_RECORDING_SESSION_H_

#include <memory>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace airborne_radar {
namespace session {

/**
 * @brief ArRecordingSessionOptions 描述 Replay 记录包装器配置。
 * @note `replay_writer` 产出可被 `ReplayArTrace()` 消费的目录。
 */
struct ONEQ_API ArRecordingSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool record_config_on_construct{true}; /**< 构造时是否记录会话配置 */

  ArRecordingSessionOptions() = default;
};

/**
 * @brief ArRecordingSession 作为 ArSession 的独立中间层记录包装器。
 */
class ONEQ_API ArRecordingSession {
 public:
  /**
   * @brief 用会话配置与记录选项构造包装会话。
   * @param[in] config 四域会话配置，透传给内部 ArSession。
   * @param[in] options 记录器配置（replay_writer / 是否记录配置）。
   */
  explicit ArRecordingSession(const config::ArSessionConfig& config = {},
                              ArRecordingSessionOptions options = {});

  ArRecordingSession(ArRecordingSession&& other) noexcept;
  ArRecordingSession& operator=(ArRecordingSession&& other) noexcept;

  ArRecordingSession(const ArRecordingSession&) = delete;
  ArRecordingSession& operator=(const ArRecordingSession&) = delete;

  ~ArRecordingSession();

  /** @brief 记录并执行一个单周期，返回当前周期轨迹帧。 */
  TrackOutputFrame Step(const ArCycleInput& input);

  /** @brief 记录并执行一个单周期，返回结构化聚合结果。 */
  ArCycleResult StepWithResult(const ArCycleInput& input);

  /** @brief 尝试应用运行期配置补丁；接受或拒绝结果均写入 Replay 记录。 */
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

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_RECORDING_SESSION_H_
