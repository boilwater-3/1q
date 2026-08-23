/**
 * @file EsrRecordingSession.h
 * @brief 为电子侦察模块提供 Replay 记录包装器。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_RECORDING_SESSION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_RECORDING_SESSION_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
}  // namespace oneq

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrRecordingSessionOptions 描述记录包装器配置。
 * @note `replay_writer` 产出可被
 *       `ReplayEsrTrace()` 消费的 Replay 目录。未配置 writer 时只透传 Session。
 */
struct ONEQ_API EsrRecordingSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool record_config_on_construct{true}; /**< 构造时是否记录配置 */

  EsrRecordingSessionOptions() = default;
};

/**
 * @brief EsrRecordingSession 作为 EsrSession 的独立中间层记录包装器。
 */
class ONEQ_API EsrRecordingSession {
 public:
  explicit EsrRecordingSession(config::EsrSessionConfig config = {},
                           EsrRecordingSessionOptions options = {});
  ~EsrRecordingSession();

  EsrRecordingSession(const EsrRecordingSession&) = delete;
  EsrRecordingSession& operator=(const EsrRecordingSession&) = delete;
  EsrRecordingSession(EsrRecordingSession&&) noexcept;
  EsrRecordingSession& operator=(EsrRecordingSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧，在已配置 replay_writer 时写入 Replay 记录。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  EsrOutputFrame Step(const EsrCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果，在已配置 replay_writer 时写入 Replay 记录。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult StepWithResult(const EsrCycleInput& input);

  /**
   * @brief 尝试应用运行期可变配置补丁并返回结构化结果（透传给内部 EsrSession）。
   * @param[in] patch 运行期补丁。
   * @return 结构化应用结果（含状态码）。
   */
  EsrRuntimeConfigApplyResult TryApplyRuntimeConfig(
      const config::EsrRuntimeConfigPatch& patch);

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

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_RECORDING_SESSION_H_
