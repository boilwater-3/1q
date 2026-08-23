/**
 * @file SarRecordingSession.h
 * @brief 定义 SAR Replay 记录会话门面。
 */

#ifndef ONEQ_SAR_SESSION_SAR_RECORDING_SESSION_H_
#define ONEQ_SAR_SESSION_SAR_RECORDING_SESSION_H_

#include <memory>
#include <utility>

#include "1q/sar/session/SarSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
}  // namespace oneq

namespace sar {
namespace session {

/**
 * @brief SAR Replay 记录会话配置。
 * @note `replay_writer` 产出可被
 *       `ReplaySarTrace()` 消费的 Replay 目录。未配置 writer 时只透传 Session。
 */
struct ONEQ_API SarRecordingSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool record_config_on_construct{true};

  SarRecordingSessionOptions() = default;
};

/**
 * @brief SAR Replay 记录会话。
 */
class ONEQ_API SarRecordingSession {
 public:
  SarRecordingSession();
  explicit SarRecordingSession(SarSession session);
  explicit SarRecordingSession(config::SarSessionConfig config, SarRecordingSessionOptions options = {});
  ~SarRecordingSession();

  SarRecordingSession(const SarRecordingSession&) = delete;
  SarRecordingSession& operator=(const SarRecordingSession&) = delete;
  SarRecordingSession(SarRecordingSession&&) noexcept;
  SarRecordingSession& operator=(SarRecordingSession&&) noexcept;

  /**
   * @brief 执行单周期并返回聚合结果（同时写入 Replay 记录）。
   * @param[in] input 单周期输入载荷。
   * @return 单周期聚合结果。
   */
  SarCycleResult StepWithResult(const SarCycleInput& input);
  /**
   * @brief 执行单周期并返回产品载荷（元数据 + 聚焦图像，同时写入 Replay 记录）。
   * @param[in] input 单周期输入载荷。
   * @return 单周期产品载荷；非执行周期为空载荷。
   */
  SarCycleProduct Step(const SarCycleInput& input);
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

#endif  // ONEQ_SAR_SESSION_SAR_RECORDING_SESSION_H_
