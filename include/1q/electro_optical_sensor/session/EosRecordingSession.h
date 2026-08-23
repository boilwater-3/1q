/**
 * @file EosRecordingSession.h
 * @brief 为光电传感器模块提供 Replay 记录包装器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RECORDING_SESSION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RECORDING_SESSION_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "1q/electro_optical_sensor/session/EosSession.h"

namespace oneq {
namespace replay {
class ReplayTraceWriter;
}
}  // namespace oneq

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosRecordingSessionOptions 描述记录包装器配置。
 * @note `replay_writer` 产出可被
 *       `ReplayEosTrace()` 消费的 Replay 目录。未配置 writer 时只透传 Session。
 */
struct ONEQ_API EosRecordingSessionOptions {
  std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer{};
  bool record_config_on_construct{true}; /**< 构造时是否记录配置 */

  EosRecordingSessionOptions() = default;
};

/**
 * @brief EosRecordingSession 作为 EosSession 的独立中间层记录包装器。
 */
class ONEQ_API EosRecordingSession {
 public:
  explicit EosRecordingSession(config::EosSessionConfig config = {},
                           EosRecordingSessionOptions options = {});
  ~EosRecordingSession();

  EosRecordingSession(const EosRecordingSession&) = delete;
  EosRecordingSession& operator=(const EosRecordingSession&) = delete;
  EosRecordingSession(EosRecordingSession&&) noexcept;
  EosRecordingSession& operator=(EosRecordingSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧（输出便捷入口）。
   * @note 若需要区分本周期是否执行或是否复用上一输出，请使用
   *       `StepWithResult()` 读取结构化状态字段。
   */
  EosOutputFrame Step(const EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @note 返回值包含 `status` / `abort_reason` 等状态语义。
   */
  EosCycleResult StepWithResult(const EosCycleInput& input);

  /**
   * @brief 尝试应用运行期可变配置补丁并同步写入 Replay 记录。
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

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_RECORDING_SESSION_H_
