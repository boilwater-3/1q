/**
 * @file SarSession.h
 * @brief 定义 SAR 对外会话门面。
 */

#ifndef ONEQ_SAR_SESSION_SAR_SESSION_H_
#define ONEQ_SAR_SESSION_SAR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/foundation/SensorContract.h"

namespace sar {
namespace session {

class SarSessionFactory;

/**
 * @brief SarSession 提供 SAR 单周期步进执行入口。
 * @note 通过 `SarSessionFactory` 创建，避免外部直接拼装不一致依赖图。
 * @note 线程模型：会话内部维护可变运行态，非线程安全；并发调用需外部串行化或加锁。
 */
class ONEQ_API SarSession {
 public:
  SarSession();
  ~SarSession() noexcept;

  SarSession(const SarSession&) = delete;
  SarSession& operator=(const SarSession&) = delete;
  SarSession(SarSession&&) noexcept;
  SarSession& operator=(SarSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧。
   */
  SarOutputFrame Step(const SarCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   */
  SarCycleResult StepWithResult(const SarCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   */
  void ApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);

  /**
   * @brief 尝试应用运行期可变配置补丁。
   */
  bool TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);

 private:
  friend class SarSessionFactory;

  struct Impl;
  explicit SarSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名，防止伪对称漂移。
ONEQ_SENSOR_SESSION_CONTRACT(session::SarSession, session::SarCycleInput,
                             session::SarOutputFrame, session::SarCycleResult);

}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_SESSION_H_
