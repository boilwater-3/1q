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
#include "1q/sar/config/SarSessionConfigValidation.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/foundation/SensorContract.h"

namespace sar {
namespace session {

class SarProductLifecycleRecorder;

/**
 * @brief SarSession 提供 SAR 单周期步进执行入口。
 * @note 通过静态工厂 `SarSession::Create` 创建，避免外部直接拼装不一致依赖图。
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
   * @brief 尝试应用运行期可变配置补丁。
   * @return 补丁成功应用时返回 `true`；补丁无效或被拒绝时返回 `false`（当前配置不变）。
   */
  bool TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);

  /**
   * @brief 注册产品生命周期记录器，由 Session 在每个周期自动驱动。
   *
   * 注册后，`StepWithResult()` 和 `Step()` 内部在 CycleResult 构建完成后自动调用
   * `recorder->Update()`，调用方无需手动调用。本周期产生的生命周期事件可通过
   * `recorder->GetLastEvents()` 获取。
   * @param[in] recorder 记录器指针；传入 `nullptr` 解除注册。
   * @note Session 不拥有 recorder，调用方须保证 recorder 生命周期长于 Session 的注册期。
   */
  void AttachProductLifecycleRecorder(SarProductLifecycleRecorder* recorder) noexcept;

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static SarSession Create(const config::SarSessionConfig& config = {});

  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   *
   * 与 `Create()` 唯一区别：构造前调用 `config::ValidateSarSessionConfig`
   * 校验配置合法性，将发现的问题写入 @p issues。无论 @p issues 是否为空，
   * 都会构造并返回会话（不阻断），调用方据 `issues->empty()` 决定后续。
   *
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note `ValidateSarSessionConfig` 由此路径被实调用，构成真实契约。
   */
  static SarSession CreateWithDiagnostics(const config::SarSessionConfig& config,
                                          SarIssueList* issues);

 private:

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
