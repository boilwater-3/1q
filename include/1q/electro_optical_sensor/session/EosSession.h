/**
 * @file EosSession.h
 * @brief 定义光学传感器对外会话门面。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigValidation.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/foundation/SensorContract.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosSession 提供单周期步进执行入口。
 * @note 通过静态工厂 `EosSession::Create` 创建，避免外部直接拼装不一致依赖图。
 * @note 线程模型：会话内部维护可变运行态，非线程安全；并发调用需外部串行化或加锁。
 */
class ONEQ_API EosSession {
 public:
  EosSession();
  ~EosSession() noexcept;

  EosSession(const EosSession&) = delete;
  EosSession& operator=(const EosSession&) = delete;
  EosSession(EosSession&&) noexcept;
  EosSession& operator=(EosSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧（输出便捷入口）。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   * @note 该接口仅返回输出帧，不携带 `executed_this_cycle` /
   *       `reused_previous_output` 等状态语义；若调用方需要区分
   *       "本周期实际执行" 与 "复用上一有效输出"，请使用 `StepWithResult()`。
   * @note 非线程安全：会读写会话内部状态；并发调用需外部同步。
   */
  EosOutputFrame Step(const EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note 结果中的 `executed_this_cycle` / `reused_previous_output`
   *       提供结构化周期状态语义。
   * @note 非线程安全：会读写会话内部状态；并发调用需外部同步。
   */
  EosCycleResult StepWithResult(const EosCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期补丁。
   * @note 非线程安全：会更新运行期配置并可能重置扫描相位；并发调用需外部同步。
   */
  void ApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch);

  /**
   * @brief 尝试应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @return 补丁被接受并应用成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch);

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static EosSession Create(const config::EosSessionConfig& config = {});

  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   *
   * 与 `Create()` 唯一区别：构造前调用 `config::ValidateEosSessionConfig`
   * 校验配置合法性，将发现的问题写入 @p issues。无论 @p issues 是否为空，
   * 都会构造并返回会话（不阻断），调用方据 `issues->empty()` 决定后续。
   *
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note `ValidateEosSessionConfig` 由此路径被实调用，构成真实契约。
   */
  static EosSession TryCreate(const config::EosSessionConfig& config,
                              config::ValidationIssueList* issues);

 private:

  struct Impl;
  explicit EosSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名，防止伪对称漂移。
ONEQ_SENSOR_SESSION_CONTRACT(session::EosSession, session::EosCycleInput,
                             session::EosOutputFrame, session::EosCycleResult);

}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_
