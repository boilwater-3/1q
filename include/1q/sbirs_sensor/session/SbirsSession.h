/**
 * @file SbirsSession.h
 * @brief 定义 SBIRS-inspired 会话门面。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/foundation/SensorContract.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief SBIRS-inspired 会话门面，是外部调用方的主要使用面。
 * @note 该类不可拷贝但可移动，内部持有实现 (PIMPL)。实例本身非线程安全；
 *       运行期配置采用立即提交策略；pipeline snapshot 是经完整校验的 internal checkpoint，
 *       当前 controller 周期路径不声明执行后回滚步骤。
 */
class ONEQ_API SbirsSession {
 public:
  SbirsSession();
  ~SbirsSession() noexcept;

  SbirsSession(const SbirsSession&) = delete;
  SbirsSession& operator=(const SbirsSession&) = delete;
  SbirsSession(SbirsSession&&) noexcept;
  SbirsSession& operator=(SbirsSession&&) noexcept;

  /**
   * @brief 执行一个仿真周期，返回原始系统输出帧（主输出层）。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsOutputFrame`
   */
  SbirsOutputFrame Step(const SbirsCycleInput& input);
  /**
   * @brief 执行一个仿真周期，返回结构化执行结果（含输出帧、归属、校验与状态）。
   * @param[in] input 单周期输入
   * @return 本周期的 `SbirsCycleResult`
   */
  SbirsCycleResult StepWithResult(const SbirsCycleInput& input);
  /**
   * @brief 提交运行期配置补丁，立即生效；patch 无效或无可更新项时静默忽略。
   * @param[in] patch 运行期配置补丁
   * @note 内部委托 `TryApplyRuntimeConfig`，不返回成功与否；需要判别请使用后者。
   */
  void ApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);
  /**
   * @brief 尝试提交运行期配置补丁并立即生效。
   * @param[in] patch 运行期配置补丁
   * @return patch 有效且产生更新返回 true；patch 无效或无可更新项返回 false
   */
  bool TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);

  /**
   * @brief 工厂构造一个会话实例。
   * @param[in] config 会话初始化配置，留空使用默认配置
   * @return 构造好的 `SbirsSession`
   */
  static SbirsSession Create(const config::SbirsSessionConfig& config = {});
  /**
   * @brief 工厂构造会话实例，并输出配置校验问题。
   * @param[in] config 会话初始化配置
   * @param[out] issues 接收校验问题列表；可为 nullptr
   * @return 构造好的 `SbirsSession`
   */
  static SbirsSession CreateWithValidation(const config::SbirsSessionConfig& config,
                                           config::ValidationIssueList* issues);

 private:
  struct Impl;
  explicit SbirsSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

ONEQ_SENSOR_SESSION_CONTRACT(session::SbirsSession, session::SbirsCycleInput,
                             session::SbirsOutputFrame, session::SbirsCycleResult);

}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SESSION_H_
