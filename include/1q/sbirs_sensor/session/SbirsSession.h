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
   * @brief 尝试提交运行期配置补丁并立即生效。
   * @param[in] patch 运行期配置补丁
   * @return patch 有效且产生更新返回 true；patch 无效或无可更新项返回 false
   */
  bool TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch);

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static SbirsSession Create(const config::SbirsSessionConfig& config = {});
  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   *
   * 与 `Create()` 唯一区别：构造前调用 `config::ValidateSbirsSessionConfig`
   * 校验配置合法性，将发现的问题写入 @p issues。无论 @p issues 是否为空，
   * 都会构造并返回会话（不阻断），调用方据 `issues->empty()` 决定后续。
   *
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note `ValidateSbirsSessionConfig` 由此路径被实调用，构成真实契约。
   */
  static SbirsSession CreateWithDiagnostics(const config::SbirsSessionConfig& config,
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
