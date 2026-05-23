/**
 * @file EosSession.h
 * @brief 定义光学传感器对外会话门面。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace extension {
class IEosPipeline;
class EosController;
}  // namespace extension
namespace environment {
class IEosEnvironmentService;
}
namespace session {

class EosSessionFactory;

/**
 * @brief EosSession 提供单周期步进执行入口。
 * @note 通过 `EosSessionFactory` 创建，避免外部直接拼装不一致依赖图。
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
  session::EosOutputFrame Step(const EosCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note 结果中的 `executed_this_cycle` / `reused_previous_output`
   *       提供结构化周期状态语义。
   * @note 非线程安全：会读写会话内部状态；并发调用需外部同步。
   */
  ::electro_optical_sensor::session::EosCycleResult StepWithResult(const EosCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期补丁。
   * @note 非线程安全：会更新运行期配置并可能重置扫描相位；并发调用需外部同步。
   */
  void ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch);

  /**
   * @brief 尝试应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @return 补丁被接受并应用成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const EosRuntimeConfigPatch& patch);

 private:
  friend class EosSessionFactory;

  struct Impl;
  explicit EosSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_H_
