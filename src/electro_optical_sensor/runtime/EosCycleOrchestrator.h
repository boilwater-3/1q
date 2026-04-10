/**
 * @file EosCycleOrchestrator.h
 * @brief EOS 运行期周期编排器：封装会话步进与运行期配置提交。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_RUNTIME_EOS_CYCLE_ORCHESTRATOR_H_
#define ELECTRO_OPTICAL_SENSOR_RUNTIME_EOS_CYCLE_ORCHESTRATOR_H_

#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace extension {
class IEosPipeline;
class EosController;
}  // namespace extension

namespace session {
namespace internal {

/**
 * @brief EosCycleOrchestrator 负责 EOS 会话的周期执行与运行期补丁提交。
 */
class EosCycleOrchestrator {
 public:
  EosCycleOrchestrator(const EosSessionConfig& config,
                       ::electro_optical_sensor::extension::IEosPipeline& pipeline,
                       ::electro_optical_sensor::extension::EosController& controller);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EosCycleResult Step(const EosCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期补丁。
   */
  void ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch);

 private:
  EosCycleResult BuildResult(const EosCycleInput& input) const;

  EosSessionConfig runtime_config_{};
  ::electro_optical_sensor::extension::IEosPipeline& pipeline_;
  ::electro_optical_sensor::extension::EosController& controller_;
};

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_RUNTIME_EOS_CYCLE_ORCHESTRATOR_H_
