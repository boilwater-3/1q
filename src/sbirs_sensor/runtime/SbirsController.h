/**
 * @file SbirsController.h
 * @brief SBIRS-inspired 周期控制器。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"
#include "sbirs_sensor/runtime/SbirsRuntimeConfigImpact.h"

namespace sbirs_sensor {
namespace runtime {

/**
 * @brief 周期控制器，负责输入校验、周期执行、结果组装与失败输出复用。
 * @note pipeline 结果已经把 record 与 attribution 组成原子元素，当前输出装配后不存在可能失败的
 *       commit 步骤，因此不建立虚构的 controller rollback 分支。
 */
class SbirsController {
 public:
  /**
   * @brief 构造控制器并应用初始内部执行配置。
   * @param[in] config 内部执行配置
   */
  explicit SbirsController(const config::SbirsInternalExecutionConfig& config);

  /**
   * @brief 应用新的内部执行配置（runtime patch 立即生效后调用）。
   * @param[in] config 新的内部执行配置
   */
  void ApplyConfig(const config::SbirsInternalExecutionConfig& config,
                   const SbirsRuntimeConfigImpact& impact);
  /**
   * @brief 执行一个周期：校验输入、推进 pipeline、生成结构化结果，必要时复用上一有效输出。
   * @param[in] input 单周期输入
   * @return 单周期结构化执行结果
   */
  session::SbirsCycleResult RunOnce(const session::SbirsCycleInput& input);

 private:
  pipeline::SbirsPipeline pipeline_;
  bool has_latest_output_{false};
  session::SbirsOutputFrame latest_output_{};
};

}  // namespace runtime
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_
