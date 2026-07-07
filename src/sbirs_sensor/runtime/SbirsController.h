/**
 * @file SbirsController.h
 * @brief SBIRS-inspired 周期控制器。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_CONTROLLER_H_

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "sbirs_sensor/pipeline/SbirsPipeline.h"

namespace sbirs_sensor {
namespace runtime {

/**
 * @brief 周期控制器，负责输入校验、状态机 capture/restore、失败输出复用与周期执行。
 * @note 状态机 capture/restore 是 controller 内部失败回滚机制，不上升为 session 层事务契约。
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
  void ApplyConfig(const config::SbirsInternalExecutionConfig& config);
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
