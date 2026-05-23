/**
 * @file IInterceptPipeline.h
 * @brief 定义电子侦察流水线抽象接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_I_INTERCEPT_PIPELINE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_I_INTERCEPT_PIPELINE_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace extension {

/**
 * @brief IInterceptPipeline 定义电子侦察单周期主处理流程接口。
 */
class ONEQ_API IInterceptPipeline {
 public:
  virtual ~IInterceptPipeline() = default;

  /**
   * @brief 执行一次电子侦察流水线循环。
   * @param[in] input_state 当前周期输入。
   * @param[in] environment 环境服务只读接口。
   * @return 当前周期流水线输出。
   */
  virtual InterceptPipelineResult RunCycle(const session::EsrCycleInput& input_state,
                                         const environment::IEsrEnvironmentService& environment) = 0;

  /**
   * @brief 更新流水线配置。
   * @param[in] config 新配置。
   */
  virtual void UpdateConfig(InterceptPipelineConfig config) = 0;

  /**
   * @brief 更新运行态配置。
   * @param[in] runtime_config 新运行态配置。
   */
  virtual void UpdateRuntimeConfig(InterceptRuntimeConfig runtime_config) = 0;

  /**
   * @brief 捕获当前运行态快照。
   * @return 当前运行态快照。
   */
  virtual InterceptPipelineRuntimeState CaptureRuntimeState() const = 0;

  /**
   * @brief 使用快照恢复运行态。
   * @param[in] state 待恢复的快照。
   * @return 恢复成功返回 `true`。
   */
  virtual bool RestoreRuntimeState(const InterceptPipelineRuntimeState& state) = 0;
};

}  // namespace extension
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_I_INTERCEPT_PIPELINE_H_
