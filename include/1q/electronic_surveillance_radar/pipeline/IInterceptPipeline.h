/**
 * @file IInterceptPipeline.h
 * @brief 定义电子侦察流水线抽象接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_I_INTERCEPT_PIPELINE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_I_INTERCEPT_PIPELINE_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {

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
  virtual InterceptCycleResult RunCycle(const core::context::EsrCycleInput& input_state,
                                        const environment::IEsrEnvironmentService& environment) = 0;
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_I_INTERCEPT_PIPELINE_H_
