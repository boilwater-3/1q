/**
 * @file IMutableInterceptPipeline.h
 * @brief 定义可变电子侦察流水线接口，供 Session 外部装配注入。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_I_MUTABLE_INTERCEPT_PIPELINE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_I_MUTABLE_INTERCEPT_PIPELINE_H_

#include "1q/electronic_surveillance_radar/pipeline/IInterceptPipeline.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief 在 IInterceptPipeline 基础上补充运行态更新能力。
 */
class ONEQ_API IMutableInterceptPipeline : public IInterceptPipeline {
 public:
  ~IMutableInterceptPipeline() override = default;

  /**
   * @brief 更新流水线配置。
   * @param config 新配置。
   */
  virtual void UpdateConfig(InterceptPipelineConfig config) = 0;

  /**
   * @brief 更新运行态配置。
   * @param runtime_config 新运行态配置。
   */
  virtual void UpdateRuntimeConfig(InterceptRuntimeConfig runtime_config) = 0;
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_I_MUTABLE_INTERCEPT_PIPELINE_H_
