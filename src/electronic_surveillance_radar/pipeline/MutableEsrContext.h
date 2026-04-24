/**
 * @file MutableEsrContext.h
 * @brief 定义面向内部流水线的可变电子侦察上下文默认实现。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_MUTABLE_ESR_CONTEXT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_MUTABLE_ESR_CONTEXT_H_

#include "1q/electronic_surveillance_radar/extension/IEsrContext.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief MutableEsrContext 提供一个可直接驱动流水线的默认上下文实现。
 */
class MutableEsrContext final : public extension::IEsrContext {
 public:
  MutableEsrContext() = default;
  ~MutableEsrContext() override = default;

  /**
   * @brief 使用周期输入和环境快照初始化上下文。
   * @param input 周期输入。
   * @param environment_snapshot 环境快照。
   * @param pipeline_config 流水线配置。
   * @param runtime_config 运行态配置。
   */
  void BeginCycle(const session::EsrCycleInput& input,
                  const environment::EsrEnvironmentSnapshot& environment_snapshot,
                  const extension::InterceptPipelineConfig& pipeline_config,
                  const extension::InterceptRuntimeConfig& runtime_config);

  std::uint32_t GetCycleIndex() const override;
  float GetCycleDeltaTimeSec() const override;
  const session::EsrPoseState& GetPlatformPose() const override;
  const session::EsrSceneEmitterList& GetSceneEmitters() const override;
  const environment::EsrEnvironmentSnapshot& GetEnvironmentSnapshot() const override;
  const extension::InterceptPipelineConfig& GetPipelineConfig() const override;
  const extension::InterceptRuntimeConfig& GetRuntimeConfig() const override;

 private:
  std::uint32_t cycle_index_{0U};
  float dt_sec_{1.0f};
  session::EsrPoseState platform_pose_{};
  session::EsrSceneEmitterList scene_emitters_{};
  environment::EsrEnvironmentSnapshot environment_snapshot_{};
  extension::InterceptPipelineConfig pipeline_config_{};
  extension::InterceptRuntimeConfig runtime_config_{};
};

}  // namespace pipeline

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_MUTABLE_ESR_CONTEXT_H_
