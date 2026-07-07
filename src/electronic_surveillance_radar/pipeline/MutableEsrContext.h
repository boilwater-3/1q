/**
 * @file MutableEsrContext.h
 * @brief 定义面向内部流水线的可变电子侦察上下文默认实现。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_MUTABLE_ESR_CONTEXT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_MUTABLE_ESR_CONTEXT_H_

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief MutableEsrContext 提供一个可直接驱动流水线的周期上下文。
 */
class MutableEsrContext final {
 public:
  MutableEsrContext() = default;
  ~MutableEsrContext() = default;

  /**
   * @brief 使用周期输入和环境快照初始化上下文。
   * @param[in] input 周期输入。
   * @param[in] environment_snapshot 环境快照。
   * @param[in] pipeline_config 流水线配置。
   * @param[in] runtime_config 运行态配置。
   */
  void BeginCycle(const session::EsrCycleInput& input,
                  const session::EsrEnvironmentSnapshot& environment_snapshot,
                  const extension::InterceptPipelineConfig& pipeline_config,
                  const extension::InterceptRuntimeConfig& runtime_config);

  /** @brief 返回当前周期号。 */
  std::uint32_t GetCycleIndex() const;
  /** @brief 返回当前周期步长（单位：s）。 */
  float GetCycleDeltaTimeSec() const;
  /** @brief 返回侦察平台姿态状态的只读引用。 */
  const oneq::foundation::PoseState& GetPlatformPose() const;
  /** @brief 返回场景辐射源输入列表的只读引用。 */
  const session::EsrSceneEmitterList& GetSceneEmitters() const;
  /** @brief 返回当前周期环境快照的只读引用。 */
  const session::EsrEnvironmentSnapshot& GetEnvironmentSnapshot() const;
  /** @brief 返回流水线配置的只读引用。 */
  const extension::InterceptPipelineConfig& GetPipelineConfig() const;
  /** @brief 返回运行态配置的只读引用。 */
  const extension::InterceptRuntimeConfig& GetRuntimeConfig() const;

 private:
  std::uint32_t cycle_index_{0U};
  float dt_sec_{1.0f};
  oneq::foundation::PoseState platform_pose_{};
  session::EsrSceneEmitterList scene_emitters_{};
  session::EsrEnvironmentSnapshot environment_snapshot_{};
  extension::InterceptPipelineConfig pipeline_config_{};
  extension::InterceptRuntimeConfig runtime_config_{};
};

}  // namespace pipeline

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_MUTABLE_ESR_CONTEXT_H_
