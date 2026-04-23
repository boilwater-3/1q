/**
 * @file IEsrContext.h
 * @brief 定义电子侦察周期上下文抽象接口。
 *
 * IEsrContext 为流水线各阶段提供统一的周期输入和配置只读访问，
 * 解耦流水线与上层调度之间的数据依赖。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_I_ESR_CONTEXT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_I_ESR_CONTEXT_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/model/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"

namespace electronic_surveillance_radar {
namespace extension {

/**
 * @brief IEsrContext 抽象了电子侦察单周期执行上下文。
 *
 * 通过该接口，流水线各阶段不再直接依赖上层输入结构，
 * 实现依赖倒置与阶段间数据共享。
 */
class ONEQ_API IEsrContext {
 public:
  virtual ~IEsrContext() = default;

  /** @brief 获取当前周期号。 */
  virtual std::uint32_t GetCycleIndex() const = 0;

  /** @brief 获取当前周期步长（单位：s）。 */
  virtual float GetCycleDeltaTimeSec() const = 0;

  /** @brief 获取当前周期平台姿态。 */
  virtual const model::EsrPoseState& GetPlatformPose() const = 0;

  /** @brief 获取当前周期场景辐射源列表。 */
  virtual const model::EmitterTruthStateList& GetSceneEmitters() const = 0;

  /** @brief 获取当前周期环境快照。 */
  virtual const environment::EsrEnvironmentSnapshot& GetEnvironmentSnapshot() const = 0;

  /** @brief 获取当前流水线配置。 */
  virtual const InterceptPipelineConfig& GetPipelineConfig() const = 0;

  /** @brief 获取当前运行态配置。 */
  virtual const InterceptRuntimeConfig& GetRuntimeConfig() const = 0;
};

}  // namespace extension

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_I_ESR_CONTEXT_H_
