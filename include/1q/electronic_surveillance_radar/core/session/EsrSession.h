/**
 * @file EsrSession.h
 * @brief 定义面向外部调用方的电子侦察会话门面。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_SESSION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {

/**
 * @brief EsrSessionConfig 描述电子侦察会话默认装配配置。
 */
struct ONEQ_API EsrSessionConfig {
  pipeline::InterceptPipelineConfig pipeline_config{}; /**< 流水线配置 */
  environment::EsrEnvironmentModelConfig environment_config{}; /**< 环境模型配置 */
};

/**
 * @brief EsrSession 提供单周期外部接入门面。
 */
class ONEQ_API EsrSession {
 public:
  /**
   * @brief 使用默认装配配置构造会话。
   * @param[in] config 会话配置。
   */
  explicit EsrSession(EsrSessionConfig config = {});
  ~EsrSession();

  EsrSession(const EsrSession&) = delete;
  EsrSession& operator=(const EsrSession&) = delete;

  /**
   * @brief 执行单周期并返回输出帧。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  common::EsrOutputFrame Step(const context::EsrCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult StepWithResult(const context::EsrCycleInput& input);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_SESSION_H_
