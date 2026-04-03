/**
 * @file IEsrEnvironmentService.h
 * @brief 定义电子侦察环境服务只读接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_ESR_ENVIRONMENT_SERVICE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_ESR_ENVIRONMENT_SERVICE_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief IEsrEnvironmentService 为电子侦察管线提供周期冻结与快照采样能力。
 */
class ONEQ_API IEsrEnvironmentService {
 public:
  virtual ~IEsrEnvironmentService() = default;

  /**
   * @brief 冻结当前周期环境事实。
   * @param[in] cycle_context 当前周期上下文。
   */
  virtual void BeginCycle(const EsrEnvironmentCycleContext& cycle_context) = 0;

  /**
   * @brief 采样当前周期环境快照。
   * @return 当前周期环境快照。
   */
  virtual EsrEnvironmentSnapshot SampleEnvironment() const = 0;

  /**
   * @brief 更新环境模型配置。
   * @param[in] config 新配置。
   */
  virtual void UpdateModelConfig(EsrEnvironmentModelConfig config) = 0;
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_ESR_ENVIRONMENT_SERVICE_H_
