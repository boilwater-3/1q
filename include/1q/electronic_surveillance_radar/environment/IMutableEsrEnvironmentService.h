/**
 * @file IMutableEsrEnvironmentService.h
 * @brief 定义 ESR 可变环境服务接口，供 Session 外部装配注入。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_MUTABLE_ESR_ENVIRONMENT_SERVICE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_MUTABLE_ESR_ENVIRONMENT_SERVICE_H_

#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief 在 IEsrEnvironmentService 基础上补充配置更新能力。
 */
class ONEQ_API IMutableEsrEnvironmentService : public IEsrEnvironmentService {
 public:
  ~IMutableEsrEnvironmentService() override = default;

  /**
   * @brief 更新环境模型配置。
   * @param config 新配置。
   */
  virtual void UpdateModelConfig(EsrEnvironmentModelConfig config) = 0;
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_MUTABLE_ESR_ENVIRONMENT_SERVICE_H_
