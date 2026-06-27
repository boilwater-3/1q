/**
 * @file IEosEnvironmentService.h
 * @brief EOS 内部环境建模抽象。
 *
 * 这是 EOS pipeline 内部用于解耦环境因子计算的抽象（默认实现 DefaultEosEnvironmentService
 * 在 EosPipeline.cpp 内部）。当前 public boundary 不支持外部替换 environment service
 * （见 design.md §公开边界），故本接口不是外部扩展点。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_I_EOS_ENVIRONMENT_SERVICE_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_I_EOS_ENVIRONMENT_SERVICE_H_

#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief IEosEnvironmentService 是 EOS 内部环境建模抽象（非外部扩展点）。
 */
class IEosEnvironmentService {
 public:
  virtual ~IEosEnvironmentService() = default;

  /**
   * @brief 计算环境因子。
   * @param[in] inputs 当前目标相关环境输入。
   * @return 环境因子输出。
   */
  virtual session::EosEnvironmentModelResult ResolveFactors(
      const session::EosEnvironmentModelInputs& inputs) const = 0;
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_I_EOS_ENVIRONMENT_SERVICE_H_
