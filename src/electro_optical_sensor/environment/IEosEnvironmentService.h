/**
 * @file IEosEnvironmentService.h
 * @brief EOS 环境扩展接口，允许外部接管环境因子建模。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_I_EOS_ENVIRONMENT_SERVICE_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_I_EOS_ENVIRONMENT_SERVICE_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief IEosEnvironmentService 定义 EOS 环境建模扩展点。
 */
class IEosEnvironmentService {
 public:
  virtual ~IEosEnvironmentService() = default;

  /**
   * @brief 计算环境因子。
   * @param[in] inputs 当前目标相关环境输入。
   * @return 环境因子输出。
   */
  virtual EosEnvironmentModelResult ResolveFactors(
      const EosEnvironmentModelInputs& inputs) const = 0;
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_I_EOS_ENVIRONMENT_SERVICE_H_
