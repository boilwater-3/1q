/**
 * @file EosEnvironmentConfigBuilder.h
 * @brief 提供 EOS 环境默认配置链式构造器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief EosEnvironmentDefaultConfig 链式构造器。
 */
class ONEQ_API EosEnvironmentConfigBuilder {
 public:
  explicit EosEnvironmentConfigBuilder(const EosEnvironmentDefaultConfig& config = {})
      : config_(config) {}

  EosEnvironmentConfigBuilder& WithModelType(EosEnvironmentModelType model_type) {
    config_.model_type = model_type;
    return *this;
  }

  EosEnvironmentDefaultConfig Build() const { return config_; }

 private:
  EosEnvironmentDefaultConfig config_{};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_BUILDER_H_
