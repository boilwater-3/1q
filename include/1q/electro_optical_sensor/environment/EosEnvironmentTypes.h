/**
 * @file EosEnvironmentTypes.h
 * @brief EOS 环境模型输入输出类型契约。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief EosEnvironmentModelInputs 描述环境模型输入。
 */
struct ONEQ_API EosEnvironmentModelInputs {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  float platform_altitude_m{0.0f};
  float cloud_coverage_ratio{0.0f};
  float wind_speed_mps{0.0f};
  float base_aerosol_density_factor{1.0f};
  float base_turbulence_factor{1.0f};
};

/**
 * @brief EosEnvironmentModelResult 描述环境模型输出。
 */
struct ONEQ_API EosEnvironmentModelResult {
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  float path_radiance_scale_bias{1.0f};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_
