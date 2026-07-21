/**
 * @file EosEnvironmentModel.h
 * @brief 定义 EOS 环境模型参数派生接口（内部使用）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_

#include "1q/environment/AtmosphericTypes.h"

namespace electro_optical_sensor {
namespace environment {

/** @brief 环境模型内部输入；不属于 EOS public session DTO。 */
struct EnvironmentModelInputs {
  float base_aerosol_density_factor{1.0f};
  float base_turbulence_factor{1.0f};
  float platform_altitude_m{0.0f};
  float cloud_coverage_ratio{0.0f};
  float wind_speed_mps{0.0f};
  oneq::environment::AtmosphericObservation atmospheric_physics{};
};

/** @brief 环境模型内部输出；仅供 pipeline 辐射传输计算消费。 */
struct EnvironmentModelResult {
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  float path_radiance_scale_bias{1.0f};
  float molecular_density_factor{1.0f};
};

/**
 * @brief 解析当前环境参数对辐射传输的修正。
 * @param[in] inputs 环境模型输入。
 * @return 环境模型输出。
 */
EnvironmentModelResult ResolveEnvironmentFactors(const EnvironmentModelInputs& inputs);

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_
