/**
 * @file EosEnvironmentModel.h
 * @brief 定义 EOS 环境模型参数派生接口（内部使用）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief EosEnvironmentModelType 描述环境模型策略。
 */
enum class EosEnvironmentModelType {
  kSimplified = 0,
  kAdvanced
};

/**
 * @brief EosEnvironmentModelInputs 描述环境模型输入。
 */
struct EosEnvironmentModelInputs {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  float platform_altitude_m{1000.0f};
  float cloud_coverage_ratio{0.2f};
  float wind_speed_mps{0.0f};
  float base_aerosol_density_factor{1.0f};
  float base_turbulence_factor{1.0f};
};

/**
 * @brief EosEnvironmentModelResult 描述环境模型输出。
 */
struct EosEnvironmentModelResult {
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  float path_radiance_scale_bias{1.0f};
};

/**
 * @brief 解析当前环境参数对辐射传输的修正。
 * @param[in] inputs 环境模型输入。
 * @return 环境模型输出。
 */
EosEnvironmentModelResult ResolveEnvironmentFactors(const EosEnvironmentModelInputs& inputs);

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_
