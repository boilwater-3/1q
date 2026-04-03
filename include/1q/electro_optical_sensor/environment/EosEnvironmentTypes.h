/**
 * @file EosEnvironmentTypes.h
 * @brief EOS 环境模型输入输出契约。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief EosEnvironmentModelType 描述环境模型策略。
 */
enum class ONEQ_API EosEnvironmentModelType {
  kSimplified = 0, /**< 简化模型（固定环境参数） */
  kAdvanced        /**< 高级模型（高度/风速/云量驱动） */
};

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

/**
 * @brief EosEnvironmentDefaultConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EosEnvironmentDefaultConfig {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
};

/**
 * @brief EosEnvironmentRuntimeConfigPatch 描述运行期可变环境补丁。
 */
struct ONEQ_API EosEnvironmentRuntimeConfigPatch {
  bool has_model_type{false};
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};

  bool has_radiative_transfer_model{false};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};

  bool has_aerosol_density_factor{false};
  float aerosol_density_factor{1.0f};

  bool has_turbulence_factor{false};
  float turbulence_factor{1.0f};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_TYPES_H_
