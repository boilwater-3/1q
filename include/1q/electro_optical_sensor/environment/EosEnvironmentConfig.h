/**
 * @file EosEnvironmentConfig.h
 * @brief EOS 环境默认配置契约。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_H_

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
 * @brief EosEnvironmentDefaultConfig 描述初始化阶段默认环境配置。
 */
struct ONEQ_API EosEnvironmentDefaultConfig {
  EosEnvironmentModelType model_type{EosEnvironmentModelType::kSimplified};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_CONFIG_H_
