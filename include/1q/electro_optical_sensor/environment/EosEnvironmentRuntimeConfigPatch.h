/**
 * @file EosEnvironmentRuntimeConfigPatch.h
 * @brief EOS 环境运行期补丁契约。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace environment {

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

  bool has_enable_optical_countermeasure_extension{false};
  bool enable_optical_countermeasure_extension{false};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
