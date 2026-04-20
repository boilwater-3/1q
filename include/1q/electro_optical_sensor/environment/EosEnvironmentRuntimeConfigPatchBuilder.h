/**
 * @file EosEnvironmentRuntimeConfigPatchBuilder.h
 * @brief EOS 环境运行期补丁链式构造器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatch.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief EosEnvironmentRuntimeConfigPatch 链式构造器。
 */
class ONEQ_API EosEnvironmentRuntimeConfigPatchBuilder {
 public:
  explicit EosEnvironmentRuntimeConfigPatchBuilder(
      const EosEnvironmentRuntimeConfigPatch& patch = {}) : patch_(patch) {}

  EosEnvironmentRuntimeConfigPatchBuilder& WithModelType(EosEnvironmentModelType model_type) {
    patch_.has_model_type = true;
    patch_.model_type = model_type;
    return *this;
  }

  EosEnvironmentRuntimeConfigPatchBuilder& WithRadiativeTransferModel(
      foundation::radiative_transfer::RadiativeTransferModel model) {
    patch_.has_radiative_transfer_model = true;
    patch_.radiative_transfer_model = model;
    return *this;
  }

  EosEnvironmentRuntimeConfigPatchBuilder& WithAerosolDensityFactor(float value) {
    patch_.has_aerosol_density_factor = true;
    patch_.aerosol_density_factor = value;
    return *this;
  }

  EosEnvironmentRuntimeConfigPatchBuilder& WithTurbulenceFactor(float value) {
    patch_.has_turbulence_factor = true;
    patch_.turbulence_factor = value;
    return *this;
  }

  EosEnvironmentRuntimeConfigPatchBuilder& WithEnableOpticalCountermeasureExtension(
      bool enable) {
    patch_.has_enable_optical_countermeasure_extension = true;
    patch_.enable_optical_countermeasure_extension = enable;
    return *this;
  }

  EosEnvironmentRuntimeConfigPatchBuilder& WithModelDetails(
      foundation::radiative_transfer::RadiativeTransferModel model,
      float aerosol_density_factor,
      float turbulence_factor,
      bool enable_optical_countermeasure_extension = false) {
    patch_.has_radiative_transfer_model = true;
    patch_.radiative_transfer_model = model;
    patch_.has_aerosol_density_factor = true;
    patch_.aerosol_density_factor = aerosol_density_factor;
    patch_.has_turbulence_factor = true;
    patch_.turbulence_factor = turbulence_factor;
    patch_.has_enable_optical_countermeasure_extension = true;
    patch_.enable_optical_countermeasure_extension = enable_optical_countermeasure_extension;
    return *this;
  }

  EosEnvironmentRuntimeConfigPatch Build() const { return patch_; }

 private:
  EosEnvironmentRuntimeConfigPatch patch_{};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
