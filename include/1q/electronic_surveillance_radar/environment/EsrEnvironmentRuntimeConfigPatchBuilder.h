/**
 * @file EsrEnvironmentRuntimeConfigPatchBuilder.h
 * @brief ESR 环境运行期补丁链式构造器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatch.h"

namespace electronic_surveillance_radar {
namespace environment {

/**
 * @brief EsrEnvironmentRuntimeConfigPatch 链式构造器。
 */
class ONEQ_API EsrEnvironmentRuntimeConfigPatchBuilder {
 public:
  explicit EsrEnvironmentRuntimeConfigPatchBuilder(
      const EsrEnvironmentRuntimeConfigPatch& patch = {}) : patch_(patch) {}

  EsrEnvironmentRuntimeConfigPatchBuilder& WithModelConfig(
      const EsrEnvironmentModelConfig& model_config) {
    patch_.has_model_config = true;
    patch_.model_config = model_config;
    return *this;
  }

  EsrEnvironmentRuntimeConfigPatch Build() const { return patch_; }

 private:
  EsrEnvironmentRuntimeConfigPatch patch_{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
