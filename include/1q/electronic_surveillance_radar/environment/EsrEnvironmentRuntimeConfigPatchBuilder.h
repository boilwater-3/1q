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

  EsrEnvironmentRuntimeConfigPatchBuilder& WithAtmosphericPhysicsConfig(
      const EsrAtmosphericPhysicsConfig& atmospheric_physics) {
    patch_.has_atmospheric_physics = true;
    patch_.atmospheric_physics = atmospheric_physics;
    return *this;
  }

  EsrEnvironmentRuntimeConfigPatchBuilder& WithAtmosphericContext(
      const EsrAtmosphericDerivedContext& atmospheric_context) {
    patch_.has_atmospheric_context = true;
    patch_.atmospheric_context = atmospheric_context;
    return *this;
  }

  EsrEnvironmentRuntimeConfigPatch Build() const { return patch_; }

 private:
  EsrEnvironmentRuntimeConfigPatch patch_{};
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_ESR_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
