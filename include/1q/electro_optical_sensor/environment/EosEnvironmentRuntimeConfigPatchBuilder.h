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

  EosEnvironmentRuntimeConfigPatch Build() const { return patch_; }

 private:
  EosEnvironmentRuntimeConfigPatch patch_{};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_BUILDER_H_
