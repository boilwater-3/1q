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
  bool has_scenario_config{false};
  EosEnvironmentScenarioConfig scenario_config{};
};

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_RUNTIME_CONFIG_PATCH_H_
