/**
 * @file SbirsRuntimeConfigResolver.h
 * @brief SBIRS-inspired runtime patch resolver。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_RESOLVER_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_RESOLVER_H_

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace sbirs_sensor {
namespace runtime {

struct SbirsRuntimeConfigResolution {
  bool is_valid{false};
  bool has_requested_update{false};
  config::SbirsSessionConfig resolved_config{};
};

SbirsRuntimeConfigResolution ResolveSbirsRuntimeConfigPatch(
    const config::SbirsSessionConfig& current_config, const config::SbirsRuntimeConfigPatch& patch);

}  // namespace runtime
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_RESOLVER_H_
