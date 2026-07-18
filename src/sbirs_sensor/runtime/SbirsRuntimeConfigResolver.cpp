#include "sbirs_sensor/runtime/SbirsRuntimeConfigResolver.h"

#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"

namespace sbirs_sensor {
namespace runtime {

SbirsRuntimeConfigResolution ResolveSbirsRuntimeConfigPatch(
    const config::SbirsSessionConfig& current_config,
    const config::SbirsRuntimeConfigPatch& patch) {
  SbirsRuntimeConfigResolution resolution;
  resolution.resolved_config = current_config;
  resolution.has_requested_update = patch.has_mission || patch.has_policy ||
                                    patch.has_environment || patch.has_work_mode ||
                                    patch.has_scan_rate_deg_per_sec || patch.has_sensor_enabled;
  if (!resolution.has_requested_update) {
    return resolution;
  }
  if (patch.has_mission) {
    resolution.resolved_config.mission = patch.mission;
  }
  if (patch.has_policy) {
    resolution.resolved_config.policy = patch.policy;
  }
  if (patch.has_environment) {
    resolution.resolved_config.environment = patch.environment;
  }
  if (patch.has_work_mode) {
    resolution.resolved_config.mission.work_mode = patch.work_mode;
  }
  if (patch.has_scan_rate_deg_per_sec) {
    resolution.resolved_config.mission.scan_rate_deg_per_sec = patch.scan_rate_deg_per_sec;
  }
  if (patch.has_sensor_enabled) {
    resolution.resolved_config.mission.power_on = patch.sensor_enabled;
  }
  resolution.is_valid = config::ValidateSbirsSessionConfig(resolution.resolved_config).empty();
  return resolution;
}

}  // namespace runtime
}  // namespace sbirs_sensor
