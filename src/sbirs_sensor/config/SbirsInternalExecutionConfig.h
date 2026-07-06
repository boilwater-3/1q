/**
 * @file SbirsInternalExecutionConfig.h
 * @brief SBIRS-inspired pipeline 内部执行配置。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_CONFIG_SBIRS_INTERNAL_EXECUTION_CONFIG_H_
#define ONEQ_SRC_SBIRS_SENSOR_CONFIG_SBIRS_INTERNAL_EXECUTION_CONFIG_H_

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace sbirs_sensor {
namespace config {

struct SbirsInternalExecutionConfig {
  SbirsSessionConfig session{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_CONFIG_SBIRS_INTERNAL_EXECUTION_CONFIG_H_
