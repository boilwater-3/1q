/**
 * @file SbirsPipelineConfigMapper.h
 * @brief SBIRS-inspired public config 到 internal config 映射。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_PIPELINE_CONFIG_MAPPER_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_PIPELINE_CONFIG_MAPPER_H_

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "sbirs_sensor/config/SbirsInternalExecutionConfig.h"

namespace sbirs_sensor {
namespace runtime {

config::SbirsInternalExecutionConfig MapSessionToInternal(const config::SbirsSessionConfig& config);

}  // namespace runtime
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_PIPELINE_CONFIG_MAPPER_H_
