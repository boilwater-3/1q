/**
 * @file SessionToExecutionMapper.h
 * @brief 定义四域 session 配置到内部执行配置的唯一映射入口。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_SESSION_TO_EXECUTION_MAPPER_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_SESSION_TO_EXECUTION_MAPPER_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "airborne_radar/config/execution/InternalExecutionConfig.h"

namespace airborne_radar {
namespace config {
namespace mapping {

execution::InternalExecutionConfig MapSessionToExecution(
    const session::RadarSessionConfig& session_config);

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_SESSION_TO_EXECUTION_MAPPER_H_
