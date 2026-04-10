/**
 * @file EosPipelineConfigMapper.h
 * @brief 定义 EOS 会话配置到 pipeline 配置的内部映射接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PIPELINE_CONFIG_MAPPER_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PIPELINE_CONFIG_MAPPER_H_

#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

/**
 * @brief 将会话配置映射为 pipeline 配置。
 * @param[in] config 会话配置。
 * @return 对应的 pipeline 配置。
 */
::electro_optical_sensor::extension::EosPipelineConfig BuildEosPipelineConfig(
    const EosSessionConfig& config);

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PIPELINE_CONFIG_MAPPER_H_
