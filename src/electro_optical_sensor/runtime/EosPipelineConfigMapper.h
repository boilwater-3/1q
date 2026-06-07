/**
 * @file EosPipelineConfigMapper.h
 * @brief 定义 EOS 会话配置到内部执行配置的映射接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PIPELINE_CONFIG_MAPPER_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PIPELINE_CONFIG_MAPPER_H_

#include "1q/electro_optical_sensor/session/EosSession.h"
#include "electro_optical_sensor/config/EosInternalExecutionConfig.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief 将场景配置映射为环境模型配置（单入口）。
 * @note 已从公开头文件移入 src/，仅限内部使用。
 */
environment::EosEnvironmentModelConfig BuildModelConfigFromScenario(
    const environment::EosEnvironmentScenarioConfig& scenario_config);

}  // namespace environment
}  // namespace electro_optical_sensor

namespace electro_optical_sensor {
namespace runtime {
namespace session {

/**
 * @brief 将会话配置映射为内部执行配置。
 * @param[in] config 会话配置。
 * @return 对应的内部执行配置。
 */
config::execution::EosInternalExecutionConfig MapSessionToInternal(
    const ::electro_optical_sensor::config::EosSessionConfig& config);

/**
 * @brief 将检测策略解析为内部检测配置数值。
 */
void ApplyDetectionPolicyToInternal(
    const ::electro_optical_sensor::config::EosDetectionPolicyConfig& policy,
    config::execution::EosInternalExecutionConfig* exec);

/**
 * @brief 将杂散光策略解析为内部杂散光配置数值。
 */
void ApplyStrayLightPolicyToInternal(
    const ::electro_optical_sensor::config::EosStrayLightPolicyConfig& policy,
    config::execution::EosInternalExecutionConfig* exec);

/**
 * @brief 将环境模型配置写入内部环境配置。
 */
void ApplyEnvironmentModelToInternal(
    const environment::EosEnvironmentModelConfig& model_config,
    config::execution::EosInternalExecutionConfig* exec);

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PIPELINE_CONFIG_MAPPER_H_
