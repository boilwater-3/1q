/**
 * @file SessionToExecutionMapper.h
 * @brief 定义四域 session 配置到内部执行配置的唯一映射入口。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_SESSION_TO_EXECUTION_MAPPER_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_SESSION_TO_EXECUTION_MAPPER_H_

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "airborne_radar/config/InternalExecutionConfig.h"

namespace airborne_radar {
namespace config {
namespace mapping {

/**
 * @brief 将条件五域会话配置映射为内部执行配置（唯一运行期配置真值）。
 *
 * 会合并 hardware/policy/mission/orientation 域的字段，并依据 lifecycle 策略派生
 * engineering 子配置及默认 IMM 模型噪声差异系数。
 *
 * @note environment 域不参与此映射。环境观测数据（大气物理、植被散射）
 *       通过独立路径传递给 EnvironmentService 管理，不进入 InternalExecutionConfig。
 *       设计意图：environment 仅携带自然环境事实，不含算法调参字段。
 *
 * @param[in] session_config 外部提交的会话配置。
 * @return 完整初始化后的 InternalExecutionConfig。
 */
execution::InternalExecutionConfig MapSessionToExecution(
    const config::ArSessionConfig& session_config);

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_SESSION_TO_EXECUTION_MAPPER_H_
