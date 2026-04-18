/**
 * @file RadarSessionConfig.h
 * @brief 定义 RadarSession 的初始化配置结构。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSession 初始化配置。
 *
 * 会话配置由两个正交关注点组成：
 * - `pipeline_config`：完整的信号流水线配置（expert + orientation）。
 * - `environment_default_config`：会话初始化时使用的默认环境参数。
 */
struct ONEQ_API RadarSessionConfig {
  config::PipelineConfig pipeline_config{};                         /**< 信号流水线配置。 */
  environment::EnvironmentDefaultConfig environment_default_config{}; /**< 环境默认配置。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
