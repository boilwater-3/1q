/**
 * @file RadarSessionConfigPresets.h
 * @brief 定义面向外部调用方的常用配置预设工厂。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_PRESETS_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_PRESETS_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief 构造偏向探测任务的信号流水线配置。
 * @note 返回值属于初始化基线；运行期可通过 `RadarSession::ApplyRuntimeConfig(...)` 覆盖。
 */
ONEQ_API SignalPipelineConfig MakeDetectionMissionSignalPipelineConfig();

/**
 * @brief 构造偏向稳定跟踪任务的信号流水线配置。
 * @note 返回值属于初始化基线；运行期可通过 `RadarSession::ApplyRuntimeConfig(...)` 覆盖。
 */
ONEQ_API SignalPipelineConfig MakeTrackingMissionSignalPipelineConfig();

/**
 * @brief 构造偏向稳健性的信号流水线配置。
 * @note 返回值属于初始化基线；运行期可通过 `RadarSession::ApplyRuntimeConfig(...)` 覆盖。
 */
ONEQ_API SignalPipelineConfig MakeHighRobustnessSignalPipelineConfig();

/**
 * @brief 构造默认 RadarSession 配置。
 * @note 返回值用于会话初始化；可外部调整项可在运行期通过 `RadarSession::ApplyRuntimeConfig(...)`
 * 提交。
 */
ONEQ_API session::RadarSessionConfig MakeDefaultRadarSessionConfig();

/**
 * @brief 构造偏向探测任务的 RadarSession 配置。
 * @note 返回值用于会话初始化；可外部调整项可在运行期通过 `RadarSession::ApplyRuntimeConfig(...)`
 * 提交。
 */
ONEQ_API session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig();

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_PRESETS_H_
