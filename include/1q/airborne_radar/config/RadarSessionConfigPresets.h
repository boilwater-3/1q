/**
 * @file RadarSessionConfigPresets.h
 * @brief 定义公开会话配置预设工厂。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_PRESETS_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_PRESETS_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {
namespace presets {

/**
 * @brief 构造默认 RadarSession 初始化配置。
 * @return 默认会话配置，包含缺省 semantic 流水线与环境配置。
 */
ONEQ_API session::RadarSessionConfig MakeDefaultRadarSessionConfig();
/**
 * @brief 构造面向探测任务的 RadarSession 初始化配置。
 * @return 带探测任务 semantic 预设的会话配置。
 */
ONEQ_API session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig();
/**
 * @brief 构造面向跟踪稳定性的 RadarSession 初始化配置。
 * @return 带跟踪任务 semantic 预设的会话配置。
 */
ONEQ_API session::RadarSessionConfig MakeTrackingMissionRadarSessionConfig();
/**
 * @brief 构造面向高稳健场景的 RadarSession 初始化配置。
 * @return 带高稳健 semantic 预设的会话配置。
 */
ONEQ_API session::RadarSessionConfig MakeHighRobustnessRadarSessionConfig();

}  // namespace presets
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_PRESETS_H_
