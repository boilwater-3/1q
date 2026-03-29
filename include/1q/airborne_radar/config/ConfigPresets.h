/**
 * @file ConfigPresets.h
 * @brief 定义面向外部调用方的常用配置预设工厂。
 */

#ifndef AIRBORNE_RADAR_CONFIG_CONFIG_PRESETS_H_
#define AIRBORNE_RADAR_CONFIG_CONFIG_PRESETS_H_

#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace common {
namespace config {

/**
 * @brief 构造偏向探测任务的信号流水线配置。
 */
ONEQ_API signal::pipeline::SignalPipelineConfig MakeDetectionMissionSignalPipelineConfig();

/**
 * @brief 构造偏向稳定跟踪任务的信号流水线配置。
 */
ONEQ_API signal::pipeline::SignalPipelineConfig MakeTrackingMissionSignalPipelineConfig();

/**
 * @brief 构造偏向稳健性的信号流水线配置。
 */
ONEQ_API signal::pipeline::SignalPipelineConfig MakeHighRobustnessSignalPipelineConfig();

/**
 * @brief 构造默认 RadarSession 配置。
 */
ONEQ_API core::session::RadarSessionConfig MakeDefaultRadarSessionConfig();

/**
 * @brief 构造偏向探测任务的 RadarSession 配置。
 */
ONEQ_API core::session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig();

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_CONFIG_PRESETS_H_
