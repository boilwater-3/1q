/**
 * @file PipelineConfigPresets.h
 * @brief 定义公开流水线预设配置工厂。
 */

#ifndef AIRBORNE_RADAR_CONFIG_PRESETS_PIPELINE_CONFIG_PRESETS_H_
#define AIRBORNE_RADAR_CONFIG_PRESETS_PIPELINE_CONFIG_PRESETS_H_

#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {
namespace presets {

/**
 * @brief 构造面向探测任务的流水线预设。
 * @return 使用 semantic 模式的探测任务流水线配置。
 */
ONEQ_API PipelineConfig MakeDetectionMissionPipelineConfig();
/**
 * @brief 构造面向跟踪任务的流水线预设。
 * @return 使用 semantic 模式的跟踪任务流水线配置。
 */
ONEQ_API PipelineConfig MakeTrackingMissionPipelineConfig();
/**
 * @brief 构造面向高稳健场景的流水线预设。
 * @return 使用 semantic 模式的高稳健流水线配置。
 */
ONEQ_API PipelineConfig MakeHighRobustnessPipelineConfig();

}  // namespace presets
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_PRESETS_PIPELINE_CONFIG_PRESETS_H_
