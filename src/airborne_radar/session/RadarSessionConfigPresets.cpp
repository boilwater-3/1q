/**
 * @file RadarSessionConfigPresets.cpp
 * @brief 实现机载雷达公开预设配置工厂。
 */

#include "1q/airborne_radar/config/presets/RadarSessionConfigPresets.h"

#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "airborne_radar/config/presets/BuildPresetPipelineConfig.h"

namespace airborne_radar {
namespace config {
namespace presets {

/**
 * @brief 返回探测任务流水线预设。
 * @return 与探测任务匹配的流水线配置。
 */
PipelineConfig MakeDetectionMissionPipelineConfig() {
  return internal::BuildDetectionMissionPresetConfig();
}

/**
 * @brief 返回跟踪任务流水线预设。
 * @return 与跟踪任务匹配的流水线配置。
 */
PipelineConfig MakeTrackingMissionPipelineConfig() {
  return internal::BuildTrackingMissionPresetConfig();
}

/**
 * @brief 返回高稳健流水线预设。
 * @return 面向高稳健场景的流水线配置。
 */
PipelineConfig MakeHighRobustnessPipelineConfig() {
  return internal::BuildHighRobustnessPresetConfig();
}

/**
 * @brief 返回默认会话配置。
 * @return 带默认波束扫描起点、扫描顺序与工作子模式的会话配置。
 */
session::RadarSessionConfig MakeDefaultRadarSessionConfig() {
  return config::RadarSessionConfigBuilder().Build();
}

/**
 * @brief 返回探测任务会话预设。
 * @return 在默认会话基础上叠加探测任务流水线预设的会话配置。
 */
session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig() {
  session::RadarSessionConfig config;
  config.pipeline_config = MakeDetectionMissionPipelineConfig();
  return config;
}

}  // namespace presets
}  // namespace config
}  // namespace airborne_radar
