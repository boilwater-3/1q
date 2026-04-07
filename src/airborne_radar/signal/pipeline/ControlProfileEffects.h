/**
 * @file ControlProfileEffects.h
 * @brief 定义控制真值到 SignalPipeline 运行时配置的内部映射辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "airborne_radar/signal/pipeline/InternalSignalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 计算控制真值对经验信号项的修正量。
 * @param[in] cfg 控制轮廓效应配置。
 * @param[in] control_profile 当前控制真值。
 * @return 对经验信号项的修正量（dB）。
 */
float ComputeHeuristicSignalAdjustmentDb(
    const ControlProfileEffectsConfig& cfg,
    const common::control::RadarControlProfile& control_profile);

/**
 * @brief 计算控制真值对经验环境惩罚项的抵消量。
 * @param[in] cfg 干扰效应配置。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 对经验环境惩罚项的抵消量（dB）。
 */
float ComputeHeuristicEnvironmentReliefDb(
    const JammingEffectsConfig& cfg, const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 将控制真值映射为当前周期生效的运行时配置。
 * @param[in] control_profile 当前控制真值。
 * @param[in,out] runtime_config 待调整的运行时配置。
 */
void ApplyControlProfileToConfig(const common::control::RadarControlProfile& control_profile,
                                 SignalPipelineConfig* runtime_config,
                                 InternalSignalPipelineConfig* internal_config);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_
