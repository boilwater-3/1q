/**
 * @file ControlProfileEffects.h
 * @brief 定义控制真值到 SignalPipeline 运行时配置的内部映射辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_

#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

float ComputeHeuristicSignalAdjustmentDb(
    const ::airborne_radar::config::execution::ControlProfileEffectsConfig& cfg,
    const session::RadarControlProfile& control_profile);

float ComputeHeuristicEnvironmentReliefDb(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const session::RadarControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot);

void ApplyControlProfileToConfig(const session::RadarControlProfile& control_profile,
                                 ExecutionConfig* config);


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_
