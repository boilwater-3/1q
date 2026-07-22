/**
 * @file JammingEffects.h
 * @brief 定义旧单阶段执行器使用的 RF v1 链路适配函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_

#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 为待删除的单阶段执行器解析 RF v1 工程发射接收功率。
 * @note AR 两阶段 RF v2 路径不会调用本函数。
 */
bool TryResolveEngineeringInterferencePowerW(
    const ExecutionConfig& config, const session::EnvironmentSnapshot& environment_snapshot,
    float* received_power_w);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
