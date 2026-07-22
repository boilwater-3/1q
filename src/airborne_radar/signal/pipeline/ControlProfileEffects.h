/**
 * @file ControlProfileEffects.h
 * @brief 定义控制真值到 SignalPipeline 运行时配置的内部映射辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 将控制真值（LPI、ECCM、自适应波束等）就地叠加到运行时配置。
 * @param[in] control_profile 当前控制真值。
 * @param[in,out] config 待修改的运行时配置指针。
 * @note 当 config 为 nullptr 时直接返回，不做任何处理。
 */
void ApplyControlProfileToConfig(const session::ArControlProfile& control_profile,
                                 ExecutionConfig* config);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTROL_PROFILE_EFFECTS_H_
