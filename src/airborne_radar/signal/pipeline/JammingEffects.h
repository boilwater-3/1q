/**
 * @file JammingEffects.h
 * @brief 定义 SignalPipeline 干扰效应建模的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 判断环境快照是否携带多源干扰事实。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return jammer_sources 非空时返回 true。
 */
bool HasMultiSourceJammingFacts(const session::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 按当前接收频率、波束和硬件解析工程 RF 干扰接收功率。
 * @param[in] config 当前周期已解析执行配置。
 * @param[in] environment_snapshot 当前周期冻结环境快照。
 * @param[out] received_power_w 成功时写入全部发射源的线性功率和；失败时保持原值。
 * @return 非工程模式返回零并成功；工程链路事实完整且全部可求解时返回 true。
 */
bool TryResolveEngineeringInterferencePowerW(
    const ExecutionConfig& config, const session::EnvironmentSnapshot& environment_snapshot,
    float* received_power_w);

/**
 * @brief 计算在当前控制真值下单个干扰源的残余因子（0~1，越小表示抑制越强）。
 * @param[in] control_profile 当前控制真值（ECCM/自适应波束等）。
 * @param[in] jammer_source 单个干扰源事实。
 * @return 残余因子，已钳位到 [kResidualFactorMin, 1.0]。
 */
float ComputeResidualJammerFactor(const session::ArControlProfile& control_profile,
                                  const session::JammerSourceFact& jammer_source);

/**
 * @brief 计算 legacy 干扰摘要相对接收机热噪声的兼容比值。
 * @param[in] cfg 干扰效果配置。
 * @param[in] jammer_source 单个干扰源事实。
 * @return 无量纲噪声比，已按干扰类型权重与置信度加权。
 */
float ComputeLegacySourceJamToNoiseRatio(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const session::JammerSourceFact& jammer_source);

/**
 * @brief 解析当前环境下的主导干扰语义。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 主导干扰类型；未检测到干扰或无多源事实时返回 kNone，存在并立强类型时返回 kMixed。
 */
config::JammingSemantic ResolveDominantJammingSemantic(
    const session::ArControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 计算当前环境下的轨迹级干扰严重度（0~1）。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 归一化严重度，钳位到 [0, 1]；未检测到干扰或无多源事实时返回 0。
 */
float ComputeTrackLevelJammingSeverity(const session::ArControlProfile& control_profile,
                                       const session::EnvironmentSnapshot& environment_snapshot);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
