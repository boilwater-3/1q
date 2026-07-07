/**
 * @file JammingEffects.h
 * @brief 定义 SignalPipeline 干扰效应建模的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_

#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
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
 * @brief 计算在当前控制真值下单个干扰源的残余因子（0~1，越小表示抑制越强）。
 * @param[in] control_profile 当前控制真值（ECCM/自适应波束等）。
 * @param[in] jammer_source 单个干扰源事实。
 * @return 残余因子，已钳位到 [kResidualFactorMin, 1.0]。
 */
float ComputeResidualJammerFactor(const session::ArControlProfile& control_profile,
                                  const session::JammerSourceFact& jammer_source);

/**
 * @brief 启发式计算单个干扰源的检测惩罚项（单位：dB）。
 * @param[in] cfg 干扰效果配置。
 * @param[in] jammer_source 单个干扰源事实。
 * @return 经置信度加权后的源惩罚 dB。
 */
float ComputeHeuristicSourcePenaltyDb(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
                                      const session::JammerSourceFact& jammer_source);

/**
 * @brief 启发式累加所有干扰源的检测惩罚项（单位：dB）。
 * @param[in] cfg 干扰效果配置。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 全部干扰源惩罚之和；无多源干扰时返回 0。
 */
float ComputeHeuristicJammingPenaltyDb(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const session::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 物理化计算单个干扰源对接收机的噪声功率贡献（单位：W）。
 * @param[in] cfg 干扰效果配置。
 * @param[in] jammer_source 单个干扰源事实。
 * @return 干扰功率贡献（W），已按干扰类型权重与置信度加权。
 */
float ComputePhysicalSourceJamContributionW(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
                                            const session::JammerSourceFact& jammer_source);

/**
 * @brief 计算当前干扰环境对量测协方差的膨胀因子（≥1）。
 * @param[in] cfg 干扰效果配置。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 协方差膨胀因子，已钳位到 [1.0, covariance_inflation_max]；无多源干扰时返回 1.0。
 */
float ComputeMeasurementCovarianceInflation(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const session::ArControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot);

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
float ComputeTrackLevelJammingSeverity(
    const session::ArControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 将干扰事实叠加到运行时配置（关联代价、跟踪噪声、量测噪声等缩放因子）。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @param[in,out] runtime_config 待就地修改的运行时配置。
 * @note runtime_config 为 nullptr 或无多源干扰事实时直接返回。
 */
void ApplyEnvironmentJammingFactsToRuntimeConfig(
    const session::ArControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot, ExecutionConfig* runtime_config);


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
