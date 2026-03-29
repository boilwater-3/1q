/**
 * @file JammingEffects.h
 * @brief 定义 SignalPipeline 干扰效应建模的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 判断环境快照中是否携带多源干扰事实。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 至少存在一个显式干扰源事实时返回 true。
 */
bool HasMultiSourceJammingFacts(const environment::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 计算控制 profile 对单个干扰源的残余作用系数。
 * @param[in] control_profile 当前控制真值。
 * @param[in] jammer_source 单个干扰源事实。
 * @return ECCM 动作作用后的残余干扰系数。
 */
float ComputeResidualJammerFactor(const common::control::RadarControlProfile& control_profile,
                                  const environment::JammerSourceFact& jammer_source);

/**
 * @brief 计算单个干扰源对经验检测的惩罚项。
 * @param[in] cfg 干扰效应配置。
 * @param[in] jammer_source 单个干扰源事实。
 * @return 对经验检测链路的惩罚量（dB）。
 */
float ComputeHeuristicSourcePenaltyDb(const JammingEffectsConfig& cfg,
                                      const environment::JammerSourceFact& jammer_source);

/**
 * @brief 计算多源干扰对经验检测的总惩罚项。
 * @param[in] cfg 干扰效应配置。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 多源干扰的累计惩罚量（dB）。
 */
float ComputeHeuristicJammingPenaltyDb(
    const JammingEffectsConfig& cfg, const environment::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 计算单个干扰源的等效 jam 噪声功率贡献。
 * @param[in] cfg 干扰效应配置。
 * @param[in] jammer_source 单个干扰源事实。
 * @return 等效 jam 噪声功率贡献（W）。
 */
float ComputePhysicalSourceJamContributionW(const JammingEffectsConfig& cfg,
                                            const environment::JammerSourceFact& jammer_source);

/**
 * @brief 计算多源干扰对量测协方差的膨胀因子。
 * @param[in] cfg 干扰效应配置。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 量测协方差膨胀因子。
 */
float ComputeMeasurementCovarianceInflation(
    const JammingEffectsConfig& cfg, const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 汇总环境快照的主导干扰语义。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 当前周期主导干扰语义。
 */
common::utils::JammingSemantic ResolveDominantJammingSemantic(
    const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 汇总环境快照的轨迹级残余干扰强度。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 当前周期轨迹级残余干扰强度，范围 [0, 1]。
 */
float ComputeTrackLevelJammingSeverity(
    const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 把多源干扰事实传播到关联/跟踪运行时配置。
 * @param[in] cfg 干扰效应配置。
 * @param[in] control_profile 当前控制真值。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @param[in,out] runtime_config 待调整的运行时配置。
 */
void ApplyEnvironmentJammingFactsToRuntimeConfig(
    const JammingEffectsConfig& cfg, const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot,
    SignalPipelineConfig* runtime_config);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
