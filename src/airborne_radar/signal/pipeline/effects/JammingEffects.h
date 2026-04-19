/**
 * @file JammingEffects.h
 * @brief 定义 SignalPipeline 干扰效应建模的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

bool HasMultiSourceJammingFacts(const environment::EnvironmentSnapshot& environment_snapshot);

float ComputeResidualJammerFactor(const extension::control::RadarControlProfile& control_profile,
                                  const environment::JammerSourceFact& jammer_source);

float ComputeHeuristicSourcePenaltyDb(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
                                      const environment::JammerSourceFact& jammer_source);

float ComputeHeuristicJammingPenaltyDb(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const environment::EnvironmentSnapshot& environment_snapshot);

float ComputePhysicalSourceJamContributionW(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
                                            const environment::JammerSourceFact& jammer_source);

float ComputeMeasurementCovarianceInflation(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const extension::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

model::JammingSemantic ResolveDominantJammingSemantic(
    const extension::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

float ComputeTrackLevelJammingSeverity(
    const extension::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

void ApplyEnvironmentJammingFactsToRuntimeConfig(
    const extension::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot, ExecutionConfig* runtime_config);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
