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

bool HasMultiSourceJammingFacts(const session::EnvironmentSnapshot& environment_snapshot);

float ComputeResidualJammerFactor(const session::RadarControlProfile& control_profile,
                                  const session::JammerSourceFact& jammer_source);

float ComputeHeuristicSourcePenaltyDb(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
                                      const session::JammerSourceFact& jammer_source);

float ComputeHeuristicJammingPenaltyDb(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const session::EnvironmentSnapshot& environment_snapshot);

float ComputePhysicalSourceJamContributionW(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
                                            const session::JammerSourceFact& jammer_source);

float ComputeMeasurementCovarianceInflation(
    const ::airborne_radar::config::execution::JammingEffectsConfig& cfg,
    const session::RadarControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot);

config::JammingSemantic ResolveDominantJammingSemantic(
    const session::RadarControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot);

float ComputeTrackLevelJammingSeverity(
    const session::RadarControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot);

void ApplyEnvironmentJammingFactsToRuntimeConfig(
    const session::RadarControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot, ExecutionConfig* runtime_config);


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
