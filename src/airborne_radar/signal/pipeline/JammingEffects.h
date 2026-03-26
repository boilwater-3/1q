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

bool HasMultiSourceJammingFacts(const environment::EnvironmentSnapshot& environment_snapshot);

float ComputeResidualJammerFactor(const common::RadarControlProfile& control_profile,
                                  const environment::JammerSourceFact& jammer_source);

float ComputeHeuristicSourcePenaltyDb(const JammingEffectsConfig& cfg,
                                      const environment::JammerSourceFact& jammer_source);

float ComputeHeuristicJammingPenaltyDb(const JammingEffectsConfig& cfg,
                                       const environment::EnvironmentSnapshot& environment_snapshot);

float ComputePhysicalSourceJamContributionW(const JammingEffectsConfig& cfg,
                                            const environment::JammerSourceFact& jammer_source);

float ComputeMeasurementCovarianceInflation(
    const JammingEffectsConfig& cfg, const common::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

common::JammingSemantic ResolveDominantJammingSemantic(
    const common::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot);

float ComputeTrackLevelJammingSeverity(const common::RadarControlProfile& control_profile,
                                       const environment::EnvironmentSnapshot& environment_snapshot);

void ApplyEnvironmentJammingFactsToRuntimeConfig(
    const JammingEffectsConfig& cfg, const common::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot,
    SignalPipelineConfig* runtime_config);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_JAMMING_EFFECTS_H_
