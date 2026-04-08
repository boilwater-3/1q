/**
 * @file SurvivabilityEvaluatorHelpers.h
 * @brief 生存性评估器内部 ECCM proposal 生成辅助声明。
 */

#ifndef AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_HELPERS_H_
#define AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_HELPERS_H_

#include <vector>

#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "airborne_radar/decision/pipeline/TacticalEvaluation.h"

namespace airborne_radar {
namespace decision {
namespace evaluators {
namespace internal {

void AppendEccmProposals(const model::EccmSourceInfo& source_info,
                         const model::AssociationQualityInfo& association_quality_info,
                         bool environment_jamming_detected, bool hold_only,
                         std::vector<pipeline::TacticalProposal>* proposals);

}  // namespace internal
}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_HELPERS_H_
