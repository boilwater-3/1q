#include "airborne_radar/decision/evaluators/SurvivabilityEvaluator.h"

#include "airborne_radar/decision/evaluators/SurvivabilityEvaluatorHelpers.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {
namespace evaluators {

void SurvivabilityEvaluator::Evaluate(const common::model::DecisionInputFrame& input_frame,
                                      pipeline::TacticalStateStore& state_store,
                                      pipeline::TacticalEvaluationState& evaluation_state) const {
  (void)state_store;
  if (!evaluation_state.threat_assessment_phase_done) {
    PROJECT_LOG_ERROR(
        "[SurvivabilityEvaluator] invalid evaluator order: ThreatAssessmentEvaluator must run first.");
    evaluation_state.should_enable_eccm = false;
    return;
  }
  bool should_enable_eccm = evaluation_state.eccm_source_info.has_jamming_signal ||
                            input_frame.environment_jamming_detected;

  evaluation_state.should_enable_eccm = should_enable_eccm;
  if (!should_enable_eccm) {
    PROJECT_LOG_INFO(
        "[SurvivabilityEvaluator] Environment is clear. Continuing nominal operation.");
    return;
  }

  internal::AppendEccmProposals(evaluation_state.eccm_source_info,
                                input_frame.association_quality_info,
                                input_frame.environment_jamming_detected,
                                false, &evaluation_state.proposals);
  PROJECT_LOG_INFO("[SurvivabilityEvaluator] Active jamming detected. Appending ECCM proposals.");
}

}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar
