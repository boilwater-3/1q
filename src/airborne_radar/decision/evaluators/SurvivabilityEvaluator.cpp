#include "airborne_radar/decision/evaluators/SurvivabilityEvaluator.h"

#include <cassert>

#include "airborne_radar/decision/evaluators/SurvivabilityEvaluatorHelpers.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {
namespace evaluators {

SurvivabilityEvaluator::SurvivabilityEvaluator(SurvivabilityEvaluatorConfig config)
    : config_(config) {}

void SurvivabilityEvaluator::Evaluate(const common::model::DecisionInputFrame& input_frame,
                                      pipeline::TacticalStateStore& state_store,
                                      pipeline::TacticalEvaluationState& evaluation_state) const {
  assert(evaluation_state.threat_assessment_phase_done &&
         "SurvivabilityEvaluator must run after ThreatAssessmentEvaluator");
  bool should_enable_eccm = evaluation_state.eccm_source_info.has_jamming_signal ||
                            input_frame.environment_jamming_detected;
  const bool has_current_eccm_evidence = should_enable_eccm;
  if (!should_enable_eccm && state_store.eccm_hold_cycles_remaining > 0U) {
    should_enable_eccm = true;
    --state_store.eccm_hold_cycles_remaining;
  }

  evaluation_state.should_enable_eccm = should_enable_eccm;
  if (!should_enable_eccm) {
    PROJECT_LOG_INFO(
        "[SurvivabilityEvaluator] Environment is clear. Continuing nominal operation.");
    return;
  }

  if (has_current_eccm_evidence) {
    state_store.eccm_hold_cycles_remaining = config_.eccm_hold_cycles;
  }
  internal::AppendEccmProposals(evaluation_state.eccm_source_info,
                                input_frame.association_quality_info,
                                input_frame.environment_jamming_detected,
                                !has_current_eccm_evidence, &evaluation_state.proposals);
  PROJECT_LOG_INFO("[SurvivabilityEvaluator] Active jamming detected. Appending ECCM proposals.");
}

}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar
