// Copyright 2026. All Rights Reserved.
//
// Description: TacticalCoordinator 的默认实现。

#include "1q/airborne_radar/decision/TacticalCoordinator.h"

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {

TacticalCoordinator::TacticalCoordinator(
    const environment::database::IFeatureRepository* feature_repository)
    : threat_assessment_evaluator_(feature_repository),
      emission_control_evaluator_(),
      survivability_evaluator_() {}

TacticalDecisionResult TacticalCoordinator::Evaluate(
    const common::DecisionInputFrame& input_frame,
    TacticalStateStore& state_store) {
  TacticalEvaluationState evaluation_state;
  evaluation_state.eccm_source_info.has_jamming_signal =
      input_frame.environment_jamming_detected;

  threat_assessment_evaluator_.Evaluate(input_frame, state_store,
                                        evaluation_state);
  emission_control_evaluator_.Evaluate(input_frame, state_store,
                                       evaluation_state);
  survivability_evaluator_.Evaluate(input_frame, state_store,
                                    evaluation_state);

  TacticalDecisionResult result;
  result.target_classification_result.reserve(
      evaluation_state.target_classification_result.size());
  for (std::size_t i = 0;
       i < evaluation_state.target_classification_result.size(); ++i) {
    result.target_classification_result.push_back(common::TargetCategory(
        evaluation_state.target_classification_result[i].target_type));
  }
  result.proposals = evaluation_state.proposals;
  if (evaluation_state.should_enable_eccm) {
    result.selected_mode = TacticalMode::kProtectedEmission;
  } else if (evaluation_state.should_reduce_power) {
    result.selected_mode = TacticalMode::kThreatResponse;
  } else {
    result.selected_mode = TacticalMode::kBaseline;
  }

  state_store.current_mode = result.selected_mode;
  state_store.last_classification_labels.clear();
  state_store.last_classification_labels.reserve(
      result.target_classification_result.size());
  for (std::size_t i = 0; i < result.target_classification_result.size(); ++i) {
    state_store.last_classification_labels.push_back(
        result.target_classification_result[i].target_type);
  }
  state_store.last_decision_summary = evaluation_state.should_enable_eccm
                                          ? "protected-emission"
                                          : (evaluation_state.should_reduce_power
                                                 ? "threat-response"
                                                 : "baseline");

  spdlog::debug(
      "[TacticalCoordinator] cycle_index={} tracks={} mode={} proposals={}",
      input_frame.cycle_index, input_frame.tracks.size(),
      static_cast<int>(result.selected_mode), result.proposals.size());
  return result;
}

} // namespace decision
} // namespace airborne_radar
