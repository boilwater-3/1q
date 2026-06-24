#include "1q/electronic_surveillance_radar/session/EsrOutputDebugView.h"

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

const extension::TruthAssociationRecord* FindAssociation(
    std::uint64_t emitter_id, const extension::TruthEvaluationFrame& frame) {
  for (const extension::TruthAssociationRecord& association : frame.associations) {
    if (association.matched && association.truth_emitter_id == emitter_id) {
      return &association;
    }
  }
  return nullptr;
}

EsrDebugEmitterState BuildEmitterState(const EsrSceneEmitter& emitter, const EsrCycleResult& result) {
  EsrDebugEmitterState state;
  state.emitter_id = emitter.emitter_id;
  state.emitter_name = emitter.emitter_name;
  state.present_in_input = true;
  if (!result.executed_this_cycle) {
    state.status = EsrDebugEmitterStatus::kCycleNotExecuted;
    return state;
  }
  if (!emitter.is_emitting) {
    state.status = EsrDebugEmitterStatus::kNotEmitting;
    return state;
  }
  const extension::TruthAssociationRecord* association =
      FindAssociation(emitter.emitter_id, result.output_frame.truth_evaluation_output);
  if (association == nullptr) {
    state.status = EsrDebugEmitterStatus::kNotObserved;
    return state;
  }
  state.status = EsrDebugEmitterStatus::kObserved;
  state.matched_observation = true;
  state.observation_id = association->observation_id;
  state.confidence = association->confidence;
  return state;
}

}  // namespace

EsrOutputDebugView EsrOutputDebugViewBuilder::Build(const EsrCycleInput& input,
                                                    const EsrCycleResult& result) {
  EsrOutputDebugView view;
  view.input_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.output_frame.cycle_index;
  view.executed_this_cycle = result.executed_this_cycle;
  view.reused_previous_output = result.reused_previous_output;
  view.has_validation_error = result.has_validation_error;
  view.abort_reason = result.abort_reason;
  view.emitters.reserve(input.scene.size());
  for (const EsrSceneEmitter& emitter : input.scene) {
    view.emitters.push_back(BuildEmitterState(emitter, result));
  }
  return view;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
