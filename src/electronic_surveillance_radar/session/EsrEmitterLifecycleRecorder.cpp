#include "1q/electronic_surveillance_radar/session/EsrEmitterLifecycleRecorder.h"

#include <unordered_map>
#include <utility>

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

struct EmitterState {
  bool observed{false};
  std::string emitter_name{};
};

const session::TruthAssociationRecord* FindAssociation(
    std::uint64_t emitter_id, const session::TruthEvaluationFrame& frame) {
  for (const session::TruthAssociationRecord& association : frame.associations) {
    if (association.matched && association.truth_emitter_id == emitter_id) {
      return &association;
    }
  }
  return nullptr;
}

EsrEmitterLifecycleReason InferReason(const EsrSceneEmitter& emitter, const EsrCycleResult& result) {
  if (result.has_validation_error) {
    return EsrEmitterLifecycleReason::kValidationRejected;
  }
  if (!result.executed_this_cycle) {
    return EsrEmitterLifecycleReason::kCycleNotExecuted;
  }
  if (!emitter.is_emitting) {
    return EsrEmitterLifecycleReason::kNotEmitting;
  }
  return EsrEmitterLifecycleReason::kNoMatchedObservation;
}

EsrEmitterLifecycleEvent MakeBaseEvent(const EsrSceneEmitter& emitter, const EsrCycleResult& result) {
  EsrEmitterLifecycleEvent event;
  event.cycle_index = result.input_cycle_index;
  event.emitter_id = emitter.emitter_id;
  event.emitter_name = emitter.emitter_name;
  return event;
}

}  // namespace

struct EsrEmitterLifecycleRecorder::Impl {
  EsrEmitterLifecycleRecorderConfig config;
  std::unordered_map<std::uint64_t, EmitterState> states;
};

EsrEmitterLifecycleRecorder::EsrEmitterLifecycleRecorder(EsrEmitterLifecycleRecorderConfig config)
    : impl_(new Impl{config, {}}) {}

EsrEmitterLifecycleRecorder::~EsrEmitterLifecycleRecorder() = default;

EsrEmitterLifecycleRecorder::EsrEmitterLifecycleRecorder(EsrEmitterLifecycleRecorder&&) noexcept = default;
EsrEmitterLifecycleRecorder& EsrEmitterLifecycleRecorder::operator=(EsrEmitterLifecycleRecorder&&) noexcept =
    default;

std::vector<EsrEmitterLifecycleEvent> EsrEmitterLifecycleRecorder::Update(const EsrCycleInput& input,
                                                                           const EsrCycleResult& result) {
  std::vector<EsrEmitterLifecycleEvent> events;
  events.reserve(input.scene.size());
  for (const EsrSceneEmitter& emitter : input.scene) {
    EmitterState& state = impl_->states[emitter.emitter_id];
    const session::TruthAssociationRecord* association =
        FindAssociation(emitter.emitter_id, result.output_frame.truth_evaluation_output);
    const bool observed_now = result.executed_this_cycle && association != nullptr;
    if (observed_now) {
      EsrEmitterLifecycleEvent event = MakeBaseEvent(emitter, result);
      event.kind = state.observed ? EsrEmitterLifecycleEventKind::kUpdated
                                  : EsrEmitterLifecycleEventKind::kFirstObserved;
      event.reason = EsrEmitterLifecycleReason::kNone;
      event.observation_id = association->observation_id;
      event.confidence = association->confidence;
      events.push_back(event);
      state.observed = true;
      state.emitter_name = emitter.emitter_name;
      continue;
    }

    const EsrEmitterLifecycleReason reason = InferReason(emitter, result);
    if (state.observed) {
      EsrEmitterLifecycleEvent event = MakeBaseEvent(emitter, result);
      event.kind = EsrEmitterLifecycleEventKind::kLost;
      event.reason = reason;
      events.push_back(event);
    } else if (impl_->config.emit_not_observed_events) {
      EsrEmitterLifecycleEvent event = MakeBaseEvent(emitter, result);
      event.kind = EsrEmitterLifecycleEventKind::kNotObserved;
      event.reason = reason;
      events.push_back(event);
    }
    state.observed = false;
    state.emitter_name = emitter.emitter_name;
  }
  return events;
}

void EsrEmitterLifecycleRecorder::Reset() { impl_->states.clear(); }

}  // namespace session
}  // namespace electronic_surveillance_radar
