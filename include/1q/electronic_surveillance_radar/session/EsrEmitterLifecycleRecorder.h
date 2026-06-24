/**
 * @file EsrEmitterLifecycleRecorder.h
 * @brief 定义 ESR 辐射源观测生命周期记录器。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EMITTER_LIFECYCLE_RECORDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EMITTER_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"

namespace electronic_surveillance_radar {
namespace session {

enum class EsrEmitterLifecycleEventKind {
  kFirstObserved = 0,
  kUpdated = 1,
  kLost = 2,
  kNotObserved = 3
};

enum class EsrEmitterLifecycleReason {
  kNone = 0,
  kNotEmitting = 1,
  kNoMatchedObservation = 2,
  kValidationRejected = 3,
  kCycleNotExecuted = 4,
  kUnknown = 5
};

struct ONEQ_API EsrEmitterLifecycleEvent {
  std::uint32_t cycle_index{0U};
  std::uint64_t emitter_id{0U};
  std::string emitter_name{};
  EsrEmitterLifecycleEventKind kind{EsrEmitterLifecycleEventKind::kUpdated};
  EsrEmitterLifecycleReason reason{EsrEmitterLifecycleReason::kNone};
  std::uint64_t observation_id{0U};
  float confidence{0.0f};
};

struct ONEQ_API EsrEmitterLifecycleRecorderConfig {
  bool emit_not_observed_events{false};
};

class ONEQ_API EsrEmitterLifecycleRecorder {
 public:
  explicit EsrEmitterLifecycleRecorder(
      EsrEmitterLifecycleRecorderConfig config = EsrEmitterLifecycleRecorderConfig{})
      : config_(config) {}

  std::vector<EsrEmitterLifecycleEvent> Update(const EsrCycleInput& input,
                                               const EsrCycleResult& result) {
    std::vector<EsrEmitterLifecycleEvent> events;
    events.reserve(input.scene.size());
    for (const EsrSceneEmitter& emitter : input.scene) {
      AppendEmitterEvents(emitter, result, &events);
    }
    return events;
  }

  void Reset() { states_.clear(); }

 private:
  struct EmitterState {
    bool observed{false};
    std::string emitter_name{};
  };

  void AppendEmitterEvents(const EsrSceneEmitter& emitter, const EsrCycleResult& result,
                           std::vector<EsrEmitterLifecycleEvent>* events) {
    EmitterState& state = states_[emitter.emitter_id];
    const extension::TruthAssociationRecord* association =
        FindAssociation(emitter.emitter_id, result.output_frame.truth_evaluation_output);
    const bool observed_now = result.executed_this_cycle && association != nullptr;
    if (observed_now) {
      EsrEmitterLifecycleEvent event = MakeBaseEvent(emitter, result);
      event.kind = state.observed ? EsrEmitterLifecycleEventKind::kUpdated
                                  : EsrEmitterLifecycleEventKind::kFirstObserved;
      event.reason = EsrEmitterLifecycleReason::kNone;
      event.observation_id = association->observation_id;
      event.confidence = association->confidence;
      events->push_back(event);
      state.observed = true;
      state.emitter_name = emitter.emitter_name;
      return;
    }

    const EsrEmitterLifecycleReason reason = InferReason(emitter, result);
    if (state.observed) {
      EsrEmitterLifecycleEvent event = MakeBaseEvent(emitter, result);
      event.kind = EsrEmitterLifecycleEventKind::kLost;
      event.reason = reason;
      events->push_back(event);
    } else if (config_.emit_not_observed_events) {
      EsrEmitterLifecycleEvent event = MakeBaseEvent(emitter, result);
      event.kind = EsrEmitterLifecycleEventKind::kNotObserved;
      event.reason = reason;
      events->push_back(event);
    }
    state.observed = false;
    state.emitter_name = emitter.emitter_name;
  }

  static EsrEmitterLifecycleEvent MakeBaseEvent(const EsrSceneEmitter& emitter,
                                                const EsrCycleResult& result) {
    EsrEmitterLifecycleEvent event;
    event.cycle_index = result.input_cycle_index;
    event.emitter_id = emitter.emitter_id;
    event.emitter_name = emitter.emitter_name;
    return event;
  }

  static EsrEmitterLifecycleReason InferReason(const EsrSceneEmitter& emitter,
                                               const EsrCycleResult& result) {
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

  static const extension::TruthAssociationRecord* FindAssociation(
      std::uint64_t emitter_id, const extension::TruthEvaluationFrame& frame) {
    for (const extension::TruthAssociationRecord& association : frame.associations) {
      if (association.matched && association.truth_emitter_id == emitter_id) {
        return &association;
      }
    }
    return nullptr;
  }

  EsrEmitterLifecycleRecorderConfig config_;
  std::unordered_map<std::uint64_t, EmitterState> states_;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EMITTER_LIFECYCLE_RECORDER_H_
