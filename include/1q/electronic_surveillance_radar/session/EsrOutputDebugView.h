/**
 * @file EsrOutputDebugView.h
 * @brief 定义 ESR 输出开发调试视图构建工具。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_DEBUG_VIEW_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_DEBUG_VIEW_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"

namespace electronic_surveillance_radar {
namespace session {

enum class EsrDebugEmitterStatus {
  kObserved = 0,
  kNotObserved = 1,
  kNotEmitting = 2,
  kCycleNotExecuted = 3
};

struct ONEQ_API EsrDebugEmitterState {
  std::uint64_t emitter_id{0U};
  std::string emitter_name{};
  EsrDebugEmitterStatus status{EsrDebugEmitterStatus::kNotObserved};
  bool present_in_input{false};
  bool matched_observation{false};
  std::uint64_t observation_id{0U};
  float confidence{0.0f};
};

struct ONEQ_API EsrOutputDebugView {
  std::uint32_t input_cycle_index{0U};
  std::uint32_t output_cycle_index{0U};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  bool has_validation_error{false};
  extension::EsrPipelineAbortReason abort_reason{extension::EsrPipelineAbortReason::kNone};
  std::vector<EsrDebugEmitterState> emitters{};
};

class ONEQ_API EsrOutputDebugViewBuilder {
 public:
  static EsrOutputDebugView Build(const EsrCycleInput& input, const EsrCycleResult& result) {
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

 private:
  static EsrDebugEmitterState BuildEmitterState(const EsrSceneEmitter& emitter,
                                                const EsrCycleResult& result) {
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

  static const extension::TruthAssociationRecord* FindAssociation(
      std::uint64_t emitter_id, const extension::TruthEvaluationFrame& frame) {
    for (const extension::TruthAssociationRecord& association : frame.associations) {
      if (association.matched && association.truth_emitter_id == emitter_id) {
        return &association;
      }
    }
    return nullptr;
  }
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_DEBUG_VIEW_H_
