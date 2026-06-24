/**
 * @file EosOutputDebugView.h
 * @brief 定义 EOS 输出开发调试视图构建工具。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_DEBUG_VIEW_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_DEBUG_VIEW_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace session {

enum class EosDebugTargetStatus {
  kDetected = 0,
  kObservedBelowThreshold = 1,
  kNotInOutput = 2,
  kCycleNotExecuted = 3
};

struct ONEQ_API EosDebugTargetState {
  std::uint64_t target_id{0U};
  std::string target_name{};
  EosDebugTargetStatus status{EosDebugTargetStatus::kNotInOutput};
  bool present_in_input{false};
  bool has_raw_output_record{false};
  bool detected{false};
  float range_m{0.0f};
  float azimuth_deg{0.0f};
  float elevation_deg{0.0f};
  float fused_snr_db{0.0f};
};

struct ONEQ_API EosOutputDebugView {
  std::uint32_t input_cycle_index{0U};
  std::uint32_t output_cycle_index{0U};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  bool has_validation_error{false};
  extension::EosPipelineAbortReason abort_reason{extension::EosPipelineAbortReason::kNone};
  std::vector<EosDebugTargetState> targets{};
};

class ONEQ_API EosOutputDebugViewBuilder {
 public:
  static EosOutputDebugView Build(const EosCycleInput& input, const EosCycleResult& result) {
    EosOutputDebugView view;
    view.input_cycle_index = result.input_cycle_index;
    view.output_cycle_index = result.output_frame.cycle_index;
    view.executed_this_cycle = result.executed_this_cycle;
    view.reused_previous_output = result.reused_previous_output;
    view.has_validation_error = result.has_validation_error;
    view.abort_reason = result.abort_reason;
    view.targets.reserve(input.scene.size());
    for (const EosSceneTarget& target : input.scene) {
      view.targets.push_back(BuildTargetState(target, result));
    }
    return view;
  }

 private:
  static EosDebugTargetState BuildTargetState(const EosSceneTarget& target,
                                              const EosCycleResult& result) {
    EosDebugTargetState state;
    state.target_id = target.target_id;
    state.target_name = target.target_name;
    state.present_in_input = true;
    if (!result.executed_this_cycle) {
      state.status = EosDebugTargetStatus::kCycleNotExecuted;
      return state;
    }
    const attribution::EosDetectionAttributionRecord* attribution =
        FindAttribution(target.target_id, result.detection_attributions);
    if (attribution == nullptr) {
      state.status = EosDebugTargetStatus::kNotInOutput;
      return state;
    }
    const output::EosDetectionRecord* record =
        FindRecord(attribution->detection_id, result.output_frame);
    if (record == nullptr) {
      state.status = EosDebugTargetStatus::kNotInOutput;
      return state;
    }
    state.has_raw_output_record = true;
    state.detected = record->detected;
    state.range_m = record->range_m;
    state.azimuth_deg = record->azimuth_deg;
    state.elevation_deg = record->elevation_deg;
    state.fused_snr_db = record->fused_snr_db;
    state.status = record->detected ? EosDebugTargetStatus::kDetected
                                    : EosDebugTargetStatus::kObservedBelowThreshold;
    return state;
  }

  static const output::EosDetectionRecord* FindRecord(std::uint64_t detection_id,
                                                      const EosOutputFrame& frame) {
    for (const output::EosDetectionRecord& record : frame.detections) {
      if (record.detection_id == detection_id) {
        return &record;
      }
    }
    return nullptr;
  }

  static const attribution::EosDetectionAttributionRecord* FindAttribution(
      std::uint64_t target_id, const attribution::EosDetectionAttributionRecordList& attributions) {
    for (const attribution::EosDetectionAttributionRecord& attribution : attributions) {
      if (attribution.target_id == target_id) {
        return &attribution;
      }
    }
    return nullptr;
  }
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_DEBUG_VIEW_H_
