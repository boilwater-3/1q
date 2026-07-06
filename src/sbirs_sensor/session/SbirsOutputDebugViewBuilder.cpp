#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"

namespace sbirs_sensor {
namespace session {
namespace {

const output::SbirsDetectionRecord* FindRecord(std::uint64_t detection_id,
                                               const SbirsOutputFrame& frame) {
  for (const output::SbirsDetectionRecord& record : frame.detections) {
    if (record.detection_id == detection_id) {
      return &record;
    }
  }
  return nullptr;
}

const attribution::SbirsDetectionAttributionRecord* FindAttribution(
    std::uint64_t target_id, const attribution::SbirsDetectionAttributionRecordList& attributions) {
  for (const attribution::SbirsDetectionAttributionRecord& attribution : attributions) {
    if (attribution.target_id == target_id) {
      return &attribution;
    }
  }
  return nullptr;
}

SbirsDebugTargetState BuildTargetState(const SbirsSceneTarget& target,
                                       const SbirsCycleResult& result) {
  SbirsDebugTargetState state;
  state.target_id = target.target_id;
  state.target_name = target.target_name;
  state.present_in_input = true;
  if (!result.executed_this_cycle) {
    state.status = SbirsDebugTargetStatus::kCycleNotExecuted;
    return state;
  }

  const attribution::SbirsDetectionAttributionRecord* attribution =
      FindAttribution(target.target_id, result.detection_attributions);
  if (attribution == nullptr) {
    state.status = SbirsDebugTargetStatus::kNotInOutput;
    return state;
  }

  const output::SbirsDetectionRecord* record =
      FindRecord(attribution->detection_id, result.output_frame);
  if (record == nullptr) {
    state.status = SbirsDebugTargetStatus::kNotInOutput;
    return state;
  }

  state.has_raw_output_record = true;
  state.detected = record->detected;
  state.used_truth_assist = attribution->used_truth_assist;
  state.estimated_range_m = attribution->estimated_range_m;
  state.azimuth_deg = record->azimuth_deg;
  state.elevation_deg = record->elevation_deg;
  state.infrared_snr_linear = record->infrared_snr_linear;
  state.observation_stage = record->observation_stage;
  state.status = record->detected ? SbirsDebugTargetStatus::kDetected
                                  : SbirsDebugTargetStatus::kObservedBelowThreshold;
  return state;
}

}  // namespace

SbirsOutputDebugView SbirsOutputDebugViewBuilder::Build(const SbirsCycleInput& input,
                                                        const SbirsCycleResult& result) {
  SbirsOutputDebugView view;
  view.input_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.output_frame.cycle_index;
  view.executed_this_cycle = result.executed_this_cycle;
  view.reused_previous_output = result.reused_previous_output;
  view.has_validation_error = result.has_validation_error;
  view.abort_reason = result.abort_reason;
  view.targets.reserve(input.scene.size());
  for (const SbirsSceneTarget& target : input.scene) {
    view.targets.push_back(BuildTargetState(target, result));
  }
  return view;
}

}  // namespace session
}  // namespace sbirs_sensor
