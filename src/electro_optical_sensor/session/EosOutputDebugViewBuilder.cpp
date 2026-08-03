#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"

#include "1q/electro_optical_sensor/session/EosCycleInput.h"

namespace electro_optical_sensor {
namespace session {

namespace {

const output::EosDetectionRecord* FindRecord(std::uint64_t detection_id, const EosOutputFrame& frame) {
  for (const output::EosDetectionRecord& record : frame.detections) {
    if (record.detection_id == detection_id) {
      return &record;
    }
  }
  return nullptr;
}

const attribution::EosDetectionAttributionRecord* FindAttribution(
    std::uint64_t target_id, const attribution::EosDetectionAttributionRecordList& attributions) {
  for (const attribution::EosDetectionAttributionRecord& attribution : attributions) {
    if (attribution.target_id == target_id) {
      return &attribution;
    }
  }
  return nullptr;
}

EosDebugTargetState BuildTargetState(const EosSceneTarget& target, const EosCycleResult& result) {
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
  const output::EosDetectionRecord* record = FindRecord(attribution->detection_id, result.output_frame);
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

}  // namespace

EosOutputDebugView EosOutputDebugViewBuilder::Build(const EosCycleInput& input,
                                                     const EosCycleResult& result) {
  EosOutputDebugView view;
  view.input_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.output_frame.cycle_index;
  view.executed_this_cycle = result.executed_this_cycle;
  view.has_validation_error = result.has_validation_error;
  view.abort_reason = result.abort_reason;
  view.diagnostics = result.diagnostics;
  view.targets.reserve(input.scene.size());
  for (const EosSceneTarget& target : input.scene) {
    view.targets.push_back(BuildTargetState(target, result));
  }
  return view;
}

}  // namespace session
}  // namespace electro_optical_sensor
