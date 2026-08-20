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
  // 输入实体回填（规则 12）：目标角度/距离为 input 侧真值，无论是否检测均
  // 可见（检测记录存在时下方以记录观测值覆盖）；供调用方人读/结构化落盘。
  state.range_m = target.range_m;
  state.azimuth_deg = target.azimuth_deg;
  state.elevation_deg = target.elevation_deg;
  if (result.status != EosCycleStatus::kCompleted) {
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
  view.executed_this_cycle = (result.status == EosCycleStatus::kCompleted);
  view.abort_reason = result.abort_reason;
  view.issues = result.issues;
  view.targets.reserve(input.scene.size());
  for (const EosSceneTarget& target : input.scene) {
    view.targets.push_back(BuildTargetState(target, result));
  }
  return view;
}

}  // namespace session
}  // namespace electro_optical_sensor
