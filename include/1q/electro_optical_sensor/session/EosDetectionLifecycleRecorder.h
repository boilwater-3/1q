/**
 * @file EosDetectionLifecycleRecorder.h
 * @brief 定义 EOS 目标探测生命周期记录器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace session {

enum class EosDetectionLifecycleEventKind {
  kFirstDetected = 0,
  kUpdated = 1,
  kLost = 2,
  kNotDetected = 3
};

enum class EosDetectionLifecycleReason {
  kNone = 0,
  kOutOfFov = 1,
  kBelowSnrThreshold = 2,
  kValidationRejected = 3,
  kCycleNotExecuted = 4,
  kUnknown = 5
};

struct ONEQ_API EosDetectionLifecycleEvent {
  std::uint32_t cycle_index{0U};
  std::uint64_t target_id{0U};
  std::string target_name{};
  EosDetectionLifecycleEventKind kind{EosDetectionLifecycleEventKind::kUpdated};
  EosDetectionLifecycleReason reason{EosDetectionLifecycleReason::kNone};
  float fused_snr_db{0.0f};
  float range_m{0.0f};
};

struct ONEQ_API EosDetectionLifecycleRecorderConfig {
  bool emit_not_detected_events{false};
};

class ONEQ_API EosDetectionLifecycleRecorder {
 public:
  explicit EosDetectionLifecycleRecorder(
      EosDetectionLifecycleRecorderConfig config = EosDetectionLifecycleRecorderConfig{})
      : config_(config) {}

  std::vector<EosDetectionLifecycleEvent> Update(const EosCycleInput& input,
                                                 const EosCycleResult& result) {
    std::vector<EosDetectionLifecycleEvent> events;
    events.reserve(input.scene.size());
    for (const EosSceneTarget& target : input.scene) {
      AppendTargetEvents(target, result, &events);
    }
    return events;
  }

  void Reset() { states_.clear(); }

 private:
  struct TargetState {
    bool detected{false};
    std::string target_name{};
  };

  void AppendTargetEvents(const EosSceneTarget& target, const EosCycleResult& result,
                          std::vector<EosDetectionLifecycleEvent>* events) {
    TargetState& state = states_[target.target_id];
    const attribution::EosDetectionAttributionRecord* attribution =
        FindAttribution(target.target_id, result.detection_attributions);
    const output::EosDetectionRecord* record =
        attribution == nullptr ? nullptr
                               : FindRecord(attribution->detection_id, result.output_frame);
    const bool detected_now = result.executed_this_cycle && record != nullptr && record->detected;
    if (detected_now) {
      EosDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = state.detected ? EosDetectionLifecycleEventKind::kUpdated
                                  : EosDetectionLifecycleEventKind::kFirstDetected;
      event.reason = EosDetectionLifecycleReason::kNone;
      event.fused_snr_db = record->fused_snr_db;
      event.range_m = record->range_m;
      events->push_back(event);
      state.detected = true;
      state.target_name = target.target_name;
      return;
    }

    const EosDetectionLifecycleReason reason = InferReason(result, record);
    if (state.detected) {
      EosDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = EosDetectionLifecycleEventKind::kLost;
      event.reason = reason;
      if (record != nullptr) {
        event.fused_snr_db = record->fused_snr_db;
        event.range_m = record->range_m;
      }
      events->push_back(event);
    } else if (config_.emit_not_detected_events) {
      EosDetectionLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = EosDetectionLifecycleEventKind::kNotDetected;
      event.reason = reason;
      if (record != nullptr) {
        event.fused_snr_db = record->fused_snr_db;
        event.range_m = record->range_m;
      }
      events->push_back(event);
    }
    state.detected = false;
    state.target_name = target.target_name;
  }

  static EosDetectionLifecycleEvent MakeBaseEvent(const EosSceneTarget& target,
                                                  const EosCycleResult& result) {
    EosDetectionLifecycleEvent event;
    event.cycle_index = result.input_cycle_index;
    event.target_id = target.target_id;
    event.target_name = target.target_name;
    return event;
  }

  static EosDetectionLifecycleReason InferReason(const EosCycleResult& result,
                                                 const output::EosDetectionRecord* record) {
    if (result.has_validation_error) {
      return EosDetectionLifecycleReason::kValidationRejected;
    }
    if (!result.executed_this_cycle) {
      return EosDetectionLifecycleReason::kCycleNotExecuted;
    }
    if (record == nullptr) {
      return EosDetectionLifecycleReason::kOutOfFov;
    }
    if (!record->detected) {
      return EosDetectionLifecycleReason::kBelowSnrThreshold;
    }
    return EosDetectionLifecycleReason::kUnknown;
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

  EosDetectionLifecycleRecorderConfig config_;
  std::unordered_map<std::uint64_t, TargetState> states_;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DETECTION_LIFECYCLE_RECORDER_H_
