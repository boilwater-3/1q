/**
 * @file SbirsDetectionLifecycleRecorder.h
 * @brief 定义 SBIRS-inspired 目标探测生命周期记录器。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_DETECTION_LIFECYCLE_RECORDER_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_DETECTION_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

struct SbirsCycleInput;

enum class ONEQ_API SbirsDetectionLifecycleEventKind {
  kFirstDetected = 0,
  kUpdated,
  kLost,
  kNotDetected
};

enum class ONEQ_API SbirsDetectionLifecycleReason {
  kNone = 0,
  kOutOfFieldOfView,
  kBelowSnrThreshold,
  kValidationRejected,
  kCycleNotExecuted,
  kTargetMissingFromInput,
  kUnknown
};

struct ONEQ_API SbirsDetectionLifecycleEvent {
  std::uint32_t cycle_index{0U};
  std::uint64_t target_id{0U};
  std::string target_name{};
  SbirsDetectionLifecycleEventKind kind{SbirsDetectionLifecycleEventKind::kUpdated};
  SbirsDetectionLifecycleReason reason{SbirsDetectionLifecycleReason::kNone};
  output::SbirsObservationStage observation_stage{output::SbirsObservationStage::kWideFieldSearch};
  float infrared_snr_linear{0.0f};
  float estimated_range_m{0.0f};
  bool used_truth_assist{false};
};

struct ONEQ_API SbirsDetectionLifecycleRecorderConfig {
  bool emit_not_detected_events{false};
};

class ONEQ_API SbirsDetectionLifecycleRecorder {
 public:
  explicit SbirsDetectionLifecycleRecorder(
      SbirsDetectionLifecycleRecorderConfig config = SbirsDetectionLifecycleRecorderConfig{});
  ~SbirsDetectionLifecycleRecorder();

  SbirsDetectionLifecycleRecorder(const SbirsDetectionLifecycleRecorder&) = delete;
  SbirsDetectionLifecycleRecorder& operator=(const SbirsDetectionLifecycleRecorder&) = delete;
  SbirsDetectionLifecycleRecorder(SbirsDetectionLifecycleRecorder&&) noexcept;
  SbirsDetectionLifecycleRecorder& operator=(SbirsDetectionLifecycleRecorder&&) noexcept;

  std::vector<SbirsDetectionLifecycleEvent> Update(const SbirsCycleInput& input,
                                                   const SbirsCycleResult& result);
  void Reset();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_DETECTION_LIFECYCLE_RECORDER_H_
