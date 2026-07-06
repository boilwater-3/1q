/**
 * @file SbirsPipeline.h
 * @brief SBIRS-inspired WFOV/NFOV pipeline。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_

#include <cstdint>
#include <map>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "sbirs_sensor/config/SbirsInternalExecutionConfig.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"

namespace sbirs_sensor {
namespace pipeline {

enum class SbirsTargetState {
  kUndetected = 0,
  kWideCandidate,
  kAwaitingNfovAcquisition,
  kTruthAssistedTracking,
  kLost
};

struct SbirsPipelineSnapshot {
  float scan_azimuth_deg{0.0f};
  std::uint64_t next_detection_id{1U};
  std::map<std::uint64_t, SbirsTargetState> target_states{};
  bool has_locked_target{false};
  std::uint64_t locked_target_id{0U};
  unsigned int random_state{1U};  // 误差模型随机源状态（replay 可复现）
};

struct SbirsPipelineDetection {
  output::SbirsDetectionRecord record{};
  attribution::SbirsDetectionAttributionRecord attribution{};
};

struct SbirsPipelineResult {
  float scan_azimuth_deg{0.0f};
  std::vector<SbirsPipelineDetection> detections{};
};

class SbirsPipeline {
 public:
  explicit SbirsPipeline(const config::SbirsInternalExecutionConfig& config);

  void ApplyConfig(const config::SbirsInternalExecutionConfig& config);
  SbirsPipelineResult RunCycle(const session::SbirsCycleInput& input);

  SbirsPipelineSnapshot CaptureRuntimeState() const;
  bool RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot);

 private:
  config::SbirsInternalExecutionConfig config_{};
  float scan_azimuth_deg_{0.0f};
  std::uint64_t next_detection_id_{1U};
  std::map<std::uint64_t, SbirsTargetState> target_states_{};
  bool has_locked_target_{false};
  std::uint64_t locked_target_id_{0U};
  foundation::SbirsRandomSource random_source_;  // 误差模型确定性随机源
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_
