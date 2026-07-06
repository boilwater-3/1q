/**
 * @file SbirsOutputDebugView.h
 * @brief 定义 SBIRS-inspired 输出开发调试视图。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_DEBUG_VIEW_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_DEBUG_VIEW_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace sbirs_sensor {
namespace session {

struct SbirsCycleInput;

enum class ONEQ_API SbirsDebugTargetStatus {
  kDetected = 0,
  kObservedBelowThreshold,
  kNotInOutput,
  kCycleNotExecuted
};

struct ONEQ_API SbirsDebugTargetState {
  std::uint64_t target_id{0U};
  std::string target_name{};
  SbirsDebugTargetStatus status{SbirsDebugTargetStatus::kNotInOutput};
  bool present_in_input{false};
  bool has_raw_output_record{false};
  bool detected{false};
  bool used_truth_assist{false};
  float estimated_range_m{0.0f};
  float azimuth_deg{0.0f};
  float elevation_deg{0.0f};
  float infrared_snr_linear{0.0f};
  output::SbirsObservationStage observation_stage{output::SbirsObservationStage::kWideFieldSearch};
};

struct ONEQ_API SbirsOutputDebugView {
  std::uint32_t input_cycle_index{0U};
  std::uint32_t output_cycle_index{0U};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  bool has_validation_error{false};
  SbirsPipelineAbortReason abort_reason{SbirsPipelineAbortReason::kNone};
  std::vector<SbirsDebugTargetState> targets{};
};

class ONEQ_API SbirsOutputDebugViewBuilder {
 public:
  static SbirsOutputDebugView Build(const SbirsCycleInput& input, const SbirsCycleResult& result);
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_DEBUG_VIEW_H_
