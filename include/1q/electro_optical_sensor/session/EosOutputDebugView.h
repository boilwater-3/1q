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
#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace session {

// 前向声明：Build 参数为 const 引用，header 无需完整类型，避免拉入 EosCycleInput 重依赖。
struct EosCycleInput;

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
  session::EosPipelineAbortReason abort_reason{session::EosPipelineAbortReason::kNone};
  std::vector<EosDebugTargetState> targets{};
};

/**
 * @brief 把原始探测输出、执行结果与输入目标表合成为开发可读目标状态。
 *
 * 该构建器只读组合，不反向影响探测 pipeline。实现见 .cpp。
 */
class ONEQ_API EosOutputDebugViewBuilder {
 public:
  static EosOutputDebugView Build(const EosCycleInput& input, const EosCycleResult& result);
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_DEBUG_VIEW_H_
