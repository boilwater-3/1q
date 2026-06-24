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
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"

namespace electronic_surveillance_radar {
namespace session {

// 前向声明：Build 参数为 const 引用，header 无需完整类型，避免拉入 EsrCycleInput 重依赖。
struct EsrCycleInput;

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

/**
 * @brief 把原始三通道输出、执行结果与输入辐射源表合成为开发可读辐射源状态。
 *
 * 该构建器只读组合，不反向影响拦截 pipeline。实现见 .cpp。
 */
class ONEQ_API EsrOutputDebugViewBuilder {
 public:
  static EsrOutputDebugView Build(const EsrCycleInput& input, const EsrCycleResult& result);
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_DEBUG_VIEW_H_
