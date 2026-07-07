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

/**
 * @brief EosDebugTargetStatus 表示单目标在调试视图中的状态归类。
 */
enum class EosDebugTargetStatus {
  kDetected = 0,                /**< 已通过探测门限判决 */
  kObservedBelowThreshold = 1,  /**< 存在原始记录但信噪比 (SNR) 低于门限 */
  kNotInOutput = 2,             /**< 输入中存在但本周期输出中无记录 */
  kCycleNotExecuted = 3         /**< 本周期核心 pipeline 未实际执行 */
};

/**
 * @brief EosDebugTargetState 描述单个目标在调试视图中的可读状态。
 */
struct ONEQ_API EosDebugTargetState {
  std::uint64_t target_id{0U};                                           /**< 目标标识 */
  std::string target_name{};                                             /**< 目标名称，仅用于人读 */
  EosDebugTargetStatus status{EosDebugTargetStatus::kNotInOutput};       /**< 目标状态归类 */
  bool present_in_input{false};                                          /**< 是否出现在本周期输入场景中 */
  bool has_raw_output_record{false};                                     /**< 是否在原始输出帧中存在记录 */
  bool detected{false};                                                  /**< 是否通过探测门限判决 */
  float range_m{0.0f};                                                   /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};                                               /**< 方位角（单位：deg） */
  float elevation_deg{0.0f};                                             /**< 仰角（单位：deg） */
  float fused_snr_db{0.0f};                                              /**< 融合信噪比（单位：dB） */
};

/**
 * @brief EosOutputDebugView 描述单周期开发调试视图聚合结果。
 */
struct ONEQ_API EosOutputDebugView {
  std::uint32_t input_cycle_index{0U};            /**< 本次调用输入周期号 */
  std::uint32_t output_cycle_index{0U};           /**< 输出帧携带的周期号 */
  bool executed_this_cycle{false};                /**< 当前周期是否实际执行了核心 pipeline */
  bool reused_previous_output{false};             /**< 当前周期是否复用了上一有效输出 */
  bool has_validation_error{false};               /**< 是否存在 error 级输入问题 */
  session::EosPipelineAbortReason abort_reason{session::EosPipelineAbortReason::kNone}; /**< 当前周期终止原因 */
  std::vector<EosDebugTargetState> targets{};     /**< 各输入目标的调试状态列表 */
};

/**
 * @brief 把原始探测输出、执行结果与输入目标表合成为开发可读目标状态。
 *
 * 该构建器只读组合，不反向影响探测 pipeline。实现见 .cpp。
 */
class ONEQ_API EosOutputDebugViewBuilder {
 public:
  /**
   * @brief 由周期输入与结果构建调试视图。
   * @param[in] input 当前周期输入，提供场景目标列表。
   * @param[in] result 当前周期聚合结果。
   * @return 合成的调试视图，按输入场景目标顺序排列各目标状态。
   */
  static EosOutputDebugView Build(const EosCycleInput& input, const EosCycleResult& result);
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_DEBUG_VIEW_H_
