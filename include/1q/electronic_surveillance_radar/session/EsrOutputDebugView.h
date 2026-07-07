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

/**
 * @brief EsrDebugEmitterStatus 表示单辐射源调试状态分类。
 */
enum class EsrDebugEmitterStatus {
  kObserved = 0,        /**< 本周期被观测到且命中真值关联 */
  kNotObserved = 1,     /**< 发射中但未命中观测/真值关联 */
  kNotEmitting = 2,     /**< 辐射源本周期未发射 */
  kCycleNotExecuted = 3 /**< 本周期未执行核心 pipeline */
};

/**
 * @brief EsrDebugEmitterState 描述单个输入辐射源的调试状态。
 */
struct ONEQ_API EsrDebugEmitterState {
  std::uint64_t emitter_id{0U};                              /**< 辐射源标识 */
  std::string emitter_name{};                                /**< 辐射源名称，仅用于人读 */
  EsrDebugEmitterStatus status{EsrDebugEmitterStatus::kNotObserved}; /**< 调试状态分类 */
  bool present_in_input{false};                              /**< 是否出现在本周期输入中 */
  bool matched_observation{false};                           /**< 是否命中真值关联观测 */
  std::uint64_t observation_id{0U};                          /**< 命中观测记录标识（未命中为 0） */
  float confidence{0.0f};                                    /**< 关联置信度，范围 [0, 1] */
};

/**
 * @brief EsrOutputDebugView 描述单周期可读调试视图。
 */
struct ONEQ_API EsrOutputDebugView {
  std::uint32_t input_cycle_index{0U};                                  /**< 输入周期号 */
  std::uint32_t output_cycle_index{0U};                                 /**< 输出帧周期号 */
  bool executed_this_cycle{false};                                      /**< 本周期是否执行了核心 pipeline */
  bool reused_previous_output{false};                                   /**< 是否复用了上一有效输出 */
  bool has_validation_error{false};                                     /**< 是否存在 error 级输入问题 */
  session::EsrPipelineAbortReason abort_reason{session::EsrPipelineAbortReason::kNone}; /**< 周期终止原因 */
  std::vector<EsrDebugEmitterState> emitters{};                         /**< 逐辐射源调试状态 */
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
