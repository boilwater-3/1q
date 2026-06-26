/**
 * @file EsrCycleResult.h
 * @brief 定义电子侦察单周期输出帧与聚合结果类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_RESULT_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_RESULT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrOutputFrame 表示电子侦察模块的三通道聚合输出。
 * @warning `emitter_output` 不应包含任何真值直通字段。
 */
struct ONEQ_API EsrOutputFrame {
  std::uint32_t cycle_index{0U};                             /**< 当前周期号 */
  std::uint64_t batch_id{0U};                                /**< 当前批次号 */
  session::ObservationOutputFrame observation_output{};    /**< 传感器观测输出通道 */
  session::EmitterOutputFrame emitter_output{};            /**< 侦察假设输出通道 */
  session::TruthEvaluationFrame truth_evaluation_output{}; /**< 真值评估输出通道 */
};

/**
 * @brief EsrCycleResult 描述电子侦察会话单周期聚合结果。
 */
struct ONEQ_API EsrCycleResult {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  EsrOutputFrame output_frame{};       /**< 当前周期输出帧 */
  session::ValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  bool has_validation_error{false};                 /**< 是否存在 error 级输入问题 */
  bool executed_this_cycle{false};                  /**< 当前调用是否真正执行了 pipeline */
  bool reused_previous_output{false}; /**< 当前 output_frame 是否复用了上一有效周期输出 */
  session::EsrPipelineAbortReason abort_reason{
      session::EsrPipelineAbortReason::kNone}; /**< 若 downstream 链路 abort，给出结构化原因 */
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_RESULT_H_
