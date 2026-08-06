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
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrOutputFrame 表示电子侦察模块的去真值化输出。
 */
struct ONEQ_API EsrOutputFrame {
  std::uint32_t cycle_index{0U};                           /**< 当前周期号 */
  std::uint64_t batch_id{0U};                              /**< 当前批次号 */
  float scan_azimuth_deg{0.0f};                            /**< 本周期波束中心方位角（单位：deg，平台系，含天线安装角） */
  session::ObservationOutputFrame observation_output{};    /**< 传感器观测输出通道 */
  session::EmitterOutputFrame emitter_output{};            /**< 侦察假设输出通道 */
};

/** @brief 单周期 ESR 执行状态；只有 completed 携带本周期输出。 */
enum class ONEQ_API EsrCycleExecutionStatus {
  kCompleted = 0,
  kRejected = 1,
  kPoweredOff = 2,
};

/**
 * @brief EsrCycleResult 描述电子侦察会话单周期聚合结果。
 * @note `output_frame` 只有在 `status=kCompleted` 时才代表本周期有效计算结果；
 *       拒绝和关机周期不复用历史输出。
 */
struct ONEQ_API EsrCycleResult {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  EsrOutputFrame output_frame{};       /**< 当前周期输出帧 */
  session::ValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  EsrDiagnosticIssueList diagnostics{};             /**< 细粒度诊断（三写：结构化信号 + 诊断 + 日志） */
  bool has_validation_error{false};                 /**< 是否存在 error 级输入问题 */
  EsrCycleExecutionStatus status{EsrCycleExecutionStatus::kRejected}; /**< 周期执行真相。 */
  session::EsrPipelineAbortReason abort_reason{
      session::EsrPipelineAbortReason::kNone}; /**< 若 downstream 链路 abort，给出结构化原因 */
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_RESULT_H_
