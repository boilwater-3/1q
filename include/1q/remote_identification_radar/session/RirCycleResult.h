/**
 * @file RirCycleResult.h
 * @brief 远程识别雷达单周期执行结果类型。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_RESULT_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_RESULT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"

namespace remote_identification_radar {
namespace session {

/** @brief 远程识别雷达单周期执行状态；该枚举是结果有效性的唯一真相。 */
enum class RirCycleStatus : std::uint8_t {
  kCompleted = 0,        /**< 周期完成。 */
  kPoweredOff,           /**< 传感器关机（只推进世界时间）。 */
  kRejectedInvalidInput, /**< 输入校验拒绝。 */
  kRejectedInvalidConfig,/**< 配置校验拒绝。 */
  kRejectedExecution,    /**< 执行阶段失败。 */
};

/**
 * @brief RirCycleResult 描述单周期执行后的聚合结果。
 * @note 只有 `status == kCompleted` 携带识别输出；拒绝周期不复用上一帧。
 */
struct ONEQ_API RirCycleResult {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  RirCycleStatus status{RirCycleStatus::kRejectedInvalidInput}; /**< 周期执行状态。 */
  RirOutputFrame output_frame{}; /**< 当前调用返回的识别输出帧。 */
  RirIssueList issues{}; /**< 统一问题列表（规则 14：校验问题 phase=kInputValidation 在前 +
                               执行诊断 phase=kExecution/kOutputContract 在后） */
  RirCycleAbortReason abort_reason{
      RirCycleAbortReason::kNone}; /**< 若周期 abort，给出结构化原因 */
  bool has_recognition_summary{false}; /**< 本周期是否发布了识别效能摘要 */
  RirRecognitionCycleSummary recognition_summary{}; /**< 本周期识别效能摘要；未执行时保持默认值 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_RESULT_H_
