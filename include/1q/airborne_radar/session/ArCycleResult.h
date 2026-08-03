/**
 * @file ArCycleResult.h
 * @brief 机载雷达单周期执行结果类型。
 *
 * 单周期结果及轨迹输出查询辅助函数的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_RESULT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_RESULT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArTrackOutput.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/** @brief AR 单周期执行状态；该枚举是结果有效性的唯一真相。 */
enum class ArCycleStatus : std::uint8_t {
  kCompleted = 0,
  kPoweredOff,
  kRejectedInvalidInput,
  kRejectedInvalidConfig,
  kRejectedExecution,
};

/** @brief 接收机结构化损伤状态。 */
enum class ArReceiverImpairment : std::uint8_t { kNone = 0, kSaturated };

/**
 * @brief ArCycleResult 描述单周期执行后的聚合观测结果。
 * @note 只有 `status == kCompleted` 时携带接收、探测和跟踪输出；拒绝周期不复用上一帧。
 *       若实际发射已经在接收侧失败前提交，`kRejectedExecution` 仍通过
 *       `emission_frame` 返回该不可撤销的发射事实。
 */
struct ONEQ_API ArCycleResult {
  std::uint32_t input_cycle_index{0U};   /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  ArCycleStatus status{ArCycleStatus::kRejectedInvalidInput}; /**< 周期执行状态。 */
  TrackOutputFrame track_output_frame{}; /**< 当前调用返回的轨迹输出帧 */
  oneq::electromagnetics::RfEmissionFrame emission_frame{}; /**< 本周期实际 AR 发射。 */
  ArReceiverImpairment receiver_impairment{ArReceiverImpairment::kNone};
  ArInterferenceObservationList interference_observations{};
  std::vector<session::ArCommand>
      submitted_commands{};                /**< 当前周期已提交的控制指令；若未执行则为空 */
  ValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  ArDiagnosticIssueList diagnostics{};     /**< 细粒度诊断（三写：结构化信号 + 诊断 + 日志） */
  bool has_validation_error{false};        /**< 是否存在 error 级输入问题 */
  session::SignalCycleAbortReason abort_reason{
      session::SignalCycleAbortReason::kNone}; /**< 若下游主链路 abort，给出结构化原因 */
  bool has_control_profile{false};    /**< 当前周期是否产出了可归属到本周期的控制真值 */
  session::ArControlProfile control_profile{}; /**< 当前周期控制真值；若未执行则保持默认值 */
  session::AssociationQualityMetrics
      association_quality_metrics{}; /**< 当前周期关联质量观测指标；若未执行则保持默认值 */
  bool has_decision_observation{false}; /**< 当前周期是否发布了新的外部决策观测 */
  session::DecisionObservation decision_observation{}; /**< 当前周期决策观测 */
  session::DecisionControlSource applied_decision_source{
      session::DecisionControlSource::kNone}; /**< 本周期控制采用的决策来源 */
  std::uint32_t applied_decision_cycle_index{0U}; /**< 本周期控制对应的源周期 */
  std::uint64_t applied_decision_batch_id{0U};    /**< 本周期控制对应的源批号 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_RESULT_H_
