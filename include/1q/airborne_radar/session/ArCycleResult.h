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

#include "1q/airborne_radar/config/ArOrientationConfig.h"
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
 * @brief 指定航迹跟踪回退成因（STT 未生效时的结构化原因）。
 */
enum class ArDesignationRevertReason : std::uint8_t {
  kNone = 0,              /**< 无回退（指定跟踪正常生效）。 */
  kTrackNotConfirmed = 1, /**< 指定目标无航迹或航迹未确认（候选/不存在）。 */
  kTrackLost = 2,         /**< 指定目标航迹已丢失（kLost）。 */
  kAcquisitionTimeout = 3 /**< 限时指令捕获超时：窗口内始终未捕获 confirmed
                                航迹，指令作废（回到扫描，终态）。 */
};

/**
 * @brief ArCycleResult 描述单周期执行后的聚合观测结果。
 * @note 只有 `status == kCompleted` 时携带接收、探测和跟踪输出；拒绝周期不复用上一帧。
 *       若实际发射已经在接收侧失败前提交，`kRejectedExecution` 仍通过
 *       `emission_frame` 返回该不可撤销的发射事实。
 */
struct ONEQ_API ArCycleResult {
  std::uint32_t input_cycle_index{0U};   /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  ArCycleStatus status{ArCycleStatus::kRejectedInvalidInput}; /**< 周期执行状态。 */
  TrackOutputFrame output_frame{}; /**< 当前调用返回的轨迹输出帧 */
  oneq::electromagnetics::RfEmissionFrame emission_frame{}; /**< 本周期实际 AR 发射。 */
  ArReceiverImpairment receiver_impairment{ArReceiverImpairment::kNone};
  ArInterferenceObservationList interference_observations{};
  std::vector<session::ArCommand>
      submitted_commands{};                /**< 当前周期已提交的控制指令；若未执行则为空 */
  ArIssueList issues{}; /**< 统一问题列表（规则 14：校验问题 phase=kInputValidation 在前 +
                              执行诊断 phase=kExecution/kOutputContract 在后） */
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

  /**
   * @brief 本周期生效工作模式（可能因指定航迹回退/指令作废与已提交配置不同）。
   * @note 派生规则（冻结）：已提交 work_mode == kStt 且指定目标存在且其航迹
   *       confirmed 时为 kStt；指定航迹未确认/丢失时回退为 kTws；未指定目标的
   *       STT 保持现状 scan_center 驻留语义（仍为 kStt）；限时指令捕获超时
   *       （kExpired）后按扫描处理（已提交 kStt 时生效为 kTws，指令作废）。
   *       仅 completed 周期填充。
   */
  config::ArWorkMode effective_work_mode{config::ArWorkMode::kTws};
  /**
   * @brief 本周期是否存在对指定目标的生效航迹跟随驻留（STT 指向跟随指定航迹）。
   * @note 派生规则（冻结）：已提交 STT && 已指定 && 指定航迹 confirmed
   *       && 无显式 dwell 覆盖（显式 dwell 时指向按现状语义，不跟随航迹，
   *       本字段为 false 且不构成回退）。
   */
  bool designation_active{false};
  /** @brief 当前指定跟踪目标外部 ID（未指定时为 0）。 */
  std::uint64_t designated_target_id{0U};
  /**
   * @brief 本周期 STT 已被请求（work_mode 提交为 kStt 且已指定目标）但未生效
   *        （指定航迹未确认/丢失），生效模式回退到 TWS；限时指令捕获超时
   *        的作废周期亦为 true（成因 kAcquisitionTimeout，其后指定清零）。
   * @note 每周期派生状态指示（非转换边沿）；跨周期差分由调用方或生命周期
   *       记录器承担。
   */
  bool designation_reverted_to_tws{false};
  /** @brief 回退成因（仅 designation_reverted_to_tws == true 时有意义）。 */
  ArDesignationRevertReason designation_revert_reason{ArDesignationRevertReason::kNone};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_CYCLE_RESULT_H_
