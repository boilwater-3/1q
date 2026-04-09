/**
 * @file RadarCycleResult.h
 * @brief 定义 RadarSession 单周期聚合结果类型。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_

#include <vector>

#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarCycleResult 描述单周期执行后的聚合观测结果。
 */
struct RadarCycleResult {
  output::TrackOutputFrame track_output_frame{}; /**< 当前调用返回的轨迹输出帧；非法周期时可复用上一有效帧 */
  std::vector<extension::control::RadarCommand>
      submitted_commands{}; /**< 当前周期已提交的控制指令；若未执行则为空 */
  ValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  bool has_validation_error{false}; /**< 是否存在 error 级输入问题 */
  bool executed_this_cycle{false}; /**< 当前调用是否真正执行了 signal/decision/control 链路 */
  extension::SignalCycleAbortReason signal_cycle_abort_reason{
      extension::SignalCycleAbortReason::kNone}; /**< 若下游主链路 abort，给出结构化原因 */
  bool reused_previous_track_output{
      false}; /**< 当前 `track_output_frame` 是否复用了上一有效周期输出 */
  bool has_control_profile{false}; /**< 当前周期是否产出了可归属到本周期的控制真值 */
  extension::control::RadarControlProfile control_profile{}; /**< 当前周期控制真值；若未执行则保持默认值 */
  extension::AssociationQualityMetrics
      association_quality_metrics{}; /**< 当前周期关联质量观测指标；若未执行则保持默认值 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
