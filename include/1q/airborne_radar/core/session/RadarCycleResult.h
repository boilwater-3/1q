/**
 * @file RadarCycleResult.h
 * @brief 定义 RadarSession 单周期聚合结果类型。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_

#include <vector>

#include "1q/airborne_radar/common/control/RadarCommand.h"
#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"

namespace airborne_radar {
namespace core {
namespace session {

/**
 * @brief RadarCycleResult 描述单周期执行后的聚合观测结果。
 */
struct RadarCycleResult {
  common::output::TrackOutputFrame track_output_frame{};          /**< 当前周期轨迹输出帧 */
  std::vector<common::control::RadarCommand> submitted_commands{}; /**< 当前周期已提交的控制指令 */
  context::ValidationIssueList validation_issues{};       /**< 当前周期输入校验结果 */
  bool has_validation_error{false};                       /**< 是否存在 error 级输入问题 */
  bool has_control_profile{false};                        /**< 是否已持有最近一次控制真值 */
  common::control::RadarControlProfile control_profile{};          /**< 最近一次控制真值 */
  signal::pipeline::AssociationQualityMetrics
      association_quality_metrics{}; /**< 最近一次关联质量观测指标 */
};

}  // namespace session
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
