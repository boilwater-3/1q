/**
 * @file RadarCycleResult.h
 * @brief 定义 RadarSession 单周期输出帧、查询工具与聚合结果类型。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief TrackOutputFrame 表示单周期稳定的中性轨迹输出帧。
 */
struct ONEQ_API TrackOutputFrame {
  std::uint32_t cycle_index{0};           /**< 当前周期号 */
  std::uint64_t batch_id{0};              /**< 当前批号 */
  model::TrackStateSnapshotList tracks{}; /**< 当前周期发布的轨迹快照列表 */
};

/**
 * @brief 按外部目标 ID 构造轨迹映射。
 */
ONEQ_API std::unordered_map<std::uint64_t, model::TrackStateSnapshot>
BuildTrackMapByExternalTargetId(const TrackOutputFrame& frame);

/**
 * @brief 按关联键构造轨迹映射。
 */
ONEQ_API std::unordered_map<std::uint64_t, model::TrackStateSnapshot> BuildTrackMapByAssociationKey(
    const TrackOutputFrame& frame);

/**
 * @brief 收集指定外部目标 ID 对应的全部轨迹。
 */
ONEQ_API model::TrackStateSnapshotList CollectTracksByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id);

/**
 * @brief 收集所有已确认轨迹。
 */
ONEQ_API model::TrackStateSnapshotList CollectConfirmedTracks(const TrackOutputFrame& frame);

/**
 * @brief 收集所有 lost 轨迹。
 */
ONEQ_API model::TrackStateSnapshotList CollectLostTracks(const TrackOutputFrame& frame);

/**
 * @brief 收集所有带干扰标记的轨迹。
 */
ONEQ_API model::TrackStateSnapshotList CollectJammingTracks(const TrackOutputFrame& frame);

/**
 * @brief 判断输出帧中是否包含指定外部目标 ID。
 */
ONEQ_API bool ContainsExternalTargetId(const TrackOutputFrame& frame,
                                       std::uint64_t external_target_id);

/**
 * @brief 统计携带干扰标记的轨迹数量。
 */
ONEQ_API std::size_t CountJammingTracks(const TrackOutputFrame& frame);

/**
 * @brief 按生命周期状态统计轨迹数量。
 */
ONEQ_API std::size_t CountTracksByStatus(const TrackOutputFrame& frame, model::TrackStatus status);

/**
 * @brief RadarCycleResult 描述单周期执行后的聚合观测结果。
 */
struct ONEQ_API RadarCycleResult {
  std::uint32_t input_cycle_index{0U};   /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  TrackOutputFrame track_output_frame{}; /**< 当前调用返回的轨迹输出帧 */
  std::vector<extension::control::RadarCommand>
      submitted_commands{};                /**< 当前周期已提交的控制指令；若未执行则为空 */
  ValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  bool has_validation_error{false};        /**< 是否存在 error 级输入问题 */
  bool executed_this_cycle{false}; /**< 当前调用是否真正执行了 signal/decision/control 链路 */
  extension::SignalCycleAbortReason signal_cycle_abort_reason{
      extension::SignalCycleAbortReason::kNone}; /**< 若下游主链路 abort，给出结构化原因 */
  bool reused_previous_track_output{
      false};                      /**< 当前 `track_output_frame` 是否复用了上一有效周期输出 */
  bool has_control_profile{false}; /**< 当前周期是否产出了可归属到本周期的控制真值 */
  extension::control::RadarControlProfile
      control_profile{}; /**< 当前周期控制真值；若未执行则保持默认值 */
  extension::AssociationQualityMetrics
      association_quality_metrics{}; /**< 当前周期关联质量观测指标；若未执行则保持默认值 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
