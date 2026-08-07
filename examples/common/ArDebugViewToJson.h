/**
 * @file ArDebugViewToJson.h
 * @brief AR DebugView → JSON 字符串序列化（header-only，无第三方依赖）。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12：本库不提供跨周期
 * 状态查询接口，"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式
 * （如 JSON）写入自己的日志/事件系统获得。
 *
 * 集成方 copy 本文件 + debug_view_json.h（共享原语，可合并为一个文件）：
 * 每周期调用 ArDebugViewToJson() 得到一条 JSON 记录，写入你们自己的日志即可；
 * 字段名与格式可按需调整。
 *
 * DebugView 只是内存里的帧快照，落盘多少、落盘什么由调用方决定。除整帧序列化外，
 * 本文件另提供三种常见落盘模式参考实现（写循环在调用方，落盘代价与筛选方式
 * 成正比——例如 3000 条轨迹中 2997 条为 kConfirmed 时，模式一只落 3 行）：
 *   1) ArWriteNonNominalTracks()  —— 只落非标称行（跳过 kConfirmed）；
 *   2) ArWriteTrackStatusDeltas() —— 只落跨周期状态变化的目标行，上一周期状态表
 *      由调用方持有（external_target_id → status，库不提供跨周期接口，规则 12）；
 *   3) ArWriteDownsampledView()   —— 每 N 周期落一次全量帧，其余周期只落问题列表。
 */

#ifndef EXAMPLES_COMMON_AR_DEBUG_VIEW_TO_JSON_H_
#define EXAMPLES_COMMON_AR_DEBUG_VIEW_TO_JSON_H_

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "debug_view_json.h"

namespace {

const char* ArImpairmentName(airborne_radar::session::ArReceiverImpairment impairment) {
  switch (impairment) {
    case airborne_radar::session::ArReceiverImpairment::kNone:
      return "none";
    case airborne_radar::session::ArReceiverImpairment::kSaturated:
      return "saturated";
  }
  return "unknown";
}

const char* ArTrackStatusName(airborne_radar::session::ArDebugTrackStatus status) {
  switch (status) {
    case airborne_radar::session::ArDebugTrackStatus::kConfirmed:
      return "confirmed";
    case airborne_radar::session::ArDebugTrackStatus::kTentative:
      return "tentative";
    case airborne_radar::session::ArDebugTrackStatus::kLost:
      return "lost";
    case airborne_radar::session::ArDebugTrackStatus::kNotInOutput:
      return "not_in_output";
    case airborne_radar::session::ArDebugTrackStatus::kCycleNotCompleted:
      return "cycle_not_completed";
  }
  return "unknown";
}

/**
 * @brief 把单个目标调试轨迹状态写为一条 JSON 对象（不含换行）。
 * @param[in,out] out 输出流。
 * @param[in] track 目标调试轨迹状态。
 */
void WriteArTrackJson(std::ostream& out,
                      const airborne_radar::session::ArDebugTrackState& track) {
  out << "{\"external_target_id\":" << track.external_target_id << ",\"target_name\":\""
      << JsonEscape(track.target_name) << '"' << ",\"status\":\""
      << ArTrackStatusName(track.status) << '"'
      << ",\"present_in_input\":" << (track.present_in_input ? "true" : "false")
      << ",\"has_track\":" << (track.has_track ? "true" : "false")
      << ",\"association_key\":" << track.association_key
      << ",\"position_x\":" << track.position_x << ",\"position_y\":" << track.position_y
      << ",\"position_z\":" << track.position_z << ",\"speed\":" << track.speed
      << ",\"rcs\":" << track.rcs << ",\"hit_count\":" << track.hit_count
      << ",\"miss_count\":" << track.miss_count << ",\"target_type\":\""
      << JsonEscape(track.target_type) << "\"}";
}

}  // namespace

/**
 * @brief 把单周期 AR 调试视图序列化为一条 JSON 记录（帧快照）。
 * @param[in] view 由 ArTrackOutputDebugViewBuilder::Build() 产出的调试视图。
 * @return JSON 字符串，可直接写入调用方自己的日志/事件系统。
 */
inline std::string ArDebugViewToJson(const airborne_radar::session::ArTrackOutputDebugView& view) {
  std::ostringstream out;
  out << "{\"world_cycle_index\":" << view.world_cycle_index
      << ",\"output_cycle_index\":" << view.output_cycle_index
      << ",\"completed_this_cycle\":" << (view.completed_this_cycle ? "true" : "false")
      << ",\"receiver_impairment\":\"" << ArImpairmentName(view.receiver_impairment) << '"'
      << ",\"tracks\":[";
  for (std::size_t i = 0U; i < view.tracks.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    const airborne_radar::session::ArDebugTrackState& track = view.tracks[i];
    WriteArTrackJson(out, track);
  }
  WriteIssuesJson(out, view.issues);
  return out.str();
}

/**
 * @brief 把单个目标调试轨迹状态序列化为一条 JSON 对象字符串（不含换行，供落盘循环内复用）。
 * @param[in] track 目标调试轨迹状态。
 * @return JSON 字符串（不含换行）。
 */
inline std::string ArDebugTrackStateToJson(
    const airborne_radar::session::ArDebugTrackState& track) {
  std::ostringstream row;
  WriteArTrackJson(row, track);
  return row.str();
}

/**
 * @brief 判定轨迹状态是否属于"标称/成功"（落盘时可跳过）。
 * @param[in] status 轨迹调试状态。
 * @return true 表示标称状态（kConfirmed）；其余状态（候选/丢失/不在输出/未完成）为 false。
 * @note 判定集合可按需调整，例如把部分状态也并入标称。
 */
inline bool ArIsNominalTrackStatus(airborne_radar::session::ArDebugTrackStatus status) {
  return status == airborne_radar::session::ArDebugTrackStatus::kConfirmed;
}

/**
 * @brief 落盘模式一：只写非标称行，每行一条 JSON。
 *
 * 典型场景：3000 条轨迹中 2997 条为 kConfirmed，本函数只落候选/丢失/不在输出/
 * 未完成的行（约 3 行），日志流量与"排除了什么"成正比。
 *
 * @param[in] view 本周期调试视图。
 * @param[in,out] out 输出流（调用方自己的日志/事件系统）；行分隔符可按需调整。
 * @note 未执行周期（completed_this_cycle=false）会使全部轨迹状态翻转为
 *       kCycleNotCompleted，所有行都会落入非标称（整帧写出）；如不想记录可在
 *       调用侧先判断 view.completed_this_cycle。
 */
inline void ArWriteNonNominalTracks(const airborne_radar::session::ArTrackOutputDebugView& view,
                                    std::ostream& out) {
  for (const auto& track : view.tracks) {
    if (ArIsNominalTrackStatus(track.status)) {
      continue;  // 跳过已确认（标称行）
    }
    WriteArTrackJson(out, track);
    out << '\n';
  }
}

/**
 * @brief 落盘模式二：只写跨周期状态变化的目标行（增量）。
 *
 * 库每周期只给当帧快照（规则 12 无跨周期接口），上一周期状态表由调用方持有：
 * external_target_id → status。首次出现的目标视为状态变化（写入一次），后续状态
 * 未变的目标不写。每行携带 external_target_id，天然可作为跨周期比较的 key。
 *
 * @param[in] view 本周期调试视图。
 * @param[in,out] prev_status 调用方持有的上一周期状态表（首次调用传空表）。
 * @param[in,out] out 输出流（调用方自己的日志/事件系统）。
 * @note prev_status 只增不减：目标集长期收缩时调用方可按需清理；周期未完成
 *       （completed_this_cycle=false）会使全部轨迹状态整体翻转一次
 *       （kCycleNotCompleted），如不想记录可在调用侧先判断 view.completed_this_cycle。
 */
inline void ArWriteTrackStatusDeltas(
    const airborne_radar::session::ArTrackOutputDebugView& view,
    std::unordered_map<std::uint64_t, airborne_radar::session::ArDebugTrackStatus>& prev_status,
    std::ostream& out) {
  for (const auto& track : view.tracks) {
    const auto it = prev_status.find(track.external_target_id);
    const bool status_changed = (it == prev_status.end()) || (it->second != track.status);
    if (status_changed) {
      WriteArTrackJson(out, track);
      out << '\n';
    }
    prev_status[track.external_target_id] = track.status;
  }
}

/**
 * @brief 只写周期号 + 问题列表的 JSON 记录（供降频落盘使用）。
 * @param[in] view 本周期调试视图。
 * @param[in,out] out 输出流（调用方自己的日志/事件系统）。
 */
inline void ArWriteIssuesOnlyRecord(const airborne_radar::session::ArTrackOutputDebugView& view,
                                    std::ostream& out) {
  std::ostringstream record;
  record << "{\"world_cycle_index\":" << view.world_cycle_index << ',';
  WriteIssuesArrayJson(record, view.issues);
  record << '}';
  out << record.str() << '\n';
}

/**
 * @brief 落盘模式三：降频落盘——每 full_period 周期写一次全量帧，其余周期只写问题列表。
 *
 * issues 是独立小字段（通常只有几条），全量帧则含全部目标行；降频后日志流量
 * 约为原来的 1/full_period（周期数足够大时接近 1/10）。
 *
 * @param[in] view 本周期调试视图。
 * @param[in,out] out 输出流（调用方自己的日志/事件系统）。
 * @param[in] full_period 全量帧间隔周期数（默认 10）；周期计数取自
 *            view.world_cycle_index，调用方也可改用自己维护的周期计数器。
 *            传 0 时退化为每周期全量（避免除零）；周期从 1 计数时首个全量帧
 *            落在第 full_period 周期，如需首周期基线可自行调整判定。
 */
inline void ArWriteDownsampledView(const airborne_radar::session::ArTrackOutputDebugView& view,
                                   std::ostream& out, std::uint32_t full_period = 10U) {
  if (full_period == 0U) {
    out << ArDebugViewToJson(view) << '\n';  // full_period 为 0 时退化为每周期全量
    return;
  }
  if (view.world_cycle_index % full_period == 0U) {
    out << ArDebugViewToJson(view) << '\n';  // 全量帧（全部目标行 + 问题列表）
  } else {
    ArWriteIssuesOnlyRecord(view, out);  // 其余周期只落问题列表
  }
}

#endif  // EXAMPLES_COMMON_AR_DEBUG_VIEW_TO_JSON_H_
