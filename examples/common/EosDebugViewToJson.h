/**
 * @file EosDebugViewToJson.h
 * @brief EOS DebugView → JSON 字符串序列化（header-only，无第三方依赖）。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12：本库不提供跨周期
 * 状态查询接口，"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式
 * （如 JSON）写入自己的日志/事件系统获得。
 *
 * 集成方 copy 本文件 + debug_view_json.h（共享原语，可合并为一个文件）：
 * 每周期调用 EosDebugViewToJson() 得到一条 JSON 记录，写入你们自己的日志即可；
 * 字段名与格式可按需调整。
 *
 * DebugView 只是内存里的帧快照，落盘多少、落盘什么由调用方决定。除整帧序列化外，
 * 本文件另提供三种常见落盘模式参考实现（写循环在调用方，落盘代价与筛选方式
 * 成正比——例如 3000 个目标中 2997 个为 kDetected 时，模式一只落 3 行）：
 *   1) EosWriteNonNominalTargets()  —— 只落非标称行（跳过 kDetected）；
 *   2) EosWriteTargetStatusDeltas() —— 只落跨周期状态变化的目标行，上一周期状态表
 *      由调用方持有（target_id → status，库不提供跨周期接口，规则 12）；
 *   3) EosWriteDownsampledView()    —— 每 N 周期落一次全量帧，其余周期只落问题列表。
 */

#ifndef EXAMPLES_COMMON_EOS_DEBUG_VIEW_TO_JSON_H_
#define EXAMPLES_COMMON_EOS_DEBUG_VIEW_TO_JSON_H_

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "debug_view_json.h"

namespace {

const char* EosAbortReasonName(electro_optical_sensor::session::EosPipelineAbortReason reason) {
  switch (reason) {
    case electro_optical_sensor::session::EosPipelineAbortReason::kNone:
      return "none";
    case electro_optical_sensor::session::EosPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case electro_optical_sensor::session::EosPipelineAbortReason::kOutputContractViolation:
      return "output_contract_violation";
    case electro_optical_sensor::session::EosPipelineAbortReason::kRuntimeStateRestoreRejected:
      return "runtime_state_restore_rejected";
    case electro_optical_sensor::session::EosPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  return "unknown";
}

const char* EosTargetStatusName(electro_optical_sensor::session::EosDebugTargetStatus status) {
  switch (status) {
    case electro_optical_sensor::session::EosDebugTargetStatus::kDetected:
      return "detected";
    case electro_optical_sensor::session::EosDebugTargetStatus::kObservedBelowThreshold:
      return "observed_below_threshold";
    case electro_optical_sensor::session::EosDebugTargetStatus::kNotInOutput:
      return "not_in_output";
    case electro_optical_sensor::session::EosDebugTargetStatus::kCycleNotExecuted:
      return "cycle_not_executed";
  }
  return "unknown";
}

/**
 * @brief 把单个目标调试状态写为一条 JSON 对象（不含换行）。
 * @param[in,out] out 输出流。
 * @param[in] target 目标调试状态。
 */
void WriteEosTargetJson(std::ostream& out,
                        const electro_optical_sensor::session::EosDebugTargetState& target) {
  out << "{\"target_id\":" << target.target_id << ",\"target_name\":\""
      << JsonEscape(target.target_name) << '"' << ",\"status\":\""
      << EosTargetStatusName(target.status) << '"'
      << ",\"present_in_input\":" << (target.present_in_input ? "true" : "false")
      << ",\"has_raw_output_record\":" << (target.has_raw_output_record ? "true" : "false")
      << ",\"detected\":" << (target.detected ? "true" : "false")
      << ",\"range_m\":" << target.range_m << ",\"azimuth_deg\":" << target.azimuth_deg
      << ",\"elevation_deg\":" << target.elevation_deg
      << ",\"fused_snr_db\":" << target.fused_snr_db << '}';
}

}  // namespace

/**
 * @brief 把单周期 EOS 调试视图序列化为一条 JSON 记录（帧快照）。
 * @param[in] view 由 EosOutputDebugViewBuilder::Build() 产出的调试视图。
 * @return JSON 字符串，可直接写入调用方自己的日志/事件系统。
 */
inline std::string EosDebugViewToJson(
    const electro_optical_sensor::session::EosOutputDebugView& view) {
  std::ostringstream out;
  out << "{\"input_cycle_index\":" << view.input_cycle_index
      << ",\"output_cycle_index\":" << view.output_cycle_index
      << ",\"executed_this_cycle\":" << (view.executed_this_cycle ? "true" : "false")
      
      << ",\"abort_reason\":\"" << EosAbortReasonName(view.abort_reason) << '"' << ",\"targets\":[";
  for (std::size_t i = 0U; i < view.targets.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    const electro_optical_sensor::session::EosDebugTargetState& target = view.targets[i];
    WriteEosTargetJson(out, target);
  }
  WriteIssuesJson(out, view.issues);
  return out.str();
}

/**
 * @brief 把单个目标调试状态序列化为一条 JSON 对象字符串（不含换行，供落盘循环内复用）。
 * @param[in] target 目标调试状态。
 * @return JSON 字符串（不含换行）。
 */
inline std::string EosDebugTargetStateToJson(
    const electro_optical_sensor::session::EosDebugTargetState& target) {
  std::ostringstream row;
  WriteEosTargetJson(row, target);
  return row.str();
}

/**
 * @brief 判定目标状态是否属于"标称/成功"（落盘时可跳过）。
 * @param[in] status 目标调试状态。
 * @return true 表示标称状态（kDetected）；其余状态（低于门限/不在输出/未执行）为 false。
 * @note 判定集合可按需调整，例如把部分状态也并入标称。
 */
inline bool EosIsNominalTargetStatus(electro_optical_sensor::session::EosDebugTargetStatus status) {
  return status == electro_optical_sensor::session::EosDebugTargetStatus::kDetected;
}

/**
 * @brief 落盘模式一：只写非标称行，每行一条 JSON。
 *
 * 典型场景：3000 个目标中 2997 个为 kDetected，本函数只落被排除/低于门限/
 * 未执行的行（约 3 行），日志流量与"排除了什么"成正比。
 *
 * @param[in] view 本周期调试视图。
 * @param[in,out] out 输出流（调用方自己的日志/事件系统）；行分隔符可按需调整。
 * @note 未执行周期（executed_this_cycle=false）会使全部目标状态翻转为
 *       kCycleNotExecuted，所有行都会落入非标称（整帧写出）；如不想记录可在
 *       调用侧先判断 view.executed_this_cycle。
 */
inline void EosWriteNonNominalTargets(
    const electro_optical_sensor::session::EosOutputDebugView& view, std::ostream& out) {
  for (const auto& target : view.targets) {
    if (EosIsNominalTargetStatus(target.status)) {
      continue;  // 跳过探测成功（标称行）
    }
    WriteEosTargetJson(out, target);
    out << '\n';
  }
}

/**
 * @brief 落盘模式二：只写跨周期状态变化的目标行（增量）。
 *
 * 库每周期只给当帧快照（规则 12 无跨周期接口），上一周期状态表由调用方持有：
 * target_id → status。首次出现的目标视为状态变化（写入一次），后续状态未变的
 * 目标不写。每行携带 target_id，天然可作为跨周期比较的 key。
 *
 * @param[in] view 本周期调试视图。
 * @param[in,out] prev_status 调用方持有的上一周期状态表（首次调用传空表）。
 * @param[in,out] out 输出流（调用方自己的日志/事件系统）。
 * @note prev_status 只增不减：目标集长期收缩时调用方可按需清理；传感器关机等
 *       未执行周期会使全部目标状态整体翻转一次（kCycleNotExecuted），如不想记录
 *       可在调用侧先判断 view.executed_this_cycle。
 */
inline void EosWriteTargetStatusDeltas(
    const electro_optical_sensor::session::EosOutputDebugView& view,
    std::unordered_map<std::uint64_t,
                       electro_optical_sensor::session::EosDebugTargetStatus>& prev_status,
    std::ostream& out) {
  for (const auto& target : view.targets) {
    const auto it = prev_status.find(target.target_id);
    const bool status_changed = (it == prev_status.end()) || (it->second != target.status);
    if (status_changed) {
      WriteEosTargetJson(out, target);
      out << '\n';
    }
    prev_status[target.target_id] = target.status;
  }
}

/**
 * @brief 只写周期号 + 问题列表的 JSON 记录（供降频落盘使用）。
 * @param[in] view 本周期调试视图。
 * @param[in,out] out 输出流（调用方自己的日志/事件系统）。
 */
inline void EosWriteIssuesOnlyRecord(
    const electro_optical_sensor::session::EosOutputDebugView& view, std::ostream& out) {
  std::ostringstream record;
  record << "{\"input_cycle_index\":" << view.input_cycle_index << ',';
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
 *            view.input_cycle_index，调用方也可改用自己维护的周期计数器。
 *            传 0 时退化为每周期全量（避免除零）；周期从 1 计数时首个全量帧
 *            落在第 full_period 周期，如需首周期基线可自行调整判定。
 */
inline void EosWriteDownsampledView(const electro_optical_sensor::session::EosOutputDebugView& view,
                                    std::ostream& out, std::uint32_t full_period = 10U) {
  if (full_period == 0U) {
    out << EosDebugViewToJson(view) << '\n';  // full_period 为 0 时退化为每周期全量
    return;
  }
  if (view.input_cycle_index % full_period == 0U) {
    out << EosDebugViewToJson(view) << '\n';  // 全量帧（全部目标行 + 问题列表）
  } else {
    EosWriteIssuesOnlyRecord(view, out);  // 其余周期只落问题列表
  }
}

#endif  // EXAMPLES_COMMON_EOS_DEBUG_VIEW_TO_JSON_H_
