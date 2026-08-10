/**
 * @file logger_i18n.h
 * @brief 集成端日志中文适配：issue code → 中文名 + 问题列表格式化（纯查表，零依赖）。
 *
 * 库内 issue message 为英文人读文本且格式不承诺稳定（规则 13b：机器消费只认
 * code，不得解析 message）——因此示例层不做 message 翻译/解析，只把**稳定
 * code** 映射为中文名；量值一律从 DebugView 结构化字段取（组件组摘要行时
 * 填充）。未知 code 返回英文 message 原文，库内 code 集合演进时适配表无需
 * 同步维护。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_

#include <string>
#include <vector>

namespace component_attachment {
namespace demo {

/// issue code → 中文名（人读日志）。未知 code 返回 nullptr（调用方回退英文原文）。
inline const char* IssueCodeChineseName(const std::string& code) {
  struct Entry {
    const char* code;
    const char* name;
  };
  // 只收录实测/已知稳定 code；库内新增 code 自动回退英文 message。
  static const Entry kNames[] = {
      {"eos.target_out_of_fov", "视场外"},
      {"eos.validation.inconsistent_target_energy_balance", "目标能量平衡校验"},
      {"sbirs.target_out_of_wfov", "宽视场外"},
      {"sar.squint_angle_exceeds_limit", "斜视角超限"},
      {"sar.pulse_ring_buffer", "脉冲环缓冲"},
      {"sar.slant_range_mismatch", "斜距不匹配"},
      {"ar.target_snr_below_threshold", "目标信噪比低于门限"},
  };
  for (const auto& entry : kNames) {
    if (code == entry.code) {
      return entry.name;
    }
  }
  return nullptr;
}

/// 问题列表 → 人读文本（逗号分隔）：已知 code 输出 "code 中文名"；未知 code
/// 回退英文 message 原文（规则 13b：不翻译/不解析 message，量值走结构化字段）。
/// @tparam TIssue 各模块 *Issue 结构（含 code/message 字段）。
template <typename TIssue>
inline std::string FormatIssueText(const std::vector<TIssue>& issues) {
  std::string text;
  for (const auto& issue : issues) {
    if (!text.empty()) {
      text += ", ";
    }
    const char* zh = IssueCodeChineseName(issue.code);
    if (zh != nullptr) {
      text += issue.code + " " + zh;
    } else if (!issue.message.empty()) {
      text += issue.code + ": " + issue.message;
    } else {
      text += issue.code;
    }
  }
  return text;
}

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_
