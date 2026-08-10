/**
 * @file logger_i18n.h
 * @brief 集成端日志中文适配：issue code → 中文名 + 问题列表格式化（查表）。
 *
 * 库内 issue message 为英文人读文本且格式不承诺稳定（规则 13b：机器消费只认
 * code，不得解析 message）——因此示例层不做 message 翻译/解析，只把**稳定
 * code** 映射为中文名；量值一律从 DebugView 结构化字段取（组件组摘要行时
 * 填充）。code 直接引用库注册表常量（各模块 <Module>IssueCodes.h，单一事实
 * 来源），杜绝手抄字符串漂移；未知 code 返回英文 message 原文，库内新增
 * code 自动回退英文 message。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_I18N_H_

#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "1q/electro_optical_sensor/session/EosIssueCodes.h"
#include "1q/sar/session/SarIssueCodes.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"

namespace component_attachment {
namespace demo {

/// issue code → 中文名（人读日志）。未知 code 返回 nullptr（调用方回退英文原文）。
inline const char* IssueCodeChineseName(const std::string& code) {
  struct Entry {
    const char* code;
    const char* name;
  };
  // 只收录实测/已知稳定 code（引用库注册表常量）；库内新增 code 自动回退英文 message。
  static const Entry kNames[] = {
      {electro_optical_sensor::session::codes::kTargetOutOfFov, "视场外"},
      {electro_optical_sensor::session::codes::kInconsistentTargetEnergyBalance,
       "目标能量平衡校验"},
      {sbirs_sensor::session::codes::kTargetOutOfWfov, "宽视场外"},
      {sar::session::codes::kSquintAngleExceedsLimit, "斜视角超限"},
      {sar::session::codes::kPulseRingBuffer, "脉冲环缓冲"},
      {sar::session::codes::kSlantRangeMismatch, "斜距不匹配"},
      {airborne_radar::session::codes::kTargetSnrBelowThreshold, "目标信噪比低于门限"},
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
