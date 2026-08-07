/**
 * @file debug_view_json.h
 * @brief 传感器模块 DebugView → JSON 序列化共享原语（header-only，无第三方依赖）。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12：本库不提供跨周期
 * 状态查询接口，"到目前为止"的累积信息由调用方将每周期 DebugView 以结构化格式
 * （如 JSON）写入自己的日志/事件系统获得。
 *
 * 本文件承载 4 个模块序列化器（ArDebugViewToJson.h / EosDebugViewToJson.h /
 * SarDebugViewToJson.h / SbirsDebugViewToJson.h）逐字相同的底层原语：JSON 字符串
 * 转义、诊断严重性枚举映射、diagnostics 数组序列化。各模块特有的枚举映射与字段
 * 布局保留在模块序列化器内。
 *
 * 集成方 copy 某个模块序列化器时，连同本文件一起 copy（或合并为一个文件）：
 * 每周期调用对应 *DebugViewToJson() 得到一条 JSON 记录，写入你们自己的日志即可；
 * 字段名与格式可按需调整。
 *
 * 属于 examples 层，不是 oneq 库的 public surface；库内部不消费 JSON。
 */

#ifndef EXAMPLES_COMMON_DEBUG_VIEW_JSON_H_
#define EXAMPLES_COMMON_DEBUG_VIEW_JSON_H_

#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

namespace {
// 匿名 namespace：与既有模块序列化器一致，各 TU 内部链接，避免 ODR 冲突。

/**
 * @brief 转义 JSON 字符串内容（引号/反斜杠/控制字符）。
 * @param[in] text 原始字符串。
 * @return 转义后的字符串（不含外层引号）。
 */
std::string JsonEscape(const std::string& text) {
  std::ostringstream out;
  for (char ch : text) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20U) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<unsigned int>(static_cast<unsigned char>(ch)) << std::dec;
        } else {
          out << ch;
        }
    }
  }
  return out.str();
}

/**
 * @brief 诊断严重性枚举 → JSON 字符串。
 * @tparam Severity 各模块诊断严重性枚举（均为 kInfo/kWarning/kError）。
 * @param[in] severity 严重性值。
 * @return "info"/"warning"/"error"，未知值返回 "unknown"。
 */
template <typename Severity>
const char* JsonSeverityName(Severity severity) {
  switch (severity) {
    case Severity::kInfo:
      return "info";
    case Severity::kWarning:
      return "warning";
    case Severity::kError:
      return "error";
  }
  return "unknown";
}

/**
 * @brief 把问题列表序列化为 JSON 数组并闭合根对象。
 *
 * 各模块 DebugView 的 issues 均为最后一个字段，其前一个字段为数组
 * （tracks/targets/point_targets），根对象在 issues 之后闭合
 * （"……],"issues":[...]}"）。调用方先写完其余字段，再调用本函数。
 *
 * 统一问题列表模型（session_contract.md 规则 14）：条目含 severity/code/message；
 * 已迁移模块的条目还含 phase（来源标签）与 field（可选定位），通过成员探测按需输出。
 *
 * @tparam IssueList 各模块问题列表（std::vector<XxxIssue>）。
 * @param[in,out] out 序列化输出流（已有其余字段）。
 * @param[in] issues 问题条目列表。
 */
template <typename T, typename = void>
struct HasIssuePhase : std::false_type {};
template <typename T>
struct HasIssuePhase<T, std::void_t<decltype(std::declval<T>().phase)>> : std::true_type {};

template <typename T, typename = void>
struct HasIssueField : std::false_type {};
template <typename T>
struct HasIssueField<T, std::void_t<decltype(std::declval<T>().field)>> : std::true_type {};

template <typename IssueList>
void WriteIssuesJson(std::ostringstream& out, const IssueList& issues) {
  out << "],\"issues\":[";
  for (std::size_t i = 0U; i < issues.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    const auto& issue = issues[i];
    out << "{\"severity\":\"" << JsonSeverityName(issue.severity) << '"';
    if constexpr (HasIssuePhase<typename IssueList::value_type>::value) {
      out << ",\"phase\":" << static_cast<int>(issue.phase);
    }
    out << ",\"code\":\"" << JsonEscape(issue.code) << '"' << ",\"message\":\""
        << JsonEscape(issue.message) << '"';
    if constexpr (HasIssueField<typename IssueList::value_type>::value) {
      if (!issue.field.empty()) {
        out << ",\"field\":\"" << JsonEscape(issue.field) << '"';
      }
    }
    out << "}";
  }
  out << "]}";
}

}  // namespace

#endif  // EXAMPLES_COMMON_DEBUG_VIEW_JSON_H_
