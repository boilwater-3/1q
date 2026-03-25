/**
 * @file ValidationUtils.h
 * @brief 定义校验链路复用的有限值、问题构造与严重级别扫描工具。
 */

#ifndef COMMON_VALIDATION_VALIDATION_UTILS_H_
#define COMMON_VALIDATION_VALIDATION_UTILS_H_

#include <cmath>
#include <cstddef>
#include <string>

namespace oneq {
namespace internal {
namespace validation {

/**
 * @brief 判断数值是否为有限数。
 * @tparam T 数值类型。
 * @param[in] value 输入数值。
 * @return 输入为有限数时返回 `true`。
 */
template <typename T>
inline bool IsFinite(T value) {
  return std::isfinite(value) != 0;
}

/**
 * @brief 构造带索引字段的结构化校验问题。
 * @tparam IssueT 问题结构类型。
 * @tparam SeverityT 严重级别枚举类型。
 * @tparam CodeT 编码枚举类型。
 * @tparam IndexField 问题结构中的索引字段成员指针。
 * @param[in] severity 严重级别。
 * @param[in] code 结构化问题编码。
 * @param[in] index 与问题关联的索引值。
 * @param[in] message 面向调用方的简短说明。
 * @return 构造后的问题结构。
 */
template <typename IssueT, typename SeverityT, typename CodeT, std::size_t IssueT::* IndexField>
inline IssueT MakeIndexedIssue(SeverityT severity, CodeT code, std::size_t index,
                               const std::string& message) {
  IssueT issue;
  issue.severity = severity;
  issue.code = code;
  issue.*IndexField = index;
  issue.message = message;
  return issue;
}

/**
 * @brief 判断问题列表中是否包含指定严重级别。
 * @tparam IssueListT 问题列表类型。
 * @tparam SeverityT 严重级别枚举类型。
 * @tparam SeverityField 问题结构中的严重级别字段成员指针。
 * @param[in] issues 问题列表。
 * @param[in] expected 期望严重级别。
 * @return 存在任意一条匹配问题时返回 `true`。
 */
template <typename IssueListT, typename SeverityT,
          SeverityT IssueListT::value_type::* SeverityField>
inline bool HasSeverity(const IssueListT& issues, SeverityT expected) {
  for (typename IssueListT::const_iterator it = issues.begin(); it != issues.end(); ++it) {
    if ((*it).*SeverityField == expected) {
      return true;
    }
  }
  return false;
}

}  // namespace validation
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_VALIDATION_VALIDATION_UTILS_H_
