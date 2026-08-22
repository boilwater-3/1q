/**
 * @file JsonFormatUtils.h
 * @brief 定义 Replay 元数据序列化复用的轻量 JSON 文本工具。
 */

#ifndef ONEQ_REPLAY_JSON_FORMAT_UTILS_H_
#define ONEQ_REPLAY_JSON_FORMAT_UTILS_H_

#include <sstream>
#include <string>

namespace oneq {
namespace common {
namespace replay {

/**
 * @brief 对字符串做 JSON 转义（控制字符替换为空格，不含外层引号）。
 * @param[in] input 待转义文本。
 * @return 转义后的字符串。
 */
inline std::string EscapeJsonString(const std::string& input) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const char c = input[i];
    switch (c) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20U) {
          stream << " ";
        } else {
          stream << c;
        }
        break;
    }
  }
  return stream.str();
}

/**
 * @brief 将字符串转义并用双引号包裹为 JSON 字面量。
 * @param[in] input 待转义文本。
 * @return 形如 "..." 的 JSON 字面量。
 */
inline std::string QuoteString(const std::string& input) {
  return "\"" + EscapeJsonString(input) + "\"";
}

/**
 * @brief 将布尔值序列化为 JSON 字面量。
 * @param[in] value 输入布尔值。
 * @return "true" 或 "false"。
 */
inline std::string BoolToJson(bool value) { return value ? "true" : "false"; }

}  // namespace replay
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_REPLAY_JSON_FORMAT_UTILS_H_
