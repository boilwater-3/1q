#ifndef ONEQ_COMMON_TRACE_JSON_FORMAT_UTILS_H_
#define ONEQ_COMMON_TRACE_JSON_FORMAT_UTILS_H_

#include <sstream>
#include <string>

namespace oneq {
namespace common {
namespace trace {
namespace internal {

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

inline std::string QuoteString(const std::string& input) {
  return "\"" + EscapeJsonString(input) + "\"";
}

inline std::string BoolToJson(bool value) { return value ? "true" : "false"; }

}  // namespace internal
}  // namespace trace
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_COMMON_TRACE_JSON_FORMAT_UTILS_H_
