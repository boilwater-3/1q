/**
 * @file logger_format.h
 * @brief 集成端日志的字符串格式化门面（双后端，仿库内 ProjectLog 开关先例）。
 *
 * CA_LOG_BACKEND_SPDLOG 由 CMake 按 PROJECT_ENABLE_SPDLOG 注入（spdlog 平台
 * =1，Windows =0）：
 * 1. spdlog 分支：CA_FMT_FORMAT 一比一展开为 spdlog::fmt_lib::format——与历史
 *    版本完全同源（字面量格式串保留 fmt 编译期检查，运行时语义零变化）；
 * 2. 文件后端分支（Windows，spdlog/fmt 均不安装）：自实现迷你格式化，子集与
 *    库内 ProjectFileLog 的 FormatOne 对齐——{}（浮点走 6 位有效数字，与库
 *    Windows 后端一致的既有偏差，非 fmt 最短往返表示）、{:.Nf} 定点、{{ }}
 *    转义；未知 spec 形态按防御契约原样输出占位符并照常消耗一个参数；多余
 *    参数忽略、占位符多于参数时剩余占位符原样输出。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_FORMAT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_FORMAT_H_

#if defined(CA_LOG_BACKEND_SPDLOG) && CA_LOG_BACKEND_SPDLOG

#include <spdlog/spdlog.h>

/// spdlog 分支：直接转发 fmt（字面量格式串保留编译期检查）。
#define CA_FMT_FORMAT(...) ::spdlog::fmt_lib::format(__VA_ARGS__)

#else

#include <cctype>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <utility>

namespace component_attachment {
namespace demo {
namespace format_internal {

/// 单参数写入：spec 为空 → 默认格式（整型/字符串精确；浮点 6 位有效数字）；
/// spec 为 ".<N>f" → 定点 N 位小数；其他形态 → 占位符原样输出（参数已消耗，
/// 防御契约与库内 ProjectFileLog::FormatOne 一致）。
template <typename T>
void FormatOne(std::string& result, const std::string& raw_spec, const T& value) {
  // fmt 语法中 ':' 引导占位符内的格式说明（"{}" 无冒号，"{:.2f}" 的说明为
  // ".2f"）——先剥掉可选前导冒号再解析。
  std::string spec = raw_spec;
  if (!spec.empty() && spec[0] == ':') {
    spec.erase(0, 1U);
  }
  if (spec.empty()) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << value;
    result += out.str();
    return;
  }
  // "{:.Nf}"：'.' + 纯数字 + 'f'（本示例全部定点用法为 N=0..7）。
  if (spec.size() >= 3U && spec[0] == '.' && spec[spec.size() - 1U] == 'f') {
    bool digits_only = true;
    for (std::string::size_type i = 1U; i + 1U < spec.size(); ++i) {
      if (!std::isdigit(static_cast<unsigned char>(spec[i]))) {
        digits_only = false;
        break;
      }
    }
    if (digits_only) {
      std::istringstream precision_in(spec.substr(1U, spec.size() - 2U));
      int precision = 0;
      precision_in >> precision;
      std::ostringstream out;
      out.imbue(std::locale::classic());
      out << std::fixed << std::setprecision(precision) << value;
      result += out.str();
      return;
    }
  }
  result += '{';
  result += spec;
  result += '}';
}

/// 参数耗尽后的剩余格式串：{{ }} 仍解转义，占位符原样输出（防御，不抛错）。
inline void AppendFormat(std::string& result, const char*& fmt) {
  while (*fmt != '\0') {
    if (fmt[0] == '{' && fmt[1] == '{') {
      result += '{';
      fmt += 2;
      continue;
    }
    if (fmt[0] == '}' && fmt[1] == '}') {
      result += '}';
      fmt += 2;
      continue;
    }
    result += *fmt++;
  }
}

/// 逐占位符消耗参数：每个完整占位符消耗一个参数；{{ }} 转义不消耗；未闭合
/// 的 '{' 尾巴整体原样输出并停止消耗（多余参数忽略）。
template <typename T, typename... Rest>
void AppendFormat(std::string& result, const char*& fmt, T&& value,
                  Rest&&... rest) {
  while (*fmt != '\0') {
    if (fmt[0] == '{' && fmt[1] == '{') {
      result += '{';
      fmt += 2;
      continue;
    }
    if (fmt[0] == '}' && fmt[1] == '}') {
      result += '}';
      fmt += 2;
      continue;
    }
    if (fmt[0] != '{') {
      result += *fmt++;
      continue;
    }
    const char* close = fmt + 1;
    while (*close != '\0' && *close != '}') {
      ++close;
    }
    if (*close == '\0') {
      result += fmt;  // 未闭合：尾巴原样输出，剩余参数忽略
      return;
    }
    FormatOne(result, std::string(fmt + 1, close), value);
    fmt = close + 1;
    AppendFormat(result, fmt, std::forward<Rest>(rest)...);
    return;
  }
}

}  // namespace format_internal

/// 文件后端分支的格式化入口（签名与 spdlog::fmt_lib::format 对齐：格式串 +
/// 逐参数）。支持子集见文件头注释；本示例全部调用点仅用 {} 与 {:.Nf}。
template <typename... Args>
std::string FmtFormat(const char* fmt, Args&&... args) {
  std::string result;
  const char* cursor = fmt;
  format_internal::AppendFormat(result, cursor, std::forward<Args>(args)...);
  return result;
}

}  // namespace demo
}  // namespace component_attachment

/// 文件后端分支：统一走本目录的迷你格式化实现。
#define CA_FMT_FORMAT(...) ::component_attachment::demo::FmtFormat(__VA_ARGS__)

#endif  // CA_LOG_BACKEND_SPDLOG

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_LOGGER_LOGGER_FORMAT_H_
