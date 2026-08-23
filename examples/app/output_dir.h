/**
 * @file output_dir.h
 * @brief 场景输出目录策略：临时区硬拒（强制约束，ECS 栈与精度评估栈共用）。
 *
 * 场景产物（集成日志/CSV/验收文件）必须落在可追溯的证据目录——场景 JSON 的
 * log_dir 声明输出位置（相对 examples/log/）；CLI --output-dir 仅作非临时目录
 * 的调试覆盖。本助手对最终解析出的目录做词法归一化后判临时区前缀
 * （/tmp、/var/tmp、TMPDIR；Windows 另查 TEMP/TMP），命中即由调用方报错退出。
 */

#ifndef EXAMPLES_APP_OUTPUT_DIR_H_
#define EXAMPLES_APP_OUTPUT_DIR_H_

#include <cstdlib>
#include <string>
#include <vector>

#include "app/fs_compat.h"

namespace component_attachment {
namespace app {

/// 词法路径归一化：合并分隔符、消解 "." 与 ".." 段（不做文件系统查询；
/// 相对路径先按当前工作目录拼成绝对语义再消解）。
inline std::string LexicallyNormalize(const std::string& path) {
  std::string flat = path;
  for (char& c : flat) {
    if (c == '\\') {
      c = '/';
    }
  }
  std::string base;
  if (flat.empty() || flat[0] != '/') {
    std::error_code ec;
    base = app_fs::current_path(ec).string();
    for (char& c : base) {
      if (c == '\\') {
        c = '/';
      }
    }
    if (!base.empty() && base[0] == '/') {
      flat = base + "/" + flat;
    }
  }
  const bool rooted = !flat.empty() && flat[0] == '/';
  std::vector<std::string> parts;
  std::size_t pos = 0U;
  while (pos < flat.size()) {
    while (pos < flat.size() && flat[pos] == '/') {
      ++pos;
    }
    const std::size_t start = pos;
    while (pos < flat.size() && flat[pos] != '/') {
      ++pos;
    }
    if (pos <= start) {
      break;
    }
    const std::string seg = flat.substr(start, pos - start);
    if (seg == ".") {
      continue;
    }
    if (seg == ".." && !parts.empty() && parts.back() != "..") {
      parts.pop_back();
      continue;
    }
    parts.push_back(seg);
  }
  std::string out = rooted ? "/" : "";
  for (std::size_t i = 0U; i < parts.size(); ++i) {
    if (i != 0U) {
      out += "/";
    }
    out += parts[i];
  }
  return out;
}

/// 目录是否位于系统临时区（词法归一化后做前缀判定；命中 = 调用方拒绝）。
inline bool IsInsideTempArea(const std::string& dir) {
  const std::string norm = LexicallyNormalize(dir);
  std::vector<std::string> roots;
  roots.push_back("/tmp");
  roots.push_back("/var/tmp");
  const char* envs[] = {"TMPDIR", "TEMP", "TMP"};
  for (const char* name : envs) {
    const char* value = std::getenv(name);
    if (value != nullptr && value[0] != '\0') {
      roots.push_back(LexicallyNormalize(value));
    }
  }
  for (std::size_t i = 0U; i < roots.size(); ++i) {
    const std::string& root = roots[i];
    if (norm == root || (norm.size() > root.size() &&
                         norm.compare(0U, root.size(), root) == 0 &&
                         norm[root.size()] == '/')) {
      return true;
    }
  }
  return false;
}

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_APP_OUTPUT_DIR_H_
