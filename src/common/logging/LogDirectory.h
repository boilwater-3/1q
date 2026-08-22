/**
 * @file LogDirectory.h
 * @brief Create the parent directory of a log file path (no exceptions).
 */

#ifndef ONEQ_SRC_COMMON_LOGGING_LOG_DIRECTORY_H_
#define ONEQ_SRC_COMMON_LOGGING_LOG_DIRECTORY_H_

#include <cerrno>
#include <string>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

namespace oneq {
namespace logging {
namespace {

inline bool CreateDirectoryIfMissing(const std::string& path) {
  if (path.empty()) {
    return true;
  }
#if defined(_WIN32)
  const int result = _mkdir(path.c_str());
#else
  const int result = mkdir(path.c_str(), 0777);
#endif
  return result == 0 || errno == EEXIST;
}

}  // namespace

inline void EnsureParentDirectory(const char* file_path) {
  if (file_path == nullptr || *file_path == '\0') {
    return;
  }
  std::string path(file_path);
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (path[i] == '\\') {
      path[i] = '/';
    }
  }
  const std::size_t slash = path.find_last_of('/');
  if (slash == std::string::npos || slash == 0U) {
    return;
  }
  std::string current;
  for (std::size_t i = 0; i < slash; ++i) {
    current.push_back(path[i]);
    if (i == 1U && path[i] == ':') {
      continue;
    }
    if (path[i] == '/') {
      if (current.size() == 3U && current[1] == ':') {
        continue;
      }
      if (current != "/" && !CreateDirectoryIfMissing(current)) {
        return;
      }
    }
  }
  (void)CreateDirectoryIfMissing(path.substr(0, slash));
}

}  // namespace logging
}  // namespace oneq

#endif
