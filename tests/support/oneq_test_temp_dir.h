// 测试临时目录统一入口。
//
// 优先使用 CMake 注入的 ONEQ_TEST_TEMP_DIR（见 tests/CMakeLists.txt：
// Windows 取 %TEMP%，macOS/Linux 取 /tmp，经 add_1q_gtest /
// oneq_add_test_partition 传给全部测试 target）。未注入时（非 CMake 构建）
// 回退到 /tmp，保证既有平台行为不变。
#ifndef ONEQ_TEST_TEMP_DIR
#define ONEQ_TEST_TEMP_DIR "/tmp"
#endif

#ifndef ONEQ_TEST_TEMP_DIR_H_
#define ONEQ_TEST_TEMP_DIR_H_

#include <string>

namespace oneq_test {

// 测试临时目录（末尾带平台路径分隔符）。
inline std::string TempDir() {
  std::string dir = ONEQ_TEST_TEMP_DIR;
  const char last = dir.empty() ? '\0' : dir[dir.size() - 1U];
  if (last != '/' && last != '\\') {
    dir.push_back('/');
  }
  return dir;
}

}  // namespace oneq_test

#endif  // ONEQ_TEST_TEMP_DIR_H_
