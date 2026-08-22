/**
 * @file acceptance_timing.h
 * @brief 示例层验收计时：墙钟毫秒写入 integration_events.log。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_ACCEPTANCE_TIMING_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_ACCEPTANCE_TIMING_H_

#include <chrono>
#include <cstdint>

#include "logger/logger.h"

namespace component_attachment {
namespace demo {

inline double SteadyElapsedMs(const std::chrono::steady_clock::time_point& begin) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
      .count();
}

inline void LogAcceptanceMs(std::uint64_t cycle, double t_sec, const char* item,
                            const char* module, double ms) {
  LogEvent(cycle, t_sec, "acceptance",
           CA_FMT_FORMAT("[验收项：{}] 验收内容：{:.3f}ms 模块={}", item, ms, module));
}

}  // namespace demo
}  // namespace component_attachment

#endif
