#ifndef ONEQ_SRC_COMMON_TRACE_TIME_UTILS_H_
#define ONEQ_SRC_COMMON_TRACE_TIME_UTILS_H_

#include <chrono>
#include <cstdint>

namespace oneq {
namespace common {
namespace trace {

inline std::int64_t CurrentTimestampMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace trace
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_SRC_COMMON_TRACE_TIME_UTILS_H_
