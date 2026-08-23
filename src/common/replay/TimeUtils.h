/**
 * @file TimeUtils.h
 * @brief 定义 Replay 元数据时间戳工具。
 */

#ifndef ONEQ_SRC_COMMON_REPLAY_TIME_UTILS_H_
#define ONEQ_SRC_COMMON_REPLAY_TIME_UTILS_H_

#include <chrono>
#include <cstdint>

namespace oneq {
namespace common {
namespace replay {

/**
 * @brief 获取当前 Unix 纪元毫秒级时间戳。
 * @return 自 Unix epoch 起的毫秒数。
 */
inline std::int64_t CurrentTimestampMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace replay
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_SRC_COMMON_REPLAY_TIME_UTILS_H_
