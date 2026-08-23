/**
 * @file RuntimeCycleExecutor.h
 * @brief 定义跨模块复用的单周期执行状态类型（周期标识与输出锁存）。
 */

#ifndef ONEQ_FOUNDATION_RUNTIME_CYCLE_EXECUTOR_H_
#define ONEQ_FOUNDATION_RUNTIME_CYCLE_EXECUTOR_H_

#include <cstdint>

#include "1q/api.hpp"

namespace oneq {
namespace foundation {

/**
 * @brief RuntimeCycleStamp 描述单周期执行时的只读标识。
 */
struct ONEQ_API RuntimeCycleStamp {
  std::uint32_t cycle_index{0U}; /**< 当前周期号。 */
  std::uint64_t batch_id{1U};    /**< 当前批次号。 */
};

/**
 * @brief RuntimeCycleState 描述骨架维护的通用执行状态。
 * @tparam OutputT 输出帧类型。
 */
template <typename OutputT>
struct RuntimeCycleState {
  OutputT latest_output{};        /**< 最近一次周期输出。 */
  bool has_latest_output{false};  /**< 是否已有可读输出。 */
  std::uint64_t next_batch_id{1U}; /**< 下次执行使用的批次号。 */
};

/**
 * @brief 构造单周期标识。
 * @param[in] cycle_index 当前周期号。
 * @param[in] batch_id 当前批次号。
 * @return 只读周期标识。
 */
inline RuntimeCycleStamp MakeRuntimeCycleStamp(std::uint32_t cycle_index, std::uint64_t batch_id) {
  RuntimeCycleStamp stamp;
  stamp.cycle_index = cycle_index;
  stamp.batch_id = batch_id;
  return stamp;
}

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_RUNTIME_CYCLE_EXECUTOR_H_
