/**
 * @file OutputFrameUtils.h
 * @brief 定义输出帧构建流程复用的无业务语义工具函数。
 */

#ifndef COMMON_OUTPUT_OUTPUT_FRAME_UTILS_H_
#define COMMON_OUTPUT_OUTPUT_FRAME_UTILS_H_

#include <cstddef>
#include <cstdint>

namespace oneq {
namespace internal {
namespace output {

/**
 * @brief 统计容器内满足谓词条件的元素数量。
 * @tparam ContainerT 容器类型。
 * @tparam PredicateT 谓词类型。
 * @param[in] container 待统计容器。
 * @param[in] predicate 判定谓词。
 * @return 满足谓词的元素数量。
 */
template <typename ContainerT, typename PredicateT>
inline std::size_t CountMatching(const ContainerT& container, const PredicateT& predicate) {
  std::size_t count = 0U;
  for (typename ContainerT::const_iterator it = container.begin(); it != container.end(); ++it) {
    if (predicate(*it)) {
      ++count;
    }
  }
  return count;
}

/**
 * @brief 统一写入输出帧头的周期号与批次号。
 * @tparam FrameT 帧类型。
 * @param[out] frame 待写入帧。
 * @param[in] cycle_index 周期号。
 * @param[in] batch_id 批次号。
 */
template <typename FrameT>
inline void SetCycleAndBatch(FrameT& frame, std::uint32_t cycle_index, std::uint64_t batch_id) {
  frame.cycle_index = cycle_index;
  frame.batch_id = batch_id;
}

}  // namespace output
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_OUTPUT_OUTPUT_FRAME_UTILS_H_
