/**
 * @file ClampUtils.h
 * @brief 定义库内共享的基础限幅工具。
 */

#ifndef COMMON_NUMERICS_CLAMP_UTILS_H_
#define COMMON_NUMERICS_CLAMP_UTILS_H_

#include <algorithm>

namespace oneq {
namespace internal {
namespace numerics {

/**
 * @brief 将输入裁剪到给定区间。
 * @tparam T 标量类型。
 * @param[in] value 输入值。
 * @param[in] lower 下界。
 * @param[in] upper 上界。
 * @return 裁剪后的结果。
 */
template <typename T>
inline T Clamp(T value, T lower, T upper) {
  return std::max(lower, std::min(value, upper));
}

/**
 * @brief 将输入裁剪到 [0, 1] 区间。
 * @tparam T 标量类型。
 * @param[in] value 输入值。
 * @return 裁剪后的结果。
 */
template <typename T>
inline T Clamp01(T value) {
  return Clamp(value, static_cast<T>(0), static_cast<T>(1));
}

/**
 * @brief 将输入裁剪到非负区间。
 * @tparam T 标量类型。
 * @param[in] value 输入值。
 * @return 裁剪后的结果。
 */
template <typename T>
inline T ClampNonNegative(T value) {
  return std::max(value, static_cast<T>(0));
}

}  // namespace numerics
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_NUMERICS_CLAMP_UTILS_H_
