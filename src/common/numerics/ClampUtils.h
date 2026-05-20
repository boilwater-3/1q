/**
 * @file ClampUtils.h
 * @brief 定义库内共享的基础限幅工具。
 */

#ifndef COMMON_NUMERICS_CLAMP_UTILS_H_
#define COMMON_NUMERICS_CLAMP_UTILS_H_

#include <algorithm>
#include <cmath>

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

/**
 * @brief 当输入为非正或非有限数时返回安全回退值。
 * @tparam T 标量类型。
 * @param[in] value 输入值。
 * @param[in] fallback 回退值。
 * @return 安全限幅后的正值。
 */
template <typename T>
inline T SafePositive(T value, T fallback) {
  if (!std::isfinite(value) || value <= static_cast<T>(0)) {
    return fallback;
  }
  return value;
}

/**
 * @brief 将角度规范化到 [-180, 180] 区间。
 * @param[in] angle_deg 输入角度（单位：deg）。
 * @return 规范化后的角度（单位：deg）。
 */
inline float NormalizeAngle180(float angle_deg) {
  float normalized = angle_deg;
  while (normalized > 180.0f) {
    normalized -= 360.0f;
  }
  while (normalized < -180.0f) {
    normalized += 360.0f;
  }
  return normalized;
}

}  // namespace numerics
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_NUMERICS_CLAMP_UTILS_H_
