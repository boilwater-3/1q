/**
 * @file NumericGuard.h
 * @brief 统一的数值防护工具集，提供跨模块共享的数值下限和常用安全运算。
 */

#ifndef COMMON_NUMERICS_NUMERIC_GUARD_H_
#define COMMON_NUMERICS_NUMERIC_GUARD_H_

#include <algorithm>
#include <cmath>
#include <limits>

namespace oneq {
namespace internal {
namespace numerics {

/** @brief 对数域输入防护下限，防止 log10(0) 产生 -inf。 */
constexpr double kLog10Floor = 1.0e-30;

/** @brief 通用除零/非正防护下限。 */
constexpr double kNumericFloor = 1.0e-18;

/** @brief 协方差矩阵对角元素正定性防护下限。 */
constexpr float kCovarianceFloor = 1.0e-6f;

/**
 * @brief 安全的 log10 运算，输入钳制到下限。
 * @param value 输入值。
 * @return 10 * log10(std::max(value, kLog10Floor))。
 */
inline double SafeLog10(double value) {
  return 10.0 * std::log10(std::max(value, kLog10Floor));
}

/**
 * @brief 安全的倒数运算，分母钳制到下限。
 * @param value 输入值。
 * @return 1.0 / std::max(value, kNumericFloor)。
 */
inline double SafeInverse(double value) {
  return 1.0 / std::max(value, kNumericFloor);
}

/**
 * @brief 数值安全的正值钳制。
 * @param value 输入值。
 * @param fallback 无效输入时的回退值。
 * @return 有限正数时返回原值，否则返回 fallback。
 */
inline double SafePositive(double value, double fallback) {
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return value;
}

}  // namespace numerics
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_NUMERICS_NUMERIC_GUARD_H_
