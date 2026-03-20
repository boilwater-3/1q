/**
 * @file MathUtils.h
 * @brief 提供通用数学辅助内联函数。
 */

#ifndef AIRBORNE_RADAR_COMMON_MATH_UTILS_H_
#define AIRBORNE_RADAR_COMMON_MATH_UTILS_H_

namespace airborne_radar {
namespace common {

/**
 * @brief 对浮点值执行闭区间限幅。
 * @param value 原始值。
 * @param min_value 下界。
 * @param max_value 上界。
 * @return 限幅后的结果：[min_value, max_value]。
 */
inline float ClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_MATH_UTILS_H_
