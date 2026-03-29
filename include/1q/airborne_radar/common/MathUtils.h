/**
 * @file MathUtils.h
 * @brief 提供通用数学辅助内联函数。
 */

#ifndef AIRBORNE_RADAR_COMMON_MATH_UTILS_H_
#define AIRBORNE_RADAR_COMMON_MATH_UTILS_H_

#include <cmath>
namespace airborne_radar {
namespace common {

/**
 * @brief 对浮点值执行闭区间限幅。
 * @param[in] value 原始值。
 * @param[in] min_value 下界。
 * @param[in] max_value 上界。
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

/**
 * @brief 将 dB 功率值转换为线性功率比 (10^(x/10))。
 * @param[in] power_db 功率值（单位：dB）。
 * @return 线性功率比。
 */
inline float DbToLinearPower(float power_db) { return std::pow(10.0f, power_db / 10.0f); }

}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_MATH_UTILS_H_
