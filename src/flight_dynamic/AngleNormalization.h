/**
 * @file AngleNormalization.h
 * @brief 角度归一化单一源：NormalizeRad（→[-π,π]）与 RadToDeg360（→[0,360)）。
 *
 * Autopilot 与 Maneuver（guidance）原先各自维护一份逐字相同的 NormalizeRad，
 * 是漂移源头。本单元将其唯一化为一份。实现使用 std::fmod 常数时间归一化，
 * 满足 docs/common/contract.md 规则 5（数值归一化必须是常数时间）：
 * 无界输入（如 >>2π 或 ±inf）不会引起近似死循环。边界约定与历史 while 循环
 * 实现一致（输入恰为 +π 的奇数倍时返回 +π，恰为 -π 的奇数倍时返回 -π）。
 *
 * 本单元为模块内部私有，不对外暴露。仅使用 C++11。
 */

#ifndef FLIGHT_DYNAMIC_ANGLE_NORMALIZATION_H_
#define FLIGHT_DYNAMIC_ANGLE_NORMALIZATION_H_

#include <cmath>

#include "common/numerics/Constants.h"

namespace oneq {
namespace flight_dynamic {

using oneq::common::numerics::kPi;
using oneq::common::numerics::kTwoPi;
using oneq::common::numerics::RadToDeg;

/**
 * @brief 将弧度角归一化到 [-π, π]。
 *
 * 常数时间实现（contract 规则 5）：std::fmod 一次归约到 (-2π, 2π)，
 * 再至多一次条件加减 2π。近界输入结果与历史 while 实现逐位一致。
 *
 * @param angle_rad 输入弧度。
 * @return 归一化到 [-π, π] 的弧度。
 */
inline double NormalizeRad(double angle_rad) {
  double normalized = std::fmod(angle_rad, kTwoPi);
  if (normalized > kPi) {
    normalized -= kTwoPi;
  } else if (normalized < -kPi) {
    normalized += kTwoPi;
  }
  return normalized;
}

/**
 * @brief 将弧度角转为度并归一化到 [0, 360)。
 *
 * 常数时间实现（contract 规则 5）：std::fmod 一次归约到 (-360, 360)，
 * 负值补一个 360。
 *
 * @param angle_rad 输入弧度。
 * @return 归一化到 [0, 360) 的度。
 */
inline double RadToDeg360(double angle_rad) {
  double deg = std::fmod(RadToDeg(angle_rad), 360.0);
  if (deg < 0.0) {
    deg += 360.0;
  }
  return deg;
}

}  // namespace flight_dynamic
}  // namespace oneq

#endif  // FLIGHT_DYNAMIC_ANGLE_NORMALIZATION_H_
