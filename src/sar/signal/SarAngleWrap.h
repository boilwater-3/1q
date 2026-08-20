/**
 * @file SarAngleWrap.h
 * @brief SAR 内部角度/相位归一化单一源。
 *
 * 常数时间实现（docs/common/contract.md 规则 5：数值归一化必须是常数时间）：
 * std::fmod 一次归约到 (-2π, 2π)，再至多一次条件加减 2π。无界输入
 * （外部仅校验 isfinite 的相位/角度字段）不会引起 while 循环式近似死循环。
 * 边界约定与历史 while 实现一致：输入恰为 +π 的奇数倍返回 +π，恰为 -π 的
 * 奇数倍返回 -π。
 */

#ifndef ONEQ_SRC_SAR_SIGNAL_SAR_ANGLE_WRAP_H_
#define ONEQ_SRC_SAR_SIGNAL_SAR_ANGLE_WRAP_H_

#include <cmath>

namespace sar {
namespace signal {

/**
 * @brief 将相位/角度归一化到 [-π, π]，常数时间。
 *
 * @param phase_rad 输入弧度（需为有限值）。
 * @return 归一化到 [-π, π] 的弧度。
 */
inline double WrapPhase(double phase_rad) {
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  constexpr double kPi = 3.141592653589793238462643383279503;
  double normalized = std::fmod(phase_rad, kTwoPi);
  if (normalized > kPi) {
    normalized -= kTwoPi;
  } else if (normalized < -kPi) {
    normalized += kTwoPi;
  }
  return normalized;
}

}  // namespace signal
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SIGNAL_SAR_ANGLE_WRAP_H_
