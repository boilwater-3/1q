/**
 * @file Constants.h
 * @brief 定义库内共享的物理常量与角度换算工具。
 */

#ifndef COMMON_NUMERICS_CONSTANTS_H_
#define COMMON_NUMERICS_CONSTANTS_H_

namespace oneq {
namespace internal {
namespace numerics {
namespace constants {

/** @brief 圆周率 (π) */
constexpr long double kPi = 3.14159265358979323846L;

/** @brief 真空光速 (m/s)，CODATA 2018 */
constexpr long double kLightSpeed = 299792458.0L;

/** @brief 玻尔兹曼常数 (J/K)，CODATA 2019 */
constexpr long double kBoltzmann = 1.380649e-23L;

/**
 * @brief 将角度转换为弧度。
 * @tparam T 标量类型。
 * @param[in] angle_deg 角度（单位：deg）。
 * @return 弧度。
 */
template <typename T>
inline T DegToRad(T angle_deg) {
  return angle_deg * static_cast<T>(kPi) / static_cast<T>(180);
}

/**
 * @brief 将弧度转换为角度。
 * @tparam T 标量类型。
 * @param[in] angle_rad 弧度（单位：rad）。
 * @return 角度。
 */
template <typename T>
inline T RadToDeg(T angle_rad) {
  return angle_rad * static_cast<T>(180) / static_cast<T>(kPi);
}

}  // namespace constants
}  // namespace numerics
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_NUMERICS_CONSTANTS_H_
