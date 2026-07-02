/**
 * @file Constants.h
 * @brief 定义库内共享的物理常量与角度换算工具。
 */

#ifndef COMMON_NUMERICS_CONSTANTS_H_
#define COMMON_NUMERICS_CONSTANTS_H_

namespace oneq {
namespace common {
namespace numerics {

/** @brief 圆周率 (π) */
constexpr long double kPi = 3.14159265358979323846L;

/** @brief 真空光速 (m/s)，CODATA 2018 */
constexpr long double kLightSpeed = 299792458.0L;

/** @brief 玻尔兹曼常数 (J/K)，CODATA 2019 */
constexpr long double kBoltzmann = 1.380649e-23L;

template <typename T>
inline T DegToRad(T angle_deg) {
  return angle_deg * static_cast<T>(kPi) / static_cast<T>(180);
}

template <typename T>
inline T RadToDeg(T angle_rad) {
  return angle_rad * static_cast<T>(180) / static_cast<T>(kPi);
}

}  // namespace numerics
}  // namespace common
namespace internal {
namespace numerics {

using ::oneq::common::numerics::DegToRad;
using ::oneq::common::numerics::kBoltzmann;
using ::oneq::common::numerics::kLightSpeed;
using ::oneq::common::numerics::kPi;
using ::oneq::common::numerics::RadToDeg;

}  // namespace numerics
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_NUMERICS_CONSTANTS_H_
