/**
 * @file EosPhysicalConstants.h
 * @brief 定义 EOS 内部物理/数学常量。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PHYSICAL_CONSTANTS_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PHYSICAL_CONSTANTS_H_

#include "common/numerics/Constants.h"

namespace electro_optical_sensor {
namespace foundation {
namespace constants {

constexpr float kPi = static_cast<float>(oneq::common::numerics::kPi); /**< 圆周率常量（float 精度，派生自公共数值单一源） */

}  // namespace constants
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PHYSICAL_CONSTANTS_H_
