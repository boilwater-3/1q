/**
 * @file EsrSharedUtils.h
 * @brief ESR 模块内部共享工具函数，消除各子模块重复定义。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ESR_SHARED_UTILS_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ESR_SHARED_UTILS_H_

#include "common/numerics/ClampUtils.h"

namespace electronic_surveillance_radar {
namespace utils {

/**
 * @brief 将输入裁剪到 [0, 1]。
 * @param[in] value 待裁剪的浮点值。
 * @return 裁剪到 [0, 1] 后的值。
 */
inline float Clamp01(float value) { return oneq::common::numerics::Clamp01(value); }

/**
 * @brief 将输入裁剪到非负区间。
 * @param[in] value 待裁剪的浮点值。
 * @return 裁剪到非负区间后的值。
 */
inline float ClampNonNegative(float value) {
  return oneq::common::numerics::ClampNonNegative(value);
}

}  // namespace utils
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ESR_SHARED_UTILS_H_
