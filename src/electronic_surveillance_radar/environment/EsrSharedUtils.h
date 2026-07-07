/**
 * @file EsrSharedUtils.h
 * @brief ESR 模块内部共享工具函数，消除各子模块重复定义。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ESR_SHARED_UTILS_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ESR_SHARED_UTILS_H_

#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
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

/**
 * @brief 解析干扰技术类型并应用兼容推断。
 *
 * 当源技术类型为已知值时直接返回；否则按 deception_risk 推断：
 * 大于 0 视为压制+欺骗并存（kMixed），否则归为纯压制（kNoiseSuppression）。
 *
 * @param[in] source 干扰源输入。
 * @return 解析后的干扰技术类型。
 */
inline session::EsrJammingTechnique ResolveTechnique(
    const session::EsrJammerSource& source) {
  if (source.technique != session::EsrJammingTechnique::kUnknown) {
    return source.technique;
  }
  if (source.deception_risk > 0.0f) {
    return session::EsrJammingTechnique::kMixed;
  }
  return session::EsrJammingTechnique::kNoiseSuppression;
}

/**
 * @brief 判断技术类型是否包含压制分量。
 * @param[in] technique 干扰技术类型。
 * @return 为压制或混合类型时返回 `true`。
 */
inline bool HasSuppressionEffect(session::EsrJammingTechnique technique) {
  return technique == session::EsrJammingTechnique::kNoiseSuppression ||
         technique == session::EsrJammingTechnique::kMixed;
}

/**
 * @brief 判断技术类型是否包含欺骗分量。
 * @param[in] technique 干扰技术类型。
 * @return 为欺骗或混合类型时返回 `true`。
 */
inline bool HasDeceptionEffect(session::EsrJammingTechnique technique) {
  return technique == session::EsrJammingTechnique::kDeception ||
         technique == session::EsrJammingTechnique::kMixed;
}

}  // namespace utils
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ESR_SHARED_UTILS_H_
