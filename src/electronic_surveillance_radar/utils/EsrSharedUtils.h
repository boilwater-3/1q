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
 */
inline float Clamp01(float value) { return oneq::internal::numerics::Clamp01(value); }

/**
 * @brief 将输入裁剪到非负区间。
 */
inline float ClampNonNegative(float value) {
  return oneq::internal::numerics::ClampNonNegative(value);
}

/**
 * @brief 解析干扰技术类型并应用兼容推断。
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
 */
inline bool HasSuppressionEffect(session::EsrJammingTechnique technique) {
  return technique == session::EsrJammingTechnique::kNoiseSuppression ||
         technique == session::EsrJammingTechnique::kMixed;
}

/**
 * @brief 判断技术类型是否包含欺骗分量。
 */
inline bool HasDeceptionEffect(session::EsrJammingTechnique technique) {
  return technique == session::EsrJammingTechnique::kDeception ||
         technique == session::EsrJammingTechnique::kMixed;
}

}  // namespace utils
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ESR_SHARED_UTILS_H_
