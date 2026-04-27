/**
 * @file JammingSemantics.h
 * @brief 定义供信号层和决策层共享的干扰摘要语义。
 */

#ifndef AIRBORNE_RADAR_COMMON_JAMMING_SEMANTICS_H_
#define AIRBORNE_RADAR_COMMON_JAMMING_SEMANTICS_H_

#include "1q/api.hpp"

namespace airborne_radar {
namespace model {

/**
 * @brief JammingSemantic 表示轨迹级/量测级干扰摘要类型。
 */
enum class ONEQ_API JammingSemantic {
  kNone = 0,         /**< 未观测到有效干扰语义 */
  kNoiseSuppression, /**< 压制式/噪声式干扰占主导 */
  kDeception,        /**< 欺骗式干扰占主导 */
  kRepeater,         /**< 转发式/重复器式干扰占主导 */
  kMixed             /**< 多种干扰类型同时显著存在，无法稳定归并到单一类型 */
};

}  // namespace model
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_JAMMING_SEMANTICS_H_
