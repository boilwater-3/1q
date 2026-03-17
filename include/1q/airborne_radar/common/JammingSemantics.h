// Copyright 2026. All Rights Reserved.
//
// Description: 定义供信号层和决策层共享的干扰摘要语义。

#ifndef AIRBORNE_RADAR_COMMON_JAMMING_SEMANTICS_H_
#define AIRBORNE_RADAR_COMMON_JAMMING_SEMANTICS_H_

namespace airborne_radar {
namespace common {

/// @brief JammingSemantic 表示轨迹级/量测级干扰摘要类型。
enum class JammingSemantic {
  /// @brief 未观测到有效干扰语义。
  kNone = 0,
  /// @brief 压制式/噪声式干扰占主导。
  kNoiseSuppression,
  /// @brief 欺骗式干扰占主导。
  kDeception,
  /// @brief 转发式/重复器式干扰占主导。
  kRepeater,
  /// @brief 多种干扰类型同时显著存在，无法稳定归并到单一类型。
  kMixed
};

}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_JAMMING_SEMANTICS_H_
