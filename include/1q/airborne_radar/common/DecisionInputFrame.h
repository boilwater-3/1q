// Copyright 2026. All Rights Reserved.
//
// Description: 定义供决策引擎消费的单周期输入帧。

#ifndef AIRBORNE_RADAR_COMMON_DECISION_INPUT_FRAME_H_
#define AIRBORNE_RADAR_COMMON_DECISION_INPUT_FRAME_H_

#include <cstdint>

#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"

namespace airborne_radar {
namespace common {

/// @brief DecisionInputFrame 表示单周期决策输入帧。
struct DecisionInputFrame {
  /// @brief 当前周期号。
  std::uint32_t cycle_index{0};

  /// @brief 当前批号。
  std::uint64_t batch_id{0};

  /// @brief 环境是否检测到干扰。
  bool environment_jamming_detected{false};

  /// @brief 供 ECCM 消费的干扰事实摘要。
  EccmSourceInfo eccm_source_info{};

  /// @brief 当前周期可见的轨迹快照。
  DecisionTrackSnapshotList tracks{};

  /// @brief 默认构造。
  DecisionInputFrame() = default;

  /// @brief 使用轨迹集合构造输入帧。
  explicit DecisionInputFrame(const DecisionTrackSnapshotList& track_snapshots)
      : tracks(track_snapshots) {}
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_DECISION_INPUT_FRAME_H_
