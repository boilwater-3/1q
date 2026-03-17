// Copyright 2026. All Rights Reserved.
//
// Description: 定义控制意图到控制真值的归并器。

#ifndef AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
#define AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_

#include <vector>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/decision/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {

/// @brief ControlReductionResult 表示 reducer 的单周期输出。
struct ControlReductionResult {
  /// @brief 归并后的下一周期控制真值。
  common::RadarControlProfile profile;

  /// @brief 被采纳的控制意图。
  std::vector<common::ControlDirective> applied_directives;

  /// @brief 被拒绝的控制意图。
  std::vector<common::ControlDirective> rejected_directives;
};

/// @brief ControlReducer 负责把控制意图归并为唯一控制真值。
class ControlReducer {
 public:
  /// @brief 使用上一版 profile 和 proposal 列表生成下一版 profile。
  ControlReductionResult Reduce(
      const common::RadarControlProfile& previous_profile,
      const std::vector<TacticalProposal>& proposals) const;
};

} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
