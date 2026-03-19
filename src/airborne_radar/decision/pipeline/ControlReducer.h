// Copyright 2026. All Rights Reserved.
//
// @file ControlReducer.h
// @brief 定义控制意图到控制真值的私有归并器实现。

#ifndef AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
#define AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_

#include "1q/airborne_radar/decision/pipeline/ControlReducerTypes.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/// @brief ControlReducer 负责把控制意图归并为唯一控制真值。
class ControlReducer {
 public:
  /// @brief 使用配置构造 reducer。
  explicit ControlReducer(ControlReducerConfig config = {});

  /// @brief 更新 reducer 配置。
  void UpdateConfig(ControlReducerConfig config);

  /// @brief 获取当前 reducer 配置。
  ControlReducerConfig GetConfig() const;

  /// @brief 使用上一版 profile 和 proposal 列表生成下一版 profile。
  ControlReductionResult Reduce(
      const common::RadarControlProfile& previous_profile,
      const std::vector<TacticalProposal>& proposals) const;

 private:
  ControlReducerConfig config_{};
};

} // namespace pipeline
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
