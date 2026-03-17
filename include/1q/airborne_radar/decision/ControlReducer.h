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

/// @brief ControlReducerConfig 描述 proposal -> profile 的固定映射与冲突裁决策略。
struct ControlReducerConfig {
  /// @brief LPI 降功率意图映射到的默认功率比例。
  float lpi_power_scale_on_reduction{0.5f};

  /// @brief LPI 驻留调整意图映射到的默认驻留比例。
  float lpi_dwell_scale{0.75f};

  /// @brief ECCM 烧穿意图映射到的默认增益倍率。
  float eccm_burnthrough_gain{1.5f};

  /// @brief 当烧穿增益与 LPI 降功率并存时，对功率比例施加的保护下限。
  float burnthrough_lpi_power_floor{0.85f};

  /// @brief 是否在烧穿/LPI 冲突时优先生存性。
  bool prefer_survivability_in_power_conflict{true};
};

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

} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
