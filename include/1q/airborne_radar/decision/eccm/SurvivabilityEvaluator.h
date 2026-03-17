// Copyright 2026. All Rights Reserved.
//
// Description: 定义 ECCM 生存性 evaluator，与旧 ECCM 控制节点共享逻辑。

#ifndef AIRBORNE_RADAR_DECISION_ECCM_SURVIVABILITY_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_ECCM_SURVIVABILITY_EVALUATOR_H_

#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/decision/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace eccm {

/// @brief SurvivabilityEvaluator 负责生成 ECCM 生存性控制建议。
class SurvivabilityEvaluator final : public ITacticalEvaluator {
 public:
 /// @brief 评估单周期输入并写入 ECCM 提案。
  void Evaluate(const common::DecisionInputFrame& input_frame,
                TacticalStateStore& state_store,
                TacticalEvaluationState& evaluation_state) const override;
};

} // namespace eccm
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_ECCM_SURVIVABILITY_EVALUATOR_H_
