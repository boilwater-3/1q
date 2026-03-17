// Copyright 2026. All Rights Reserved.
//
// Description: 定义 LPI 发射控制 evaluator，与旧 LPI 控制节点共享逻辑。

#ifndef AIRBORNE_RADAR_DECISION_LPI_EMISSION_CONTROL_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_LPI_EMISSION_CONTROL_EVALUATOR_H_

#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/decision/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace lpi {

/// @brief EmissionControlEvaluator 负责生成 LPI 发射控制建议。
class EmissionControlEvaluator final : public ITacticalEvaluator {
 public:
 /// @brief 评估单周期输入并写入 LPI 提案。
  void Evaluate(const common::DecisionInputFrame& input_frame,
                TacticalStateStore& state_store,
                TacticalEvaluationState& evaluation_state) const override;
};

} // namespace lpi
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_LPI_EMISSION_CONTROL_EVALUATOR_H_
