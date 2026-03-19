// Copyright 2026. All Rights Reserved.
//
// @file TacticalEvaluation.h
// @brief 定义默认决策协调器内部评估拼装类型。

#ifndef AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_
#define AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_

#include <vector>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/// @brief TacticalEvaluationState 表示 evaluator 间共享的中间结果。
struct TacticalEvaluationState {
  common::TargetCategoryList target_classification_result;
  common::LpiSourceInfo lpi_source_info;
  common::EccmSourceInfo eccm_source_info;
  bool should_reduce_power{false};
  bool should_enable_eccm{false};
  std::vector<TacticalProposal> proposals;

  TacticalEvaluationState() : eccm_source_info(false) {}
};

/// @brief ITacticalEvaluator 抽象单个 evaluator 的评估接口。
class ITacticalEvaluator {
 public:
  virtual ~ITacticalEvaluator() = default;

  virtual void Evaluate(const common::DecisionInputFrame& input_frame,
                        TacticalStateStore& state_store,
                        TacticalEvaluationState& evaluation_state) const = 0;
};

}  // namespace pipeline
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_
