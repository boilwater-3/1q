/**
 * @file SurvivabilityEvaluator.h
 * @brief 定义 ECCM 生存性 evaluator，与旧 ECCM 控制节点共享逻辑。
 */

#ifndef AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_H_

#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "airborne_radar/decision/pipeline/TacticalEvaluation.h"

namespace airborne_radar {
namespace decision {
namespace evaluators {

/**
 * @brief 负责生成 ECCM 生存性控制建议。
 */
class SurvivabilityEvaluator final : public pipeline::ITacticalEvaluator {
 public:
  SurvivabilityEvaluator() = default;

  /**
   * @brief 评估单周期输入并写入 ECCM 提案。
   * @param input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期决策状态存储。
   * @param[in,out] evaluation_state evaluator 间共享的中间结果。
   */
  void Evaluate(const model::DecisionInputFrame& input_frame,
                pipeline::TacticalStateStore& state_store,
                pipeline::TacticalEvaluationState& evaluation_state) const override;
};

}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_H_
