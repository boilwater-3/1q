/**
 * @file EmissionControlEvaluator.h
 * @brief 定义 LPI 发射控制 evaluator，与旧 LPI 控制节点共享逻辑。
 */

#ifndef AIRBORNE_RADAR_DECISION_LPI_EMISSION_CONTROL_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_LPI_EMISSION_CONTROL_EVALUATOR_H_

#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "airborne_radar/decision/pipeline/TacticalEvaluation.h"

namespace airborne_radar {
namespace decision {
namespace lpi {

/**
 * @brief 负责生成 LPI 发射控制建议。
 */
class EmissionControlEvaluator final : public pipeline::ITacticalEvaluator {
 public:
  /**
   * @brief 评估单周期输入并写入 LPI 提案。
   * @param input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期决策状态存储。
   * @param[in,out] evaluation_state evaluator 间共享的中间结果。
   */
  void Evaluate(const common::DecisionInputFrame& input_frame,
                pipeline::TacticalStateStore& state_store,
                pipeline::TacticalEvaluationState& evaluation_state) const override;
};

} // namespace lpi
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_LPI_EMISSION_CONTROL_EVALUATOR_H_
