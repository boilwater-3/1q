/**
 * @file TacticalEvaluation.h
 * @brief 定义默认决策协调器内部评估拼装类型。
 */

#ifndef AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_
#define AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_

#include <vector>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/**
 * @brief 表示 evaluator 间共享的中间结果。
 */
struct TacticalEvaluationState {
  common::TargetCategoryList target_classification_result; /**< 当前周期的目标分类结果。 */
  common::LpiSourceInfo lpi_source_info;                   /**< 供 LPI evaluator 使用的来源信息。 */
  common::EccmSourceInfo eccm_source_info; /**< 供 ECCM evaluator 使用的来源信息。 */
  bool should_reduce_power{false};         /**< 是否应触发降功率路径。 */
  bool should_enable_eccm{false};          /**< 是否应触发 ECCM 保护发射路径。 */
  std::vector<TacticalProposal> proposals; /**< 当前周期累计的控制提案列表。 */

  TacticalEvaluationState() : eccm_source_info(false) {}
};

/**
 * @brief 抽象单个 evaluator 的评估接口。
 */
class ITacticalEvaluator {
 public:
  virtual ~ITacticalEvaluator() = default;

  /**
   * @brief 执行单个 evaluator 的评估逻辑。
   * @param input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期战术状态存储。
   * @param[in,out] evaluation_state evaluator 间共享的中间结果。
   */
  virtual void Evaluate(const common::DecisionInputFrame& input_frame,
                        TacticalStateStore& state_store,
                        TacticalEvaluationState& evaluation_state) const = 0;
};

}  // namespace pipeline
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_PIPELINE_TACTICAL_EVALUATION_H_
