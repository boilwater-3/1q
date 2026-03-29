/**
 * @file SurvivabilityEvaluator.h
 * @brief 定义 ECCM 生存性 evaluator，与旧 ECCM 控制节点共享逻辑。
 */

#ifndef AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_H_

#include <cstdint>

#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "airborne_radar/decision/pipeline/TacticalEvaluation.h"

namespace airborne_radar {
namespace decision {
namespace evaluators {

/**
 * @brief SurvivabilityEvaluatorConfig 存储生存性 evaluator 的可调参数。
 */
struct SurvivabilityEvaluatorConfig {
  /**
   * @brief 在收到 ECCM 证据后，`TacticalStateStore::eccm_hold_cycles_remaining`
   *        被重置为该值，用于在证据消失后继续延续 ECCM 保护发射路径的额外周期数。
   * @note 应与 `ControlReducerConfig::eccm_hold_cycles_after_request` 统一规划，
   *       以避免决策层与控制层保持窗口不匹配。
   */
  std::uint32_t eccm_hold_cycles{2};
};

/**
 * @brief 负责生成 ECCM 生存性控制建议。
 */
class SurvivabilityEvaluator final : public pipeline::ITacticalEvaluator {
 public:
  /**
   * @brief 使用配置构造 evaluator。
   * @param config evaluator 配置；默认值保持原有行为。
   */
  explicit SurvivabilityEvaluator(SurvivabilityEvaluatorConfig config = {});

  /**
   * @brief 评估单周期输入并写入 ECCM 提案。
   * @param input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期决策状态存储。
   * @param[in,out] evaluation_state evaluator 间共享的中间结果。
   */
  void Evaluate(const common::DecisionInputFrame& input_frame,
                pipeline::TacticalStateStore& state_store,
                pipeline::TacticalEvaluationState& evaluation_state) const override;

 private:
  SurvivabilityEvaluatorConfig config_;
};

}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_EVALUATORS_SURVIVABILITY_EVALUATOR_H_
