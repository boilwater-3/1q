#include "airborne_radar/decision/evaluators/EmissionControlEvaluator.h"

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {
namespace evaluators {

namespace {

/**
 * @brief LPI 降功率提案的保持周期计数。
 * @note 代码行为依据：每当降功率路径成立时，当前实现都会把
 *       `state_store.lpi_hold_cycles_remaining` 重置为该值，用于维持后续周期的
 *       LPI 提案输出窗口。
 */
const std::uint32_t kLpiHoldCycles = 2;
/**
 * @brief 构造一条 LPI 降功率控制意图。
 * @return 返回类型为 `REQUEST_LPI_POWER_REDUCTION` 且 `source` 字段为
 *         `EMISSION_CONTROL` 的控制意图。
 */
common::ControlDirective BuildLpiPowerDirective() {
  return common::ControlDirective(common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                                  common::ControlDirectiveSource::EMISSION_CONTROL);
}

}  // namespace

void EmissionControlEvaluator::Evaluate(const common::DecisionInputFrame& input_frame,
                                        pipeline::TacticalStateStore& state_store,
                                        pipeline::TacticalEvaluationState& evaluation_state) const {
  // input_frame 在此评估器中未使用：LPI 降功率条件由上游
  // ThreatAssessmentEvaluator / BackfillAssociationDrivenEccmTrigger 写入
  // evaluation_state.should_reduce_power，本评估器仅负责保持计数窗口和生成提案。
  bool should_reduce_power = evaluation_state.should_reduce_power;
  if (!should_reduce_power && state_store.lpi_hold_cycles_remaining > 0U) {
    should_reduce_power = true;
    --state_store.lpi_hold_cycles_remaining;
  }

  evaluation_state.should_reduce_power = should_reduce_power;
  if (!should_reduce_power) {
    PROJECT_LOG_INFO("[EmissionControlEvaluator] No severe threat. LPI remains inactive.");
    return;
  }

  state_store.lpi_hold_cycles_remaining = kLpiHoldCycles;
  evaluation_state.proposals.push_back(pipeline::TacticalProposal{
      BuildLpiPowerDirective(), 60, "high-confidence threat requires reduced emission"});
  PROJECT_LOG_INFO(
      "[EmissionControlEvaluator] High threat detected. Appending proposal: "
      "REQUEST_LPI_POWER_REDUCTION");
}

}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar
