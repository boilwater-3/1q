#include "airborne_radar/decision/evaluators/EmissionControlEvaluator.h"

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {
namespace evaluators {

namespace {

/**
 * @brief 构造一条 LPI 降功率控制意图。
 * @return 返回类型为 `REQUEST_LPI_POWER_REDUCTION` 且 `source` 字段为
 *         `EMISSION_CONTROL` 的控制意图。
 */
extension::control::ControlDirective BuildLpiPowerDirective() {
  return extension::control::ControlDirective(
      extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
      extension::control::ControlDirectiveSource::EMISSION_CONTROL);
}

}  // namespace

void EmissionControlEvaluator::Evaluate(const model::DecisionInputFrame& input_frame,
                                        pipeline::TacticalStateStore& state_store,
                                        pipeline::TacticalEvaluationState& evaluation_state) const {
  // input_frame 在此评估器中未使用：LPI 降功率条件由上游
  // ThreatAssessmentEvaluator 写入 evaluation_state.should_reduce_power。
  // 保持窗口由 ControlReducer 统一维护，本评估器仅负责生成当周期提案。
  (void)state_store;
  bool should_reduce_power = evaluation_state.should_reduce_power;

  evaluation_state.should_reduce_power = should_reduce_power;
  if (!should_reduce_power) {
    PROJECT_LOG_INFO("[EmissionControlEvaluator] No severe threat. LPI remains inactive.");
    return;
  }

  evaluation_state.proposals.push_back(pipeline::TacticalProposal{
      BuildLpiPowerDirective(), 60, "high-confidence threat requires reduced emission"});
  PROJECT_LOG_INFO(
      "[EmissionControlEvaluator] High threat detected. Appending proposal: "
      "REQUEST_LPI_POWER_REDUCTION");
}

}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar
