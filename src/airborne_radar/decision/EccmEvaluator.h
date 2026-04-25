/**
 * @file EccmEvaluator.h
 * @brief 定义 ECCM（电子抗干扰）评估器类。
 */

#ifndef AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_

#include <vector>

#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/model/DecisionSourceInfo.h"

namespace airborne_radar {
namespace decision {

/**
 * @brief 负责根据环境干扰事实生成 ECCM 战术提案。
 *
 * 输入全部来源于环境层（EccmSourceInfo），不接收关联质量等非环境层数据。
 * 基于干扰源事实（功率、频率重叠度、PRF 锁风险、技术类型、来向等）
 * 对五种 ECCM 手段分别评分，评分跨过阈值的措施生成 TacticalProposal。
 */
class EccmEvaluator final {
 public:
  EccmEvaluator() = default;

  /**
   * @brief ECCM 评估结果。
   */
  struct Result {
    bool eccm_activated{false}; /**< 本周期是否激活了任何 ECCM 措施 */
  };

  /**
   * @brief 基于环境干扰事实生成 ECCM 战术提案。
   * @param eccm_source_info  环境干扰来源信息（has_jamming_signal + jammer_sources）
   * @param hold_only         true 表示仅输出保守保底提案（持有期路径）
   * @param proposals         [out] 追加 ECCM 提案
   * @return 本评估周期的 ECCM 评估结果
   */
  Result Evaluate(const model::EccmSourceInfo& eccm_source_info,
                  bool hold_only,
                  std::vector<extension::TacticalProposal>* proposals);

 private:
  /** @brief ECCM 提案评分中间累加状态。 */
  struct EccmProposalSelection {
    float sidelobe_canceller_score{0.0f};
    float adaptive_beamforming_score{0.0f};
    float agility_frequency_score{0.0f};
    float eccm_rejitter_score{0.0f};
    float burnthrough_gain_score{0.0f};
    bool has_credible_multisource_evidence{false};
  };

  /** @brief 构造生存性域控制意图。 */
  static extension::control::ControlDirective BuildDirective(
      extension::control::ControlDirectiveType type);

  /** @brief 根据评分增益确定优先级。 */
  static int ResolvePriorityFromScore(int base_priority, float score);

  /** @brief 将浮点值裁剪到 [0, 1]。 */
  static float ClampUnit(float value);

  /** @brief 判断关联是否存在显著压力（供外部后处理使用）。 */
  static bool HasMeaningfulAssociationPressure(
      float jamming_severity, float association_stress);

  /** @brief 添加保守自适应波束形成保底分。 */
  static void AccumulateCautiousFallback(EccmProposalSelection* selection);

  /** @brief 累加单个可信干扰源事实对五种 ECCM 措施的评分。 */
  static void AccumulateMultiSourceEccmFacts(
      const model::EccmJammerSourceInfo& source,
      EccmProposalSelection* selection);

  /** @brief 构建提案解释文本。 */
  static std::string BuildProposalRationale(
      extension::control::ControlDirectiveType type,
      const EccmProposalSelection& selection);

  /** @brief 向提案列表追加一条 ECCM 提案。 */
  static void AppendProposal(
      extension::control::ControlDirectiveType type, int priority,
      const std::string& rationale,
      std::vector<extension::TacticalProposal>* proposals);
};

}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_
