/**
 * @file EccmEvaluator.h
 * @brief 定义 ECCM（电子抗干扰）评估器类。
 */

#ifndef AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_

#include <vector>

#include "airborne_radar/decision/ControlReducerTypes.h"
#include "1q/airborne_radar/session/ArInterferenceObservation.h"

namespace airborne_radar {
namespace decision {

/**
 * @brief 负责根据接收机形成的 RF 干扰观测生成 ECCM 战术提案。
 */
class EccmEvaluator final {
 public:
  EccmEvaluator() = default;

  /**
   * @brief ECCM 实际激活来源。
   */
  enum class ActivationSource {
    kNone = 0,       /**< 未激活任何 ECCM 措施。 */
    kReceiverRf      /**< 由接收机 RF 观测驱动。 */
  };

  /**
   * @brief ECCM 单周期评估结果，包含激活标志与实际激活来源。
   */
  struct Result {
    bool eccm_activated{false}; /**< 本周期是否激活了任何 ECCM 措施 */
    ActivationSource activation_source{
        ActivationSource::kNone}; /**< 本周期 ECCM 实际激活来源 */
  };

  /**
   * @brief 根据接收机形成的去真值化 RF 观测生成 ECCM 提案。
   * @param[in] observations 已通过 J/N 门限的本地观测。
   * @param[out] proposals 追加 ECCM 提案的输出列表。
   * @return 本周期 ECCM 激活状态与来源。
   */
  Result Evaluate(const session::ArInterferenceObservationList& observations,
                  std::vector<session::TacticalProposal>* proposals);

 private:
  /** @brief ECCM 提案评分中间累加状态。 */
  struct EccmProposalSelection {
    float sidelobe_canceller_score{0.0f};
    float adaptive_beamforming_score{0.0f};
    float agility_frequency_score{0.0f};
    float eccm_rejitter_score{0.0f};
    float burnthrough_gain_score{0.0f};
    float anti_rgpo_score{0.0f};
    float anti_vgpo_score{0.0f};
    float anti_false_target_score{0.0f};
  };

  /** @brief 构造生存性域控制意图。 */
  static session::ControlDirective BuildDirective(
      session::ControlDirectiveType type);

  /** @brief 根据评分增益确定优先级。 */
  static int ResolvePriorityFromScore(int base_priority, float score);

  /** @brief 将烧穿评分线性映射到 [1,2] 增益。 */
  static float ResolveBurnthroughGain(float score);

  /** @brief 累加单个接收机 RF 观测对实际 ECCM 手段的评分。 */
  static void AccumulateInterferenceObservation(
      const session::ArInterferenceObservation& observation,
      EccmProposalSelection* selection);

  /** @brief 构建提案解释文本。 */
  static std::string BuildProposalRationale(
      session::ControlDirectiveType type,
      const EccmProposalSelection& selection);

  /** @brief 向提案列表追加一条 ECCM 提案。 */
  static void AppendProposal(
      session::ControlDirectiveType type, int priority,
      const std::string& rationale,
      std::vector<session::TacticalProposal>* proposals,
      bool has_requested_value = false, float requested_value = 0.0f);
};

}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_
