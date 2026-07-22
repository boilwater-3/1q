/**
 * @file EccmEvaluator.h
 * @brief 定义 ECCM（电子抗干扰）评估器类。
 */

#ifndef AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_ECCM_EVALUATOR_H_

#include <vector>

#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/DecisionSourceInfo.h"

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
   * @brief ECCM 实际激活来源。
   *
   * @note 该枚举反映「本周期真正达标的 proposal 来自哪条路径」，即**实际激活来源**，
   *       而非「证据本身的有无」。例如：即使存在 credible 多源干扰证据
   *       (`has_credible_multisource_evidence=true`)，但若五项评分全部未跨过阈值，
   *       控制流会进入末尾「最低保底」分支强制激活自适应波束形成——此时实际达标的
   *       是保底 proposal，故 `activation_source` 标记为 `kCautiousFallback`，
   *       而非 `kEvidenceBased`。消费方若需要区分「有证据但不足以触发具体措施」与
   *       「完全无证据」，应结合 proposals 列表内容判断。
   */
  enum class ActivationSource {
    kNone = 0,             /**< 未激活任何 ECCM 措施 */
    kEvidenceBased,        /**< 由可信干扰源事实直接驱动评分达标而激活 */
    kCautiousFallback,     /**< 由保守保底路径激活（含「有证据但评分不达标→走保底」） */
    kAssociationPressure   /**< 由关联质量压力驱动评分达标而激活 */
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

  /**
   * @brief 基于环境干扰事实生成 ECCM 战术提案（基础重载）。
   *
   * @deprecated 生产路径已迁移到带 `AssociationQualityInfo` 的关联重载：
   *   `Evaluate(eccm_source_info, association_quality, hold_only, proposals)`。
   *   `TacticalCoordinator` 当前仅调用关联重载；本基础重载保留以兼容历史单元测试，
   *   新代码请勿使用。
   *
   * @param[in] eccm_source_info  环境干扰来源信息（has_jamming_signal + jammer_sources）
   * @param[in] hold_only         true 表示仅输出保守保底提案（持有期路径）
   * @param[out] proposals        追加 ECCM 提案的输出列表
   * @return 本评估周期的 ECCM 评估结果
   */
  [[deprecated("生产路径已迁移到带 AssociationQualityInfo 的关联重载；"
               "新代码请使用 Evaluate(eccm_source_info, association_quality, hold_only, proposals)")]]
  Result Evaluate(const session::EccmSourceInfo& eccm_source_info,
                  bool hold_only,
                  std::vector<session::TacticalProposal>* proposals);

  /**
   * @brief 基于关联质量压力生成 ECCM 战术提案（无实际干扰源时的路径）。
   *
   * 当环境未上报可信干扰源，但关联质量异常显示存在等效干扰压力时，
   * 直接根据干扰语义与强度推算 ECCM 评分，而非通过合成物理干扰源绕行。
   *
   * @param[in] eccm_source_info   环境干扰来源（has_jamming_signal 已置位，jammer_sources 可为空）
   * @param[in] association_quality 关联质量摘要，包含语义类型和强度
   * @param[in] hold_only           true 表示仅输出保守保底提案（持有期路径）
   * @param[out] proposals           追加 ECCM 提案的输出列表
   * @return 本评估周期的 ECCM 评估结果
   */
  Result Evaluate(const session::EccmSourceInfo& eccm_source_info,
                  const session::AssociationQualityInfo& association_quality,
                  bool hold_only,
                  std::vector<session::TacticalProposal>* proposals);

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
  static session::ControlDirective BuildDirective(
      session::ControlDirectiveType type);

  /** @brief 根据评分增益确定优先级。 */
  static int ResolvePriorityFromScore(int base_priority, float score);

  /** @brief 将烧穿评分线性映射到 [1,2] 增益。 */
  static float ResolveBurnthroughGain(float score);

  /** @brief 将浮点值裁剪到 [0, 1]。 */
  static float ClampUnit(float value);

  /** @brief 判断关联是否存在显著压力（供外部后处理使用）。 */
  static bool HasMeaningfulAssociationPressure(
      float jamming_severity, float association_stress);

  /** @brief 添加保守自适应波束形成保底分。 */
  static void AccumulateCautiousFallback(EccmProposalSelection* selection);

  /**
   * @brief 基于关联质量压力直接累加 ECCM 评分（等效于物理参数在阈值处的合成干扰源）。
   *
   * 避免向 EccmEvaluator 喂入伪造的物理参数：
   * 按干扰语义选择技术特化评分项，以 jamming_severity 作为置信度尺度。
   */
  static void AccumulateAssociationPressureFacts(
      const session::AssociationQualityInfo& association_quality,
      EccmProposalSelection* selection);

  /** @brief 累加单个可信干扰源事实对五种 ECCM 措施的评分。 */
  static void AccumulateMultiSourceEccmFacts(
      const session::EccmJammerSourceInfo& source,
      EccmProposalSelection* selection);

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
