#include "airborne_radar/decision/evaluators/SurvivabilityEvaluatorHelpers.h"

#include <algorithm>
#include <string>

namespace airborne_radar {
namespace decision {
namespace evaluators {
namespace internal {

namespace {

/**
 * @brief 触发频率捷变强化评分的频谱重叠分界值。
 * @note 代码行为依据：多源事实在频谱重叠比例不低于该值时，才显著提高
 *       `agility_frequency_score`。
 */
constexpr float kHighFrequencyOverlapRatio = 0.5f;
/**
 * @brief 触发重频抖动强化评分的锁定风险分界值。
 * @note 代码行为依据：多源事实在 `prf_lock_risk` 不低于该值时，才显著提高
 *       `eccm_rejitter_score`。
 */
constexpr float kHighPrfLockRisk = 0.5f;
/**
 * @brief 触发烧穿增益强化评分的干扰功率分界值。
 * @note 代码行为依据：多源事实路径会用它归一化功率权重并触发高功率门限。
 */
constexpr float kHighJammerPowerDb = 8.0f;
/**
 * @brief 触发烧穿增益强化评分的干信比分界值。
 * @note 代码行为依据：多源事实路径在 `jammer_to_signal_db` 不低于该值时，会把对应源纳入
 *       烧穿增益评分并参与权重归一化。
 */
constexpr float kHighJammerToSignalDb = 6.0f;
/**
 * @brief 多源干扰事实被视为可信证据的最小置信度。
 * @note 代码行为依据：`AccumulateMultiSourceEccmFacts()` 会直接过滤低于该值的源，
 *       使低置信度样本只能走保守回退路径，而不会触发激进的类型特定 ECCM 动作。
 */
constexpr float kMinimumCredibleConfidence = 0.35f;
/**
 * @brief 允许关联压力参与 ECCM 补位的最小干扰严重度。
 * @note 代码行为依据：只有 `jamming_severity` 不低于该值时，
 *       `HasMeaningfulAssociationPressure()` 才允许关联语义抬高 ECCM 评分。
 */
constexpr float kMinimumAssociationSeverity = 0.30f;
/**
 * @brief 允许关联压力参与 ECCM 补位的最小关联压力值。
 * @note 代码行为依据：只有 `association_stress` 不低于该值时，
 *       `HasMeaningfulAssociationPressure()` 才把关联退化视作 ECCM 的有效补充证据。
 */
constexpr float kMinimumAssociationStress = 0.18f;

/**
 * @brief 旁瓣对消提案的基础优先级。
 * @note 代码行为依据：该值作为 `ResolvePriorityFromScore()` 的基线，保证旁瓣污染明确时，
 *       旁瓣对消在当前 ECCM 动作中保持最高起始优先级。
 */
constexpr int kBasePrioritySidelobeCanceller = 86;
/**
 * @brief 自适应波束形成提案的基础优先级。
 * @note 代码行为依据：该值低于其他强对抗动作，使其既能作为保守回退动作输出，
 *       又不会轻易压过更具体的 ECCM 手段。
 */
constexpr int kBasePriorityAdaptiveBeamforming = 80;
/**
 * @brief 频率捷变提案的基础优先级。
 * @note 代码行为依据：该值位于旁瓣对消之后、重频抖动之前，用于表达频谱重叠类干扰下的
 *       中高优先级处置次序。
 */
constexpr int kBasePriorityAgilityFrequency = 84;
/**
 * @brief 重频抖动提案的基础优先级。
 * @note 代码行为依据：该值略低于频率捷变，用于在 PRF 锁定风险成立时参与排序，
 *       同时保留分数叠加后的调节空间。
 */
constexpr int kBasePriorityEccmRejitter = 83;
/**
 * @brief 烧穿增益提案的基础优先级。
 * @note 代码行为依据：该值低于频率捷变与重频抖动，使烧穿增益默认作为高功率压制下的
 *       补充动作，而不是最先执行的动作。
 */
constexpr int kBasePriorityBurnthroughGain = 82;

/**
 * @brief 输出旁瓣对消提案所需的最小累计评分。
 * @note 代码行为依据：只有明确存在旁瓣污染证据时，`sidelobe_canceller_score`
 *       才能跨过该门限，避免弱证据默认启用旁瓣对消。
 */
constexpr float kThresholdSidelobeCanceller = 1.5f;
/**
 * @brief 输出自适应波束形成提案所需的最小累计评分。
 * @note 代码行为依据：该门限低于其他 ECCM 动作，使保守回退路径的
 *       `adaptive_beamforming_score = 1.0f` 也能独立输出。
 */
constexpr float kThresholdAdaptiveBeamforming = 0.8f;
/**
 * @brief 输出频率捷变提案所需的最小累计评分。
 * @note 代码行为依据：该值要求频谱重叠或关联语义证据累计到足够强度后，
 *       才会真正下发频率捷变 proposal。
 */
constexpr float kThresholdAgilityFrequency = 1.5f;
/**
 * @brief 输出重频抖动提案所需的最小累计评分。
 * @note 代码行为依据：该值要求 PRF 锁定风险或关联语义证据累计到足够强度后，
 *       才会真正下发重频抖动 proposal。
 */
constexpr float kThresholdEccmRejitter = 1.5f;
/**
 * @brief 输出烧穿增益提案所需的最小累计评分。
 * @note 代码行为依据：该值要求高功率或高干信比证据累计到足够强度后，
 *       才会下发烧穿增益 proposal，避免弱事实直接触发功率对抗。
 */
constexpr float kThresholdBurnthroughGain = 1.5f;
/**
 * @brief 构造一条生存性域控制意图。
 * @param type 控制意图类型。
 * @return 返回 `type` 指定类型且 `source` 字段为 `SURVIVABILITY` 的控制意图。
 */
common::control::ControlDirective BuildDirective(common::control::ControlDirectiveType type) {
  return common::control::ControlDirective(type,
                                           common::control::ControlDirectiveSource::SURVIVABILITY);
}
/**
 * @brief ECCM 提案评分与关联偏置的中间累加状态。
 */
struct EccmProposalSelection {
  float sidelobe_canceller_score{0.0f};               /**< 旁瓣对消累计评分 */
  float adaptive_beamforming_score{0.0f};             /**< 自适应波束形成累计评分 */
  float agility_frequency_score{0.0f};                /**< 频率捷变累计评分 */
  float eccm_rejitter_score{0.0f};                    /**< 重频抖动累计评分 */
  float burnthrough_gain_score{0.0f};                 /**< 烧穿增益累计评分 */
  bool has_credible_multisource_evidence{false};      /**< 是否存在可信多源干扰证据 */
  bool association_supports_beam_adaptation{false};   /**< 关联压力是否支持波束自适应 */
  bool association_supports_frequency_agility{false}; /**< 关联压力是否支持频率捷变 */
  bool association_supports_rejitter{false};          /**< 关联压力是否支持重频抖动 */
  common::utils::JammingSemantic association_semantic{
      common::utils::JammingSemantic::kNone}; /**< 关联压力对应的干扰语义 */
};
/**
 * @brief 根据评分增益调整控制意图优先级。
 * @param base_priority 基础优先级。
 * @param score 当前评分。
 * @return 叠加评分后的优先级。
 */
int ResolvePriorityFromScore(int base_priority, float score) {
  return base_priority + static_cast<int>(score * 10.0f);
}
/**
 * @brief 将浮点值裁剪到 [0, 1] 区间。
 * @param value 输入值。
 * @return 裁剪后的结果。
 */
float ClampUnit(float value) { return std::max(0.0f, std::min(1.0f, value)); }

/**
 * @brief 判断关联质量是否足以作为 ECCM 补充触发证据。
 * @param association_quality_info 当前周期关联质量摘要。
 * @return 关联压力和干扰严重度同时达到最小门限时返回 true。
 */
bool HasMeaningfulAssociationPressure(
    const common::model::AssociationQualityInfo& association_quality_info) {
  return association_quality_info.association_stress >= kMinimumAssociationStress &&
         association_quality_info.jamming_severity >= kMinimumAssociationSeverity;
}
/**
 * @brief 为缺少细粒度事实的场景添加保守自适应波束形成偏置。
 * @param selection 待累加的提案选择状态。
 */
void AccumulateCautiousFallback(EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  selection->adaptive_beamforming_score = std::max(selection->adaptive_beamforming_score, 1.0f);
}
/**
 * @brief 把单个可信多源干扰事实累加为 ECCM 动作评分。
 * @param source 单个干扰源事实。
 * @param selection 待累加的提案选择状态。
 */
void AccumulateMultiSourceEccmFacts(const common::model::EccmJammerSourceInfo& source,
                                    EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  if (source.confidence < kMinimumCredibleConfidence - 1e-5f) {
    return;
  }

  selection->has_credible_multisource_evidence = true;
  const float confidence_weight = ClampUnit(source.confidence);
  const float power_weight =
      std::max(0.0f, std::min(source.jammer_power_db / kHighJammerPowerDb, 2.0f));
  const float js_weight =
      std::max(0.0f, std::min(source.jammer_to_signal_db / kHighJammerToSignalDb, 2.0f));

  selection->adaptive_beamforming_score += 0.8f * confidence_weight;
  if (source.jammer_in_sidelobe) {
    selection->sidelobe_canceller_score += 2.0f * confidence_weight;
  }
  if (source.frequency_overlap_ratio >= kHighFrequencyOverlapRatio - 1e-5f) {
    selection->agility_frequency_score += source.frequency_overlap_ratio * 2.0f * confidence_weight;
  }
  if (source.prf_lock_risk >= kHighPrfLockRisk - 1e-5f) {
    selection->eccm_rejitter_score += source.prf_lock_risk * 2.0f * confidence_weight;
  }
  if (source.jammer_power_db >= kHighJammerPowerDb - 1e-5f ||
      source.jammer_to_signal_db >= kHighJammerToSignalDb - 1e-5f) {
    selection->burnthrough_gain_score += std::max(power_weight, js_weight) * confidence_weight;
  }

  switch (source.technique) {
    case common::model::JammingTechnique::kNoiseSuppression:
      selection->adaptive_beamforming_score += 0.7f * confidence_weight;
      selection->burnthrough_gain_score += 0.8f * confidence_weight;
      if (source.jammer_in_sidelobe) {
        selection->sidelobe_canceller_score += 1.2f * confidence_weight;
      }
      break;
    case common::model::JammingTechnique::kDeception:
      selection->agility_frequency_score += 1.5f * confidence_weight;
      selection->eccm_rejitter_score += 1.8f * confidence_weight;
      selection->adaptive_beamforming_score += 0.5f * confidence_weight;
      break;
    case common::model::JammingTechnique::kRepeater:
      selection->eccm_rejitter_score += 1.6f * confidence_weight;
      selection->adaptive_beamforming_score += 1.0f * confidence_weight;
      selection->agility_frequency_score += 0.6f * confidence_weight;
      break;
    case common::model::JammingTechnique::kUnknown:
    default:
      selection->adaptive_beamforming_score += 0.4f * confidence_weight;
      break;
  }
}

/**
 * @brief 根据关联压力与语义信息补充 ECCM 动作偏置。
 * @param association_quality_info 当前周期关联质量摘要。
 * @param selection 待累加的提案选择状态。
 */
void AccumulateAssociationDrivenBias(
    const common::model::AssociationQualityInfo& association_quality_info,
    EccmProposalSelection* selection) {
  if (selection == nullptr || !HasMeaningfulAssociationPressure(association_quality_info)) {
    return;
  }

  const float severity = ClampUnit(association_quality_info.jamming_severity);
  const float stress = ClampUnit(association_quality_info.association_stress);
  const float cost_pressure = ClampUnit(std::max(association_quality_info.mean_match_cost / 3.0f,
                                                 association_quality_info.p95_match_cost / 4.0f));
  const float combined_weight = 0.45f * severity + 0.40f * stress + 0.15f * cost_pressure;

  switch (association_quality_info.dominant_jamming_semantic) {
    case common::utils::JammingSemantic::kDeception:
      selection->association_semantic = common::utils::JammingSemantic::kDeception;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 3.5f * combined_weight;
      selection->eccm_rejitter_score += 3.8f * combined_weight;
      selection->adaptive_beamforming_score += 0.8f * combined_weight;
      break;
    case common::utils::JammingSemantic::kRepeater:
      selection->association_semantic = common::utils::JammingSemantic::kRepeater;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 1.8f * combined_weight;
      selection->eccm_rejitter_score += 3.6f * combined_weight;
      selection->adaptive_beamforming_score += 0.9f * combined_weight;
      break;
    case common::utils::JammingSemantic::kMixed:
      selection->association_semantic = common::utils::JammingSemantic::kMixed;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 2.8f * combined_weight;
      selection->eccm_rejitter_score += 3.0f * combined_weight;
      selection->adaptive_beamforming_score += 0.8f * combined_weight;
      break;
    case common::utils::JammingSemantic::kNoiseSuppression:
    case common::utils::JammingSemantic::kNone:
    default:
      break;
  }
}
/**
 * @brief 将关联语义转换为可读描述文本。
 * @param semantic 当前周期主导干扰语义。
 * @return 用于 rationale 的简短描述。
 */
std::string DescribeAssociationSemantic(common::utils::JammingSemantic semantic) {
  switch (semantic) {
    case common::utils::JammingSemantic::kDeception:
      return "deception-like association stress";
    case common::utils::JammingSemantic::kRepeater:
      return "repeater-like association stress";
    case common::utils::JammingSemantic::kMixed:
      return "mixed association stress";
    case common::utils::JammingSemantic::kNoiseSuppression:
      return "noise-like association stress";
    case common::utils::JammingSemantic::kNone:
    default:
      return "association stress";
  }
}

/**
 * @brief 生成指定 ECCM proposal 的解释文本。
 * @param type 控制意图类型。
 * @param selection 当前提案选择状态。
 * @return 对应 proposal 的 rationale 文本。
 */
std::string BuildProposalRationale(common::control::ControlDirectiveType type,
                                   const EccmProposalSelection& selection) {
  std::string rationale;
  switch (type) {
    case common::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      rationale = "jamming facts favor sidelobe cancellation";
      break;
    case common::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      rationale = "jamming environment requires adaptive beamforming";
      if (selection.association_supports_beam_adaptation) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " also degrades association stability";
      }
      break;
    case common::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      rationale = "jamming facts favor agility frequency";
      if (selection.association_supports_frequency_agility) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " raises frequency-agility priority";
      }
      break;
    case common::control::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      rationale = "jamming facts favor rejitter";
      if (selection.association_supports_rejitter) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " raises PRF rejitter priority";
      }
      break;
    case common::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      rationale = "jamming facts favor burnthrough gain";
      break;
    default:
      rationale = "jamming facts require countermeasure";
      break;
  }
  return rationale;
}

/**
 * @brief 向提案列表追加一条 ECCM 控制意图。
 * @param type 控制意图类型。
 * @param priority 提案优先级。
 * @param rationale 提案解释文本。
 * @param proposals 提案输出列表。
 */
void AppendProposal(common::control::ControlDirectiveType type, int priority,
                    const std::string& rationale,
                    std::vector<pipeline::TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return;
  }
  proposals->push_back(pipeline::TacticalProposal{BuildDirective(type), priority, rationale});
}

}  // namespace

void AppendEccmProposals(const common::model::EccmSourceInfo& source_info,
                         const common::model::AssociationQualityInfo& association_quality_info,
                         bool environment_jamming_detected, bool hold_only,
                         std::vector<pipeline::TacticalProposal>* proposals) {
  (void)environment_jamming_detected;
  EccmProposalSelection selection;
  if (hold_only) {
    // 持有期路径：跳过多源事实积累和关联偏置，仅保留保守波束形成保底分数，
    // 确保持有期提案权重低于新鲜证据触发的提案。
    AccumulateCautiousFallback(&selection);
  } else {
    if (!source_info.jammer_sources.empty()) {
      for (std::size_t i = 0; i < source_info.jammer_sources.size(); ++i) {
        AccumulateMultiSourceEccmFacts(source_info.jammer_sources[i], &selection);
      }
      if (!selection.has_credible_multisource_evidence) {
        AccumulateCautiousFallback(&selection);
      }
    } else {
      AccumulateCautiousFallback(&selection);
    }
    AccumulateAssociationDrivenBias(association_quality_info, &selection);
  }

  if (selection.sidelobe_canceller_score >= kThresholdSidelobeCanceller) {
    AppendProposal(
        common::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
        ResolvePriorityFromScore(kBasePrioritySidelobeCanceller,
                                 selection.sidelobe_canceller_score),
        BuildProposalRationale(
            common::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER, selection),
        proposals);
  }
  if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
    AppendProposal(
        common::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
        ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                 selection.adaptive_beamforming_score),
        BuildProposalRationale(
            common::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING, selection),
        proposals);
  }
  if (selection.agility_frequency_score >= kThresholdAgilityFrequency) {
    AppendProposal(
        common::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
        ResolvePriorityFromScore(kBasePriorityAgilityFrequency, selection.agility_frequency_score),
        BuildProposalRationale(common::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                               selection),
        proposals);
  }
  if (selection.eccm_rejitter_score >= kThresholdEccmRejitter) {
    AppendProposal(
        common::control::ControlDirectiveType::REQUEST_ECCM_REJITTER,
        ResolvePriorityFromScore(kBasePriorityEccmRejitter, selection.eccm_rejitter_score),
        BuildProposalRationale(common::control::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                               selection),
        proposals);
  }
  if (selection.burnthrough_gain_score >= kThresholdBurnthroughGain) {
    AppendProposal(
        common::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
        ResolvePriorityFromScore(kBasePriorityBurnthroughGain, selection.burnthrough_gain_score),
        BuildProposalRationale(common::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                               selection),
        proposals);
  }
}

}  // namespace internal
}  // namespace evaluators
}  // namespace decision
}  // namespace airborne_radar
