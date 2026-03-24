#include "airborne_radar/decision/eccm/SurvivabilityEvaluator.h"

#include <algorithm>
#include <string>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {
namespace eccm {

namespace {

/**
 * @brief ECCM 提案保持周期计数。
 * @note 代码行为依据：只要当前周期仍判定需要启用 ECCM，`Evaluate()` 就会把
 *       `state_store.eccm_hold_cycles_remaining` 设回该值，用于延续保护发射路径。
 */
const std::uint32_t kEccmHoldCycles = 2;
/**
 * @brief 触发频率捷变强化评分的频谱重叠分界值。
 * @note 代码行为依据：旧版事实与多源事实都在频谱重叠比例不低于该值时，才显著提高
 *       `agility_frequency_score`。
 */
const float kHighFrequencyOverlapRatio = 0.5f;
/**
 * @brief 触发重频抖动强化评分的锁定风险分界值。
 * @note 代码行为依据：旧版事实与多源事实都在 `prf_lock_risk` 不低于该值时，才显著提高
 *       `eccm_rejitter_score`。
 */
const float kHighPrfLockRisk = 0.5f;
/**
 * @brief 触发烧穿增益强化评分的干扰功率分界值。
 * @note 代码行为依据：旧版事实路径把该值作为 `burnthrough_gain_score` 的高功率门限，
 *       多源事实路径还会用它归一化功率权重。
 */
const float kHighJammerPowerDb = 8.0f;
/**
 * @brief 触发烧穿增益强化评分的干信比分界值。
 * @note 代码行为依据：多源事实路径在 `jammer_to_signal_db` 不低于该值时，会把对应源纳入
 *       烧穿增益评分并参与权重归一化。
 */
const float kHighJammerToSignalDb = 6.0f;
/**
 * @brief 多源干扰事实被视为可信证据的最小置信度。
 * @note 代码行为依据：`AccumulateMultiSourceEccmFacts()` 会直接过滤低于该值的源，
 *       使低置信度样本只能走保守回退路径，而不会触发激进的类型特定 ECCM 动作。
 */
const float kMinimumCredibleConfidence = 0.35f;
/**
 * @brief 允许关联压力参与 ECCM 补位的最小干扰严重度。
 * @note 代码行为依据：只有 `jamming_severity` 不低于该值时，
 *       `HasMeaningfulAssociationPressure()` 才允许关联语义抬高 ECCM 评分。
 */
const float kMinimumAssociationSeverity = 0.30f;
/**
 * @brief 允许关联压力参与 ECCM 补位的最小关联压力值。
 * @note 代码行为依据：只有 `association_stress` 不低于该值时，
 *       `HasMeaningfulAssociationPressure()` 才把关联退化视作 ECCM 的有效补充证据。
 */
const float kMinimumAssociationStress = 0.18f;

/**
 * @brief 旁瓣对消提案的基础优先级。
 * @note 代码行为依据：该值作为 `ResolvePriorityFromScore()` 的基线，保证旁瓣污染明确时，
 *       旁瓣对消在当前 ECCM 动作中保持最高起始优先级。
 */
const int kBasePrioritySidelobeCanceller = 86;
/**
 * @brief 自适应波束形成提案的基础优先级。
 * @note 代码行为依据：该值低于其他强对抗动作，使其既能作为保守回退动作输出，
 *       又不会轻易压过更具体的 ECCM 手段。
 */
const int kBasePriorityAdaptiveBeamforming = 80;
/**
 * @brief 频率捷变提案的基础优先级。
 * @note 代码行为依据：该值位于旁瓣对消之后、重频抖动之前，用于表达频谱重叠类干扰下的
 *       中高优先级处置次序。
 */
const int kBasePriorityAgilityFrequency = 84;
/**
 * @brief 重频抖动提案的基础优先级。
 * @note 代码行为依据：该值略低于频率捷变，用于在 PRF 锁定风险成立时参与排序，
 *       同时保留分数叠加后的调节空间。
 */
const int kBasePriorityEccmRejitter = 83;
/**
 * @brief 烧穿增益提案的基础优先级。
 * @note 代码行为依据：该值低于频率捷变与重频抖动，使烧穿增益默认作为高功率压制下的
 *       补充动作，而不是最先执行的动作。
 */
const int kBasePriorityBurnthroughGain = 82;

/**
 * @brief 输出旁瓣对消提案所需的最小累计评分。
 * @note 代码行为依据：只有明确存在旁瓣污染证据时，`sidelobe_canceller_score`
 *       才能跨过该门限，避免弱证据默认启用旁瓣对消。
 */
const float kThresholdSidelobeCanceller = 1.5f;
/**
 * @brief 输出自适应波束形成提案所需的最小累计评分。
 * @note 代码行为依据：该门限低于其他 ECCM 动作，使保守回退路径的
 *       `adaptive_beamforming_score = 1.0f` 也能独立输出。
 */
const float kThresholdAdaptiveBeamforming = 0.8f;
/**
 * @brief 输出频率捷变提案所需的最小累计评分。
 * @note 代码行为依据：该值要求频谱重叠或关联语义证据累计到足够强度后，
 *       才会真正下发频率捷变 proposal。
 */
const float kThresholdAgilityFrequency = 1.5f;
/**
 * @brief 输出重频抖动提案所需的最小累计评分。
 * @note 代码行为依据：该值要求 PRF 锁定风险或关联语义证据累计到足够强度后，
 *       才会真正下发重频抖动 proposal。
 */
const float kThresholdEccmRejitter = 1.5f;
/**
 * @brief 输出烧穿增益提案所需的最小累计评分。
 * @note 代码行为依据：该值要求高功率或高干信比证据累计到足够强度后，
 *       才会下发烧穿增益 proposal，避免弱事实直接触发功率对抗。
 */
const float kThresholdBurnthroughGain = 1.5f;
/**
 * @brief 构造一条生存性域控制意图。
 * @param type 控制意图类型。
 * @return 返回 `type` 指定类型且 `source` 字段为 `SURVIVABILITY` 的控制意图。
 */
common::ControlDirective BuildDirective(common::ControlDirectiveType type) {
  return common::ControlDirective(type,
                                  common::ControlDirectiveSource::SURVIVABILITY);
}
/**
 * @brief 判断 ECCM 输入是否携带可用的细粒度干扰事实。
 * @param source_info 当前周期 ECCM 输入摘要。
 * @return 至少存在多源事实或兼容字段时返回 true。
 */
bool HasDetailedEccmFacts(const common::EccmSourceInfo& source_info) {
  if (!source_info.jammer_sources.empty()) {
    return true;
  }
  return source_info.jammer_power_db > 0.0f ||
         source_info.frequency_overlap_ratio > 0.0f ||
         source_info.prf_lock_risk > 0.0f || source_info.jammer_in_sidelobe;
}

struct EccmProposalSelection {
  float sidelobe_canceller_score{0.0f};
  float adaptive_beamforming_score{0.0f};
  float agility_frequency_score{0.0f};
  float eccm_rejitter_score{0.0f};
  float burnthrough_gain_score{0.0f};
  bool has_credible_multisource_evidence{false};
  bool association_supports_beam_adaptation{false};
  bool association_supports_frequency_agility{false};
  bool association_supports_rejitter{false};
  common::JammingSemantic association_semantic{common::JammingSemantic::kNone};
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
float ClampUnit(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

/**
 * @brief 判断关联质量是否足以作为 ECCM 补充触发证据。
 * @param association_quality_info 当前周期关联质量摘要。
 * @return 关联压力和干扰严重度同时达到最小门限时返回 true。
 */
bool HasMeaningfulAssociationPressure(
    const common::AssociationQualityInfo& association_quality_info) {
  return association_quality_info.association_stress >=
             kMinimumAssociationStress &&
         association_quality_info.jamming_severity >=
             kMinimumAssociationSeverity;
}
/**
 * @brief 为缺少细粒度事实的场景添加保守自适应波束形成偏置。
 * @param selection 待累加的提案选择状态。
 */
void AccumulateCautiousFallback(EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  selection->adaptive_beamforming_score =
      std::max(selection->adaptive_beamforming_score, 1.0f);
}
/**
 * @brief 将兼容旧版平铺 ECCM 字段累加为提案评分。
 * @param source_info 当前周期 ECCM 输入摘要。
 * @param selection 待累加的提案选择状态。
 */
void AccumulateLegacyEccmFacts(const common::EccmSourceInfo& source_info,
                               EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  const bool has_detailed_facts = HasDetailedEccmFacts(source_info);
  if (!has_detailed_facts || source_info.jammer_in_sidelobe) {
    selection->sidelobe_canceller_score =
        std::max(selection->sidelobe_canceller_score, 2.0f);
  }
  selection->adaptive_beamforming_score =
      std::max(selection->adaptive_beamforming_score, 1.5f);
  if (!has_detailed_facts ||
      source_info.frequency_overlap_ratio >= kHighFrequencyOverlapRatio) {
    selection->agility_frequency_score =
        std::max(selection->agility_frequency_score, 2.0f);
  }
  if (!has_detailed_facts || source_info.prf_lock_risk >= kHighPrfLockRisk) {
    selection->eccm_rejitter_score =
        std::max(selection->eccm_rejitter_score, 2.0f);
  }
  if (!has_detailed_facts || source_info.jammer_power_db >= kHighJammerPowerDb) {
    selection->burnthrough_gain_score =
        std::max(selection->burnthrough_gain_score, 2.0f);
  }
}

/**
 * @brief 把单个可信多源干扰事实累加为 ECCM 动作评分。
 * @param source 单个干扰源事实。
 * @param selection 待累加的提案选择状态。
 */
void AccumulateMultiSourceEccmFacts(
    const common::EccmJammerSourceInfo& source,
    EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  if (source.confidence < kMinimumCredibleConfidence) {
    return;
  }

  selection->has_credible_multisource_evidence = true;
  const float confidence_weight = std::max(source.confidence, 0.5f);
  const float power_weight =
      std::min(source.jammer_power_db / kHighJammerPowerDb, 2.0f);
  const float js_weight =
      std::min(source.jammer_to_signal_db / kHighJammerToSignalDb, 2.0f);

  selection->adaptive_beamforming_score += 0.8f * confidence_weight;
  if (source.jammer_in_sidelobe) {
    selection->sidelobe_canceller_score += 2.0f * confidence_weight;
  }
  if (source.frequency_overlap_ratio >= kHighFrequencyOverlapRatio) {
    selection->agility_frequency_score +=
        source.frequency_overlap_ratio * 2.0f * confidence_weight;
  }
  if (source.prf_lock_risk >= kHighPrfLockRisk) {
    selection->eccm_rejitter_score +=
        source.prf_lock_risk * 2.0f * confidence_weight;
  }
  if (source.jammer_power_db >= kHighJammerPowerDb ||
      source.jammer_to_signal_db >= kHighJammerToSignalDb) {
    selection->burnthrough_gain_score +=
        std::max(power_weight, js_weight) * confidence_weight;
  }

  switch (source.technique) {
    case common::JammingTechnique::kNoiseSuppression:
      selection->adaptive_beamforming_score += 0.7f * confidence_weight;
      selection->burnthrough_gain_score += 0.8f * confidence_weight;
      if (source.jammer_in_sidelobe) {
        selection->sidelobe_canceller_score += 1.2f * confidence_weight;
      }
      break;
    case common::JammingTechnique::kDeception:
      selection->agility_frequency_score += 1.5f * confidence_weight;
      selection->eccm_rejitter_score += 1.8f * confidence_weight;
      selection->adaptive_beamforming_score += 0.5f * confidence_weight;
      break;
    case common::JammingTechnique::kRepeater:
      selection->eccm_rejitter_score += 1.6f * confidence_weight;
      selection->adaptive_beamforming_score += 1.0f * confidence_weight;
      selection->agility_frequency_score += 0.6f * confidence_weight;
      break;
    case common::JammingTechnique::kUnknown:
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
    const common::AssociationQualityInfo& association_quality_info,
    EccmProposalSelection* selection) {
  if (selection == nullptr ||
      !HasMeaningfulAssociationPressure(association_quality_info)) {
    return;
  }

  const float severity = ClampUnit(association_quality_info.jamming_severity);
  const float stress = ClampUnit(association_quality_info.association_stress);
  const float cost_pressure =
      ClampUnit(std::max(association_quality_info.mean_match_cost / 3.0f,
                         association_quality_info.p95_match_cost / 4.0f));
  const float combined_weight =
      0.45f * severity + 0.40f * stress + 0.15f * cost_pressure;

  switch (association_quality_info.dominant_jamming_semantic) {
    case common::JammingSemantic::kDeception:
      selection->association_semantic = common::JammingSemantic::kDeception;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 3.5f * combined_weight;
      selection->eccm_rejitter_score += 3.8f * combined_weight;
      selection->adaptive_beamforming_score += 0.8f * combined_weight;
      break;
    case common::JammingSemantic::kRepeater:
      selection->association_semantic = common::JammingSemantic::kRepeater;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 1.8f * combined_weight;
      selection->eccm_rejitter_score += 3.6f * combined_weight;
      selection->adaptive_beamforming_score += 0.9f * combined_weight;
      break;
    case common::JammingSemantic::kMixed:
      selection->association_semantic = common::JammingSemantic::kMixed;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 2.8f * combined_weight;
      selection->eccm_rejitter_score += 3.0f * combined_weight;
      selection->adaptive_beamforming_score += 0.8f * combined_weight;
      break;
    case common::JammingSemantic::kNoiseSuppression:
    case common::JammingSemantic::kNone:
    default:
      break;
  }
}
/**
 * @brief 将关联语义转换为可读描述文本。
 * @param semantic 当前周期主导干扰语义。
 * @return 用于 rationale 的简短描述。
 */
std::string DescribeAssociationSemantic(common::JammingSemantic semantic) {
  switch (semantic) {
    case common::JammingSemantic::kDeception:
      return "deception-like association stress";
    case common::JammingSemantic::kRepeater:
      return "repeater-like association stress";
    case common::JammingSemantic::kMixed:
      return "mixed association stress";
    case common::JammingSemantic::kNoiseSuppression:
      return "noise-like association stress";
    case common::JammingSemantic::kNone:
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
std::string BuildProposalRationale(
    common::ControlDirectiveType type,
    const EccmProposalSelection& selection) {
  std::string rationale;
  switch (type) {
    case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      rationale = "jamming facts favor sidelobe cancellation";
      break;
    case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      rationale = "jamming environment requires adaptive beamforming";
      if (selection.association_supports_beam_adaptation) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " also degrades association stability";
      }
      break;
    case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      rationale = "jamming facts favor agility frequency";
      if (selection.association_supports_frequency_agility) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " raises frequency-agility priority";
      }
      break;
    case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      rationale = "jamming facts favor rejitter";
      if (selection.association_supports_rejitter) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " raises PRF rejitter priority";
      }
      break;
    case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
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
void AppendProposal(common::ControlDirectiveType type, int priority,
                    const std::string& rationale,
                    std::vector<pipeline::TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return;
  }
  proposals->push_back(
      pipeline::TacticalProposal{BuildDirective(type), priority, rationale});
}

/**
 * @brief 根据干扰事实与关联质量生成当前周期的 ECCM proposal 集合。
 * @param source_info 当前周期 ECCM 输入摘要。
 * @param association_quality_info 当前周期关联质量摘要。
 * @param environment_jamming_detected 环境层是否直接报告干扰存在。
 * @param hold_only 当前周期是否仅由 hold 机制维持 ECCM 输出。
 * @param proposals 提案输出列表。
 */
void AppendEccmProposals(const common::EccmSourceInfo& source_info,
                         const common::AssociationQualityInfo& association_quality_info,
                         bool environment_jamming_detected,
                         bool hold_only,
                         std::vector<pipeline::TacticalProposal>* proposals) {
  EccmProposalSelection selection;
  if (!source_info.jammer_sources.empty()) {
    for (std::size_t i = 0; i < source_info.jammer_sources.size(); ++i) {
      AccumulateMultiSourceEccmFacts(source_info.jammer_sources[i], &selection);
    }
    if (!selection.has_credible_multisource_evidence) {
      AccumulateCautiousFallback(&selection);
    }
  } else {
    const bool has_detailed_facts = HasDetailedEccmFacts(source_info);
    const bool should_use_cautious_fallback =
        !has_detailed_facts && !environment_jamming_detected &&
        (HasMeaningfulAssociationPressure(association_quality_info) ||
         hold_only);
    if (should_use_cautious_fallback) {
      AccumulateCautiousFallback(&selection);
    } else {
      AccumulateLegacyEccmFacts(source_info, &selection);
    }
  }
  AccumulateAssociationDrivenBias(association_quality_info, &selection);

  if (selection.sidelobe_canceller_score >= kThresholdSidelobeCanceller) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
        ResolvePriorityFromScore(kBasePrioritySidelobeCanceller,
                                 selection.sidelobe_canceller_score),
        BuildProposalRationale(
            common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
            selection),
        proposals);
  }
  if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
        ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                 selection.adaptive_beamforming_score),
        BuildProposalRationale(
            common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
            selection),
        proposals);
  }
  if (selection.agility_frequency_score >= kThresholdAgilityFrequency) {
    AppendProposal(common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                   ResolvePriorityFromScore(kBasePriorityAgilityFrequency,
                                            selection.agility_frequency_score),
                   BuildProposalRationale(
                       common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                       selection),
                   proposals);
  }
  if (selection.eccm_rejitter_score >= kThresholdEccmRejitter) {
    AppendProposal(common::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                   ResolvePriorityFromScore(kBasePriorityEccmRejitter,
                                            selection.eccm_rejitter_score),
                   BuildProposalRationale(
                       common::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                       selection),
                   proposals);
  }
  if (selection.burnthrough_gain_score >= kThresholdBurnthroughGain) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
        ResolvePriorityFromScore(kBasePriorityBurnthroughGain,
                                 selection.burnthrough_gain_score),
        BuildProposalRationale(
            common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
            selection),
        proposals);
  }
}

} // namespace

void SurvivabilityEvaluator::Evaluate(
    const common::DecisionInputFrame& input_frame, pipeline::TacticalStateStore& state_store,
    pipeline::TacticalEvaluationState& evaluation_state) const {
  bool should_enable_eccm =
      evaluation_state.eccm_source_info.has_jamming_signal ||
      input_frame.environment_jamming_detected;
  const bool has_current_eccm_evidence = should_enable_eccm;
  if (!should_enable_eccm && state_store.eccm_hold_cycles_remaining > 0U) {
    should_enable_eccm = true;
    --state_store.eccm_hold_cycles_remaining;
  }

  evaluation_state.should_enable_eccm = should_enable_eccm;
  if (!should_enable_eccm) {
    PROJECT_LOG_INFO(
        "[SurvivabilityEvaluator] Environment is clear. Continuing nominal operation.");
    return;
  }

  state_store.eccm_hold_cycles_remaining = kEccmHoldCycles;
  AppendEccmProposals(evaluation_state.eccm_source_info,
                      input_frame.association_quality_info,
                      input_frame.environment_jamming_detected,
                      !has_current_eccm_evidence,
                      &evaluation_state.proposals);
  PROJECT_LOG_INFO(
      "[SurvivabilityEvaluator] Active jamming detected. Appending ECCM proposals.");
}

} // namespace eccm
} // namespace decision
} // namespace airborne_radar
