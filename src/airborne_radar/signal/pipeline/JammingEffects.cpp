#include "airborne_radar/signal/pipeline/JammingEffects.h"

#include <algorithm>
#include <cmath>

#include "airborne_radar/common/utils/MathUtils.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

using common::utils::ClampFloat;
using common::utils::DbToLinearPower;

// ---------- A. ECCM 物理抑制系数（不开放配置，属于雷达对抗物理模型） ----------
constexpr float kSidelobeCancellerResidual = 0.55f;    /**< 旁瓣对消主链残余因子 */
constexpr float kAdaptiveBeamNarrowResidual = 0.72f;   /**< 自适应波束成形（窄角干扰）残余因子 */
constexpr float kAdaptiveBeamWideResidual = 0.82f;     /**< 自适应波束成形（宽角干扰）残余因子 */
constexpr float kAgilityFreqResidualMin = 0.35f;       /**< 频率捷变残余因子下界 */
constexpr float kEccmRejitterResidualMin = 0.30f;      /**< ECCM 抖动残余因子下界 */
constexpr float kBurnthroughResidualMin = 0.55f;       /**< 穿透增益残余因子下界 */
constexpr float kResidualFactorMin = 0.15f;            /**< 总残余因子下界 */
constexpr float kNoiseSidelobeCancellerFactor = 0.75f; /**< 噪声压制+旁瓣对消额外抑制 */
constexpr float kDeceptionAgilityFactor = 0.65f;       /**< 欺骗干扰+频率捷变额外抑制 */
constexpr float kDeceptionRejitterFactor = 0.65f;      /**< 欺骗干扰+ECCM 抖动额外抑制 */
constexpr float kRepeaterRejitterFactor = 0.60f;       /**< 转发干扰+ECCM 抖动额外抑制 */

// ---------- E. 主导语义判定阈值（算法逻辑阈值，不开放配置） ----------
constexpr float kMixedSemanticDominanceRatio =
    0.65f;                                       /**< 次优类型得分 >= 最优 * 此值时判为 Mixed */
constexpr float kMixedSemanticMinScore = 0.18f;  /**< 次优类型得分须超过此绝对值才判为 Mixed */
constexpr float kUnknownNoiseSplitWeight = 0.5f; /**< 未知类型拆分到噪声/欺骗的各自比例 */

// ---------- 单源噪声严重度物理权重（不开放配置，与探测链路物理量纲一致） ----------
constexpr float kPhysicalWeightNoise = 1.0f;
constexpr float kPhysicalWeightDeception = 0.20f;
constexpr float kPhysicalWeightRepeater = 0.40f;
constexpr float kPhysicalWeightUnknown = 0.70f;

/**
 * @brief 将干扰源置信度裁剪到配置允许的权重区间，作为后续加权计算的公共因子
 * @param[in] cfg 干扰效果配置，提供置信度下界 confidence_weight_min
 * @param[in] jammer_source 待评估的干扰源事实
 * @return 裁剪至 [cfg.confidence_weight_min, 1.0f] 的置信度权重
 */
float ResolveJammerConfidenceWeight(const JammingEffectsConfig& cfg,
                                    const environment::JammerSourceFact& jammer_source) {
  return ClampFloat(jammer_source.confidence, cfg.confidence_weight_min, 1.0f);
}

/**
 * @brief 计算单个干扰源在轨迹级的干扰贡献得分，参与主导语义判定（E 组）
 * @param[in] control_profile 雷达控制配置，提供 ECCM 使能状态和穿透增益
 * @param[in] jammer_source 待评估的干扰源事实
 * @return 裁剪至 [0.0f, 1.0f] 的轨迹级干扰贡献得分
 * @note 使用固定经验系数，不依赖 cfg，以保持语义判定阶段的稳定性
 */
float ComputeTrackLevelJammingContribution(
    const common::control::RadarControlProfile& control_profile,
    const environment::JammerSourceFact& jammer_source) {
  // 轨迹级贡献使用固定系数——该函数参与主导语义判定（E 组），不用 cfg。
  const float confidence_weight = ClampFloat(jammer_source.confidence, 0.25f, 1.0f);
  const float residual_factor = ComputeResidualJammerFactor(control_profile, jammer_source);
  float contribution = 0.10f + 0.020f * ClampFloat(jammer_source.power_db, 0.0f, 20.0f) +
                       0.015f * ClampFloat(jammer_source.js_db, 0.0f, 12.0f);

  switch (jammer_source.technique) {
    case environment::JammingTechnique::kNoiseSuppression:
      contribution += jammer_source.in_sidelobe ? 0.14f : 0.08f;
      break;
    case environment::JammingTechnique::kDeception:
      contribution += 0.25f * ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      contribution += 0.22f * ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      break;
    case environment::JammingTechnique::kRepeater:
      contribution += 0.18f * ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      contribution += 0.14f * ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      break;
    case environment::JammingTechnique::kUnknown:
    default:
      contribution += 0.15f * ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      contribution += 0.12f * ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      break;
  }

  return ClampFloat(contribution * confidence_weight * residual_factor, 0.0f, 1.0f);
}

}  // namespace

bool HasMultiSourceJammingFacts(const environment::EnvironmentSnapshot& environment_snapshot) {
  return !environment_snapshot.jammer_sources.empty();
}

float ComputeResidualJammerFactor(const common::control::RadarControlProfile& control_profile,
                                  const environment::JammerSourceFact& jammer_source) {
  float residual_factor = 1.0f;

  if (control_profile.enable_sidelobe_canceller && jammer_source.in_sidelobe) {
    residual_factor *= kSidelobeCancellerResidual;
  }
  if (control_profile.enable_adaptive_beamforming) {
    residual_factor = residual_factor * (jammer_source.angular_span_deg > 0.0f &&
                                                 jammer_source.angular_span_deg <= 10.0f
                                             ? kAdaptiveBeamNarrowResidual
                                             : kAdaptiveBeamWideResidual);
  }
  if (control_profile.enable_agility_frequency && jammer_source.frequency_overlap_ratio > 0.0f) {
    residual_factor *= ClampFloat(1.0f - 0.50f * jammer_source.frequency_overlap_ratio,
                                  kAgilityFreqResidualMin, 1.0f);
  }
  if (control_profile.enable_eccm_rejitter && jammer_source.prf_lock_risk > 0.0f) {
    residual_factor *=
        ClampFloat(1.0f - 0.55f * jammer_source.prf_lock_risk, kEccmRejitterResidualMin, 1.0f);
  }
  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    residual_factor *=
        ClampFloat(1.0f / control_profile.eccm_burnthrough_gain, kBurnthroughResidualMin, 1.0f);
  }

  switch (jammer_source.technique) {
    case environment::JammingTechnique::kNoiseSuppression:
      if (control_profile.enable_sidelobe_canceller && jammer_source.in_sidelobe) {
        residual_factor *= kNoiseSidelobeCancellerFactor;
      }
      break;
    case environment::JammingTechnique::kDeception:
      if (control_profile.enable_agility_frequency) {
        residual_factor *= kDeceptionAgilityFactor;
      }
      if (control_profile.enable_eccm_rejitter) {
        residual_factor *= kDeceptionRejitterFactor;
      }
      break;
    case environment::JammingTechnique::kRepeater:
      if (control_profile.enable_eccm_rejitter) {
        residual_factor *= kRepeaterRejitterFactor;
      }
      break;
    case environment::JammingTechnique::kUnknown:
    default:
      break;
  }

  return ClampFloat(residual_factor, kResidualFactorMin, 1.0f);
}

float ComputeHeuristicSourcePenaltyDb(const JammingEffectsConfig& cfg,
                                      const environment::JammerSourceFact& jammer_source) {
  const float confidence_weight = ResolveJammerConfidenceWeight(cfg, jammer_source);
  float penalty_db =
      cfg.heuristic_base_penalty_db +
      cfg.heuristic_power_penalty_slope * ClampFloat(jammer_source.power_db, 0.0f, 20.0f);

  switch (jammer_source.technique) {
    case environment::JammingTechnique::kNoiseSuppression:
      penalty_db += cfg.heuristic_noise_sidelobe_penalty *
                    (jammer_source.in_sidelobe ? 1.0f : cfg.heuristic_noise_frontlobe_ratio);
      penalty_db +=
          cfg.heuristic_noise_js_slope * ClampFloat(jammer_source.js_db, 0.0f, 12.0f) / 6.0f;
      break;
    case environment::JammingTechnique::kDeception:
      penalty_db += cfg.heuristic_deception_freq_penalty *
                    ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      penalty_db +=
          cfg.heuristic_deception_prf_penalty * ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      break;
    case environment::JammingTechnique::kRepeater:
      penalty_db +=
          cfg.heuristic_repeater_prf_penalty * ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      penalty_db += cfg.heuristic_repeater_freq_penalty *
                    ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      break;
    case environment::JammingTechnique::kUnknown:
    default:
      penalty_db += cfg.heuristic_unknown_freq_penalty *
                    ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      penalty_db +=
          cfg.heuristic_unknown_prf_penalty * ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      penalty_db += jammer_source.in_sidelobe ? cfg.heuristic_unknown_sidelobe_penalty
                                              : cfg.heuristic_unknown_frontlobe_penalty;
      break;
  }

  return penalty_db * confidence_weight;
}

float ComputeHeuristicJammingPenaltyDb(
    const JammingEffectsConfig& cfg, const environment::EnvironmentSnapshot& environment_snapshot) {
  if (!HasMultiSourceJammingFacts(environment_snapshot)) {
    return 0.0f;
  }

  float penalty_db = 0.0f;
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    penalty_db += ComputeHeuristicSourcePenaltyDb(cfg, environment_snapshot.jammer_sources[i]);
  }
  return penalty_db;
}

float ComputePhysicalSourceJamContributionW(const JammingEffectsConfig& cfg,
                                            const environment::JammerSourceFact& jammer_source) {
  float source_weight = 1.0f;
  switch (jammer_source.technique) {
    case environment::JammingTechnique::kNoiseSuppression:
      source_weight = kPhysicalWeightNoise;
      break;
    case environment::JammingTechnique::kDeception:
      source_weight = kPhysicalWeightDeception;
      break;
    case environment::JammingTechnique::kRepeater:
      source_weight = kPhysicalWeightRepeater;
      break;
    case environment::JammingTechnique::kUnknown:
    default:
      source_weight = kPhysicalWeightUnknown;
      break;
  }
  return DbToLinearPower(jammer_source.power_db) * source_weight *
         ResolveJammerConfidenceWeight(cfg, jammer_source);
}

float ComputeMeasurementCovarianceInflation(
    const JammingEffectsConfig& cfg, const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot) {
  if (!HasMultiSourceJammingFacts(environment_snapshot)) {
    return 1.0f;
  }

  float inflation = 1.0f;
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    const environment::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
    const float residual_factor = ComputeResidualJammerFactor(control_profile, source);
    const float confidence_weight = ResolveJammerConfidenceWeight(cfg, source);
    switch (source.technique) {
      case environment::JammingTechnique::kDeception:
        inflation += cfg.covariance_deception_inflation_step * confidence_weight * residual_factor *
                     (1.0f + source.frequency_overlap_ratio);
        break;
      case environment::JammingTechnique::kRepeater:
        inflation += cfg.covariance_repeater_inflation_step * confidence_weight * residual_factor *
                     (1.0f + source.prf_lock_risk);
        break;
      case environment::JammingTechnique::kNoiseSuppression:
        inflation += cfg.covariance_noise_inflation_step * confidence_weight * residual_factor *
                     (source.in_sidelobe ? 1.0f : 0.5f);
        break;
      case environment::JammingTechnique::kUnknown:
      default:
        inflation += cfg.covariance_unknown_inflation_step * confidence_weight * residual_factor;
        break;
    }
  }
  return ClampFloat(inflation, 1.0f, cfg.covariance_inflation_max);
}

common::utils::JammingSemantic ResolveDominantJammingSemantic(
    const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot) {
  if (!environment_snapshot.jamming_detected) {
    return common::utils::JammingSemantic::kNone;
  }

  if (!HasMultiSourceJammingFacts(environment_snapshot)) {
    return common::utils::JammingSemantic::kNone;
  }

  float type_scores[3] = {0.0f, 0.0f, 0.0f};
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    const environment::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
    const float contribution = ComputeTrackLevelJammingContribution(control_profile, source);
    switch (source.technique) {
      case environment::JammingTechnique::kNoiseSuppression:
        type_scores[0] += contribution;
        break;
      case environment::JammingTechnique::kDeception:
        type_scores[1] += contribution;
        break;
      case environment::JammingTechnique::kRepeater:
        type_scores[2] += contribution;
        break;
      case environment::JammingTechnique::kUnknown:
      default:
        type_scores[0] += kUnknownNoiseSplitWeight * contribution;
        type_scores[1] += kUnknownNoiseSplitWeight * contribution;
        break;
    }
  }

  std::size_t best_index = 0U;
  std::size_t second_index = 1U;
  for (std::size_t i = 1; i < 3U; ++i) {
    if (type_scores[i] > type_scores[best_index]) {
      second_index = best_index;
      best_index = i;
    } else if (i != best_index && type_scores[i] > type_scores[second_index]) {
      second_index = i;
    }
  }

  if (type_scores[best_index] <= 1e-6f) {
    return common::utils::JammingSemantic::kNone;
  }
  if (second_index != best_index &&
      type_scores[second_index] >= kMixedSemanticDominanceRatio * type_scores[best_index] &&
      type_scores[second_index] > kMixedSemanticMinScore) {
    return common::utils::JammingSemantic::kMixed;
  }

  if (best_index == 0U) {
    return common::utils::JammingSemantic::kNoiseSuppression;
  }
  if (best_index == 1U) {
    return common::utils::JammingSemantic::kDeception;
  }
  return common::utils::JammingSemantic::kRepeater;
}

float ComputeTrackLevelJammingSeverity(
    const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot) {
  if (!environment_snapshot.jamming_detected) {
    return 0.0f;
  }

  if (!HasMultiSourceJammingFacts(environment_snapshot)) {
    return 0.0f;
  }

  float total_severity = 0.0f;
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    total_severity += ComputeTrackLevelJammingContribution(control_profile,
                                                           environment_snapshot.jammer_sources[i]);
  }
  return ClampFloat(total_severity, 0.0f, 1.0f);
}

void ApplyEnvironmentJammingFactsToRuntimeConfig(
    const common::control::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot,
    InternalSignalPipelineConfig* internal_runtime_config, SignalPipelineConfig* runtime_config) {
  if (runtime_config == nullptr || internal_runtime_config == nullptr ||
      !HasMultiSourceJammingFacts(environment_snapshot)) {
    return;
  }
  const JammingEffectsConfig& cfg = internal_runtime_config->jamming_effects;

  float association_scale = 1.0f;
  float tracking_noise_scale = 1.0f;
  float measurement_noise_scale = 1.0f;

  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    const environment::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
    const float residual_factor = ComputeResidualJammerFactor(control_profile, source);
    const float confidence_weight = ResolveJammerConfidenceWeight(cfg, source);

    switch (source.technique) {
      case environment::JammingTechnique::kDeception:
        association_scale += cfg.deception_association_step * confidence_weight * residual_factor *
                             (1.0f + source.frequency_overlap_ratio);
        tracking_noise_scale += cfg.deception_tracking_step * confidence_weight * residual_factor *
                                (1.0f + source.prf_lock_risk);
        measurement_noise_scale +=
            cfg.deception_measurement_step * confidence_weight * residual_factor;
        break;
      case environment::JammingTechnique::kRepeater:
        association_scale += cfg.repeater_association_step * confidence_weight * residual_factor *
                             (1.0f + source.prf_lock_risk);
        tracking_noise_scale += cfg.repeater_tracking_step * confidence_weight * residual_factor *
                                (1.0f + source.prf_lock_risk);
        measurement_noise_scale +=
            cfg.repeater_measurement_step * confidence_weight * residual_factor;
        break;
      case environment::JammingTechnique::kNoiseSuppression:
        measurement_noise_scale += cfg.noise_measurement_step * confidence_weight * residual_factor;
        break;
      case environment::JammingTechnique::kUnknown:
      default:
        association_scale += cfg.unknown_association_step * confidence_weight * residual_factor;
        tracking_noise_scale += cfg.unknown_tracking_step * confidence_weight * residual_factor;
        measurement_noise_scale +=
            cfg.unknown_measurement_step * confidence_weight * residual_factor;
        break;
    }
  }

  internal_runtime_config->association.unassigned_cost *=
      ClampFloat(association_scale, 1.0f, cfg.association_scale_max);
  internal_runtime_config->tracking.kalman_noise_diff_coeff *=
      ClampFloat(tracking_noise_scale, 1.0f, cfg.tracking_noise_scale_max);
  runtime_config->tracking.kalman_measurement_noise_std *=
      ClampFloat(measurement_noise_scale, 1.0f, cfg.measurement_noise_scale_max);
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
