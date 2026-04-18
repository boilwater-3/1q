#include "airborne_radar/signal/pipeline/assembly/OutputAssemblySupport.h"

#include <algorithm>

#include "airborne_radar/signal/pipeline/assembly/DecisionFrameBuilders.h"
#include "airborne_radar/signal/pipeline/effects/JammingEffects.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

/** @brief 根据干扰语义计算关联脆弱性权重。
 *  @param semantic 主导干扰语义类型。
 *  @return 脆弱性权重，范围 [0, 1]，欺骗类干扰权重最高。 */
float ResolveAssociationFragilityWeight(model::JammingSemantic semantic) {
  switch (semantic) {
    case model::JammingSemantic::kDeception:
      return 1.00f;
    case model::JammingSemantic::kRepeater:
      return 0.88f;
    case model::JammingSemantic::kMixed:
      return 0.94f;
    case model::JammingSemantic::kNoiseSuppression:
      return 0.60f;
    case model::JammingSemantic::kNone:
    default:
      return 0.0f;
  }
}

/** @brief 将关联域质量度量转换为流水线层关联质量度量。
 *  @param source 关联域原始质量度量。
 *  @param dominant_jamming_semantic 主导干扰语义类型。
 *  @param jamming_severity 干扰严重程度，范围 [0, 1]。
 *  @param association_unassigned_cost 未分配代价，用于归一化代价压力。
 *  @return 包含关联压力计算的流水线层关联质量度量。 */
AssociationQualityMetrics ToPipelineAssociationQualityMetrics(
    const association::AssociationQualityMetrics& source,
    model::JammingSemantic dominant_jamming_semantic, float jamming_severity,
    float association_unassigned_cost) {
  AssociationQualityMetrics metrics;
  metrics.prior_track_count = source.prior_track_count;
  metrics.detection_count = source.detection_count;
  metrics.matched_count = source.matched_count;
  metrics.new_track_count = source.new_track_count;
  metrics.missed_track_count = source.missed_track_count;
  metrics.match_rate = source.match_rate;
  metrics.new_track_rate = source.new_track_rate;
  metrics.missed_track_rate = source.missed_track_rate;
  metrics.mean_match_cost = source.mean_match_cost;
  metrics.p95_match_cost = source.p95_match_cost;
  metrics.dominant_jamming_semantic = dominant_jamming_semantic;
  metrics.jamming_severity = std::max(0.0f, std::min(1.0f, jamming_severity));
  const float normalized_cost_pressure =
      association_unassigned_cost > 1e-6f
          ? std::max(0.0f, std::min(1.0f, source.mean_match_cost / association_unassigned_cost))
          : 0.0f;
  const float operational_pressure =
      0.20f + 0.30f * std::max(0.0f, std::min(1.0f, 1.0f - source.match_rate)) +
      0.20f * source.new_track_rate + 0.15f * source.missed_track_rate +
      0.15f * normalized_cost_pressure;
  metrics.association_stress = std::max(
      0.0f,
      std::min(1.0f, metrics.jamming_severity *
                         ResolveAssociationFragilityWeight(metrics.dominant_jamming_semantic) *
                         operational_pressure));
  return metrics;
}

/** @brief 根据雷达控制配置计算生命周期额外漏检容忍次数。
 *  @param control_profile 雷达控制配置文件。
 *  @return 额外漏检容忍次数，ECCM 相关功能启用时累加。 */
std::uint32_t ResolveLifecycleExtraMissTolerance(
    const extension::control::RadarControlProfile& control_profile) {
  std::uint32_t extra_miss_tolerance = 0U;
  if (control_profile.enable_sidelobe_canceller || control_profile.enable_agility_frequency ||
      control_profile.enable_eccm_rejitter) {
    extra_miss_tolerance += 1U;
  }
  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    extra_miss_tolerance += 1U;
  }
  return extra_miss_tolerance;
}

}  // namespace

void CollectCycleOutputs(const extension::control::RadarControlProfile& control_profile,
                         std::uint32_t cycle_index, std::uint64_t batch_id,
                         const InternalPipelineConfig& internal_runtime_config,
                         const environment::EnvironmentSnapshot& environment_snapshot,
                         const model::TargetFeatureList& input_state,
                         const association::AssociationResult& association_result,
                         const std::vector<tracking::TrackMeasurement>& track_measurements,
                         tracking::ITrackLifecycleManager* auto_lifecycle_manager,
                         AssociationQualityMetrics* association_quality_metrics,
                         model::DecisionInputFrame* decision_frame) {
  if (association_quality_metrics == nullptr || decision_frame == nullptr) {
    return;
  }

  const model::JammingSemantic dominant_jamming_semantic =
      ResolveDominantJammingSemantic(control_profile, environment_snapshot);
  const float jamming_severity =
      ComputeTrackLevelJammingSeverity(control_profile, environment_snapshot);
  *association_quality_metrics = ToPipelineAssociationQualityMetrics(
      association_result.quality_metrics, dominant_jamming_semantic, jamming_severity,
      internal_runtime_config.association.unassigned_cost);

  const model::EccmSourceInfo eccm_source_info = BuildEccmSourceInfo(environment_snapshot);
  const model::AssociationQualityInfo association_quality_info =
      BuildAssociationQualityInfo(*association_quality_metrics);
  const model::PerceptionQualityInfo perception_quality_info =
      BuildPerceptionQualityInfo(input_state.size(), *association_quality_metrics);

  if (auto_lifecycle_manager == nullptr) {
    PROJECT_LOG_ERROR(
        "[OutputAssemblySupport] auto_lifecycle_manager is null; decision frame assembly aborted.");
    return;
  }

  tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = environment_snapshot.cycle_dt_sec;
  cycle.extra_miss_tolerance = ResolveLifecycleExtraMissTolerance(control_profile);
  auto_lifecycle_manager->Update(cycle, track_measurements);
  *decision_frame = auto_lifecycle_manager->BuildDecisionFrame(
      cycle_index, batch_id, eccm_source_info.has_jamming_signal);
  decision_frame->environment_jamming_detected = eccm_source_info.has_jamming_signal;
  decision_frame->eccm_source_info = eccm_source_info;
  decision_frame->association_quality_info = association_quality_info;
  decision_frame->perception_quality_info = perception_quality_info;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
