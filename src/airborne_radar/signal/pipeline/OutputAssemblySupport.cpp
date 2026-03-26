#include "airborne_radar/signal/pipeline/OutputAssemblySupport.h"

#include "airborne_radar/signal/pipeline/DecisionFrameBuilders.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

std::uint32_t ResolveLifecycleExtraMissTolerance(
    const common::RadarControlProfile& control_profile) {
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

void CollectCycleOutputs(
    const common::RadarControlProfile& control_profile, std::uint32_t cycle_index,
    std::uint64_t batch_id, const SignalPipelineConfig& runtime_config,
    const environment::EnvironmentSnapshot& environment_snapshot,
    const common::TargetFeatureList& input_state, const common::TargetFeatureList& output_state,
    const association::AssociationResult& association_result,
    const std::vector<tracking::TrackMeasurement>& track_measurements,
    core::output::DataOutputManager* output_manager,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager,
    AssociationQualityMetrics* association_quality_metrics, common::DecisionInputFrame* decision_frame) {
  if (output_manager == nullptr || association_quality_metrics == nullptr || decision_frame == nullptr) {
    return;
  }

  const common::JammingSemantic dominant_jamming_semantic =
      ResolveDominantJammingSemantic(control_profile, environment_snapshot);
  const float jamming_severity =
      ComputeTrackLevelJammingSeverity(control_profile, environment_snapshot);
  *association_quality_metrics = ToPipelineAssociationQualityMetrics(
      association_result.quality_metrics, dominant_jamming_semantic, jamming_severity,
      runtime_config.association.unassigned_cost);

  const common::EccmSourceInfo eccm_source_info = BuildEccmSourceInfo(environment_snapshot);
  const common::AssociationQualityInfo association_quality_info =
      BuildAssociationQualityInfo(*association_quality_metrics);
  const common::PerceptionQualityInfo perception_quality_info =
      BuildPerceptionQualityInfo(input_state.size(), *association_quality_metrics);

  if (auto_lifecycle_manager != nullptr) {
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
    return;
  }

  const common::TrackOutputFrame track_output_frame =
      output_manager->BuildTrackOutputFrame(cycle_index, batch_id,
                                            BuildDecisionSnapshotsFromFeatures(output_state));
  *decision_frame = output_manager->BuildDecisionInputFrame(
      track_output_frame, eccm_source_info, association_quality_info, perception_quality_info);
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
