#include "airborne_radar/signal/pipeline/CycleExecutor.h"

#include <vector>

#include "airborne_radar/signal/pipeline/AssociationExecutionSupport.h"
#include "airborne_radar/signal/pipeline/ContextBindingSupport.h"
#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/signal/pipeline/OutputAssemblySupport.h"
#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/runtime/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/runtime/ScanScheduleResolver.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

bool HasValidRuntime(const CycleExecutionRuntime& runtime) {
  return runtime.base_config != nullptr && runtime.control_profile != nullptr &&
         runtime.association_engine != nullptr && runtime.track_filter != nullptr &&
         runtime.output_manager != nullptr;
}

}  // namespace

void ExecuteCycle(const common::TargetFeatureList& input_state,
                  const environment::IEnvironmentService& environment,
                  std::uint32_t cycle_index, std::uint64_t batch_id,
                  const CycleExecutionRuntime& runtime, CycleExecutionContext* cycle_context) {
  if (cycle_context == nullptr || !HasValidRuntime(runtime)) {
    return;
  }

  cycle_context->input_state = &input_state;
  cycle_context->environment = &environment;
  cycle_context->runtime_config = runtime::internal::BuildRuntimeConfigFromControlProfile(
      *runtime.base_config, *runtime.control_profile);
  runtime::internal::ApplyScanScheduleToRuntimeConfig(cycle_index, &cycle_context->runtime_config);

  CycleWorkspace cycle_workspace = BuildCycleWorkspaceBindings(
      &cycle_context->output_state, &cycle_context->decision_frame,
      &cycle_context->association_quality_metrics, &cycle_context->track_measurements,
      &cycle_context->signal_term_db, &cycle_context->speed_penalty_db,
      &cycle_context->detection_margin_db, &cycle_context->detection_succeeded,
      &cycle_context->association_keys, &cycle_context->measurement_slots,
      &cycle_context->target_geometry, &cycle_context->measurement_covariances,
      &cycle_context->association_result);
  ResetCycleWorkspace(input_state, cycle_context->runtime_config, &cycle_workspace);
  SyncAssociationAndTrackFilterConfigs(cycle_context->runtime_config, runtime.association_engine,
                                       runtime.track_filter);

  const std::vector<tracking::AssociationTrackSeed> kEmptyAssociationSeeds;
  const std::vector<tracking::AssociationTrackSeed>& association_seeds =
      runtime.manual_association_seeds != nullptr ? *runtime.manual_association_seeds
                                                  : kEmptyAssociationSeeds;
  PrepareAssociationSeedsForCycle(runtime.has_manual_association_seeds, association_seeds,
                                  runtime.auto_lifecycle_manager, runtime.association_engine);

  cycle_context->environment_snapshot = environment.SampleEnvironment();
  ApplyEnvironmentJammingFactsToRuntimeConfig(
      cycle_context->runtime_config.jamming_effects, *runtime.control_profile,
      cycle_context->environment_snapshot, &cycle_context->runtime_config);
  RefreshMeasurementCovariances(cycle_context->target_geometry.size(),
                                cycle_context->runtime_config.tracking.kalman_measurement_noise_std,
                                &cycle_context->measurement_covariances);
  SyncAssociationAndTrackFilterConfigs(cycle_context->runtime_config, runtime.association_engine,
                                       runtime.track_filter);
  cycle_context->dominant_jamming_semantic = ResolveDominantJammingSemantic(
      *runtime.control_profile, cycle_context->environment_snapshot);
  cycle_context->jamming_severity =
      ComputeTrackLevelJammingSeverity(*runtime.control_profile,
                                       cycle_context->environment_snapshot);

  DetectionExecutionBuffers detection_buffers = BuildDetectionExecutionBuffers(
      &cycle_context->target_geometry, &cycle_context->signal_term_db,
      &cycle_context->speed_penalty_db, &cycle_context->detection_margin_db,
      &cycle_context->detection_succeeded, &cycle_context->measurement_covariances);
  if (cycle_context->runtime_config.detection.enable_physics_detection &&
      runtime.signal_detector != nullptr) {
    RunPhysicalDetectionPass(input_state, cycle_context->runtime_config, *runtime.control_profile,
                             cycle_context->environment_snapshot, runtime.signal_detector,
                             &detection_buffers);
  } else {
    RunHeuristicDetectionPass(input_state, cycle_context->runtime_config, *runtime.control_profile,
                              cycle_context->environment_snapshot, &detection_buffers);
  }

  RunAssociationPass(input_state, cycle_context->detection_succeeded,
                     cycle_context->measurement_covariances,
                     cycle_context->environment_snapshot.cycle_dt_sec,
                     runtime.association_engine, &cycle_context->association_result,
                     &cycle_context->association_keys);

  TrackMeasurementBuildContext measurement_build_context =
      BuildTrackMeasurementBuildContextBindings(
          cycle_context->input_state, &cycle_context->association_result,
          &cycle_context->detection_succeeded, &cycle_context->association_keys,
          &cycle_context->detection_margin_db, &cycle_context->target_geometry,
          &cycle_context->measurement_covariances,
          cycle_context->environment_snapshot.jamming_detected,
          cycle_context->dominant_jamming_semantic, cycle_context->jamming_severity,
          &cycle_context->measurement_slots, &cycle_context->track_measurements);
  BuildTrackMeasurementsPass(measurement_build_context);

  TrackFilterApplyContext track_filter_apply_context = BuildTrackFilterApplyContextBindings(
      cycle_context->input_state, &cycle_context->output_state,
      &cycle_context->detection_succeeded, &cycle_context->detection_margin_db,
      cycle_context->environment_snapshot.jamming_detected,
      cycle_context->dominant_jamming_semantic, cycle_context->jamming_severity,
      runtime.track_filter, &cycle_context->measurement_slots, &cycle_context->track_measurements);
  ApplyTrackFilterPass(track_filter_apply_context);

  CollectCycleOutputs(
      *runtime.control_profile, cycle_index, batch_id, cycle_context->runtime_config,
      cycle_context->environment_snapshot, *cycle_context->input_state,
      cycle_context->output_state, cycle_context->association_result,
      cycle_context->track_measurements, runtime.output_manager,
      runtime.auto_lifecycle_manager, &cycle_context->association_quality_metrics,
      &cycle_context->decision_frame);
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
