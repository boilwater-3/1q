#include "airborne_radar/signal/pipeline/core/CycleExecutor.h"

#include <vector>

#include "airborne_radar/signal/pipeline/core/AssociationExecutionSupport.h"
#include "airborne_radar/signal/pipeline/core/ContextBindingSupport.h"
#include "airborne_radar/signal/pipeline/core/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/core/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/effects/JammingEffects.h"
#include "airborne_radar/signal/pipeline/assembly/OutputAssemblySupport.h"
#include "airborne_radar/signal/pipeline/assembly/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/core/ScanScheduleResolver.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

bool HasValidRuntime(const CycleExecutionRuntime& runtime) {
  return runtime.base_config != nullptr && runtime.base_internal_config != nullptr &&
         runtime.control_profile != nullptr && runtime.association_engine != nullptr &&
         runtime.track_filter != nullptr;
}

CycleExecutionContract BuildCycleExecutionContract(const model::TargetFeatureList& input_state,
                                                   const extension::IEnvironmentService& environment,
                                                   std::uint32_t cycle_index,
                                                   std::uint64_t batch_id,
                                                   const CycleExecutionRuntime& runtime) {
  CycleExecutionContract output;
  output.input_state = &input_state;
  output.environment = &environment;
  output.cycle_index = cycle_index;
  output.batch_id = batch_id;

  const assembly::internal::ResolvedRuntimeSignalPipelineConfig resolved =
      assembly::internal::ResolveRuntimeSignalPipelineConfig(
          *runtime.base_config, *runtime.base_internal_config, *runtime.control_profile);
  output.runtime_config = resolved.public_config;
  output.internal_runtime_config = resolved.internal_config;
  core::internal::ApplyScanScheduleToRuntimeConfig(cycle_index, &output.runtime_config);
  return output;
}

void PrepareExecutionState(const CycleExecutionContract& contract,
                           const CycleExecutionRuntime& runtime, CycleExecutionScratch* scratch) {
  CycleWorkspace cycle_workspace = BuildCycleWorkspaceBindings(
      &scratch->output_state, &scratch->decision_frame, &scratch->association_quality_metrics,
      &scratch->track_measurements, &scratch->signal_term_db, &scratch->speed_penalty_db,
      &scratch->detection_margin_db, &scratch->detection_succeeded, &scratch->association_keys,
      &scratch->measurement_slots, &scratch->target_geometry, &scratch->measurement_covariances,
      &scratch->association_result);
  ResetCycleWorkspace(*contract.input_state, contract.runtime_config, &cycle_workspace);
  SyncAssociationAndTrackFilterConfigs(contract.runtime_config, contract.internal_runtime_config,
                                       runtime.association_engine, runtime.track_filter,
                                       runtime.auto_lifecycle_manager);
}

void PrepareAssociationSeeds(const CycleExecutionRuntime& runtime) {
  const std::vector<tracking::AssociationTrackSeed> kEmptyAssociationSeeds;
  const std::vector<tracking::AssociationTrackSeed>& association_seeds =
      runtime.manual_association_seeds != nullptr ? *runtime.manual_association_seeds
                                                  : kEmptyAssociationSeeds;
  PrepareAssociationSeedsForCycle(runtime.has_manual_association_seeds, association_seeds,
                                  runtime.auto_lifecycle_manager, runtime.association_engine);
}

EnvironmentPhaseOutput RunEnvironmentPhase(CycleExecutionContract* contract,
                                           const CycleExecutionRuntime& runtime,
                                           CycleExecutionScratch* scratch) {
  EnvironmentPhaseOutput output;
  output.environment_snapshot = contract->environment->SampleEnvironment();

  ApplyEnvironmentJammingFactsToRuntimeConfig(*runtime.control_profile, output.environment_snapshot,
                                              &contract->internal_runtime_config,
                                              &contract->runtime_config);
  RefreshMeasurementCovariances(scratch->target_geometry.size(),
                                contract->runtime_config.tracking.kalman_measurement_noise_std,
                                &scratch->measurement_covariances);
  SyncAssociationAndTrackFilterConfigs(contract->runtime_config, contract->internal_runtime_config,
                                       runtime.association_engine, runtime.track_filter,
                                       runtime.auto_lifecycle_manager);

  output.dominant_jamming_semantic =
      ResolveDominantJammingSemantic(*runtime.control_profile, output.environment_snapshot);
  output.jamming_severity =
      ComputeTrackLevelJammingSeverity(*runtime.control_profile, output.environment_snapshot);
  return output;
}

DetectionPhaseOutput RunDetectionPhase(const CycleExecutionContract& contract,
                                       const CycleExecutionRuntime& runtime,
                                       const EnvironmentPhaseOutput& environment_phase,
                                       CycleExecutionScratch* scratch) {
  DetectionExecutionBuffers detection_buffers = BuildDetectionExecutionBuffers(
      &scratch->target_geometry, &scratch->signal_term_db, &scratch->speed_penalty_db,
      &scratch->detection_margin_db, &scratch->detection_succeeded,
      &scratch->measurement_covariances);

  if (contract.runtime_config.detection.enable_physics_detection &&
      runtime.signal_detector != nullptr) {
    RunPhysicalDetectionPass(*contract.input_state, contract.runtime_config,
                             contract.internal_runtime_config,
                             *runtime.control_profile, environment_phase.environment_snapshot,
                             runtime.signal_detector, &detection_buffers);
  } else {
    RunHeuristicDetectionPass(*contract.input_state, contract.runtime_config,
                              contract.internal_runtime_config,
                              *runtime.control_profile, environment_phase.environment_snapshot,
                              &detection_buffers);
  }

  DetectionPhaseOutput output;
  output.detection_succeeded = &scratch->detection_succeeded;
  output.detection_margin_db = &scratch->detection_margin_db;
  output.measurement_covariances = &scratch->measurement_covariances;
  output.target_geometry = &scratch->target_geometry;
  return output;
}

AssociationPhaseOutput RunAssociationPhase(const CycleExecutionContract& contract,
                                           const CycleExecutionRuntime& runtime,
                                           const EnvironmentPhaseOutput& environment_phase,
                                           const DetectionPhaseOutput& detection_phase,
                                           CycleExecutionScratch* scratch) {
  RunAssociationPass(*contract.input_state, *detection_phase.detection_succeeded,
                     *detection_phase.measurement_covariances,
                     environment_phase.environment_snapshot.cycle_dt_sec,
                     runtime.association_engine, &scratch->association_result,
                     &scratch->association_keys);

  AssociationPhaseOutput output;
  output.association_result = &scratch->association_result;
  output.association_keys = &scratch->association_keys;
  return output;
}

MeasurementBuildPhaseOutput RunMeasurementBuildPhase(
    const CycleExecutionContract& contract, const EnvironmentPhaseOutput& environment_phase,
    const DetectionPhaseOutput& detection_phase, const AssociationPhaseOutput& association_phase,
    CycleExecutionScratch* scratch) {
  TrackMeasurementBuildContext build_context = BuildTrackMeasurementBuildContextBindings(
      contract.input_state, association_phase.association_result,
      detection_phase.detection_succeeded,
      association_phase.association_keys, detection_phase.detection_margin_db,
      detection_phase.target_geometry, detection_phase.measurement_covariances,
      environment_phase.environment_snapshot.jamming_detected,
      environment_phase.dominant_jamming_semantic, environment_phase.jamming_severity,
      &scratch->measurement_slots, &scratch->track_measurements);
  BuildTrackMeasurementsPass(build_context);

  MeasurementBuildPhaseOutput output;
  output.measurement_slots = &scratch->measurement_slots;
  output.track_measurements = &scratch->track_measurements;
  return output;
}

void RunTrackFilterPhase(const CycleExecutionContract& contract,
                         const CycleExecutionRuntime& runtime,
                         const EnvironmentPhaseOutput& environment_phase,
                         const DetectionPhaseOutput& detection_phase,
                         const MeasurementBuildPhaseOutput& measurement_phase,
                         CycleExecutionScratch* scratch) {
  TrackFilterApplyContext filter_context = BuildTrackFilterApplyContextBindings(
      contract.input_state, &scratch->output_state, detection_phase.detection_succeeded,
      detection_phase.detection_margin_db, environment_phase.environment_snapshot.jamming_detected,
      environment_phase.dominant_jamming_semantic, environment_phase.jamming_severity,
      runtime.track_filter, measurement_phase.measurement_slots, &scratch->track_measurements);
  ApplyTrackFilterPass(filter_context);
}

void AssembleOutputs(std::uint32_t cycle_index, std::uint64_t batch_id,
                     const CycleExecutionContract& contract,
                     const EnvironmentPhaseOutput& environment_phase,
                     const CycleExecutionRuntime& runtime,
                     const AssociationPhaseOutput& association_phase,
                     const MeasurementBuildPhaseOutput& measurement_phase,
                     CycleExecutionScratch* scratch) {
  CollectCycleOutputs(*runtime.control_profile, cycle_index, batch_id,
                      contract.internal_runtime_config,
                      environment_phase.environment_snapshot, *contract.input_state,
                      *association_phase.association_result, *measurement_phase.track_measurements,
                      runtime.auto_lifecycle_manager, &scratch->association_quality_metrics,
                      &scratch->decision_frame);
}

}  // namespace

void ExecuteCycle(const model::TargetFeatureList& input_state,
                  const extension::IEnvironmentService& environment, std::uint32_t cycle_index,
                  std::uint64_t batch_id, const CycleExecutionRuntime& runtime,
                  CycleExecutionScratch* cycle_scratch) {
  if (cycle_scratch == nullptr) {
    PROJECT_LOG_ERROR("[CycleExecutor] ExecuteCycle called with null cycle_scratch.");
    return;
  }
  if (!HasValidRuntime(runtime)) {
    PROJECT_LOG_ERROR(
        "[CycleExecutor] ExecuteCycle received invalid runtime: base_config={} "
        "base_internal_config={} control_profile={} association_engine={} track_filter={}",
        runtime.base_config != nullptr, runtime.base_internal_config != nullptr,
        runtime.control_profile != nullptr, runtime.association_engine != nullptr,
        runtime.track_filter != nullptr);
    return;
  }

  CycleExecutionContract contract =
      BuildCycleExecutionContract(input_state, environment, cycle_index, batch_id, runtime);
  PrepareExecutionState(contract, runtime, cycle_scratch);
  PrepareAssociationSeeds(runtime);

  const EnvironmentPhaseOutput environment_phase =
      RunEnvironmentPhase(&contract, runtime, cycle_scratch);
  const DetectionPhaseOutput detection_phase =
      RunDetectionPhase(contract, runtime, environment_phase, cycle_scratch);
  const AssociationPhaseOutput association_phase =
      RunAssociationPhase(contract, runtime, environment_phase, detection_phase, cycle_scratch);
  const MeasurementBuildPhaseOutput measurement_phase =
      RunMeasurementBuildPhase(contract, environment_phase, detection_phase, association_phase,
                               cycle_scratch);
  RunTrackFilterPhase(contract, runtime, environment_phase, detection_phase, measurement_phase,
                      cycle_scratch);
  AssembleOutputs(contract.cycle_index, contract.batch_id, contract, environment_phase, runtime,
                  association_phase,
                  measurement_phase, cycle_scratch);
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
