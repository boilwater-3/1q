#include "airborne_radar/signal/pipeline/CycleExecutor.h"

#include <algorithm>
#include <vector>

#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/DecisionFrameBuilders.h"
#include "airborne_radar/signal/pipeline/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"
#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/pipeline/DeceptionMeasurementGenerator.h"
#include "common/geometry/BearingCluster.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

// ---------------------------------------------------------------------------
// 关联种子
// ---------------------------------------------------------------------------

void PrepareAssociationSeeds(const CycleExecutionRuntime& runtime) {
  if (runtime.has_manual_association_seeds) {
    runtime.association_engine.SetAssociationSeeds(runtime.manual_association_seeds);
    return;
  }
  runtime.association_engine.SetAssociationSeeds(
      runtime.auto_lifecycle_manager.BuildAssociationSeeds());
}

// ---------------------------------------------------------------------------
// 环境阶段：同步周期配置。
// ---------------------------------------------------------------------------

bool RunEnvironmentPhase(CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                         CycleExecutionScratch& scratch) {
  RefreshMeasurementCovariances(
      scratch.target_geometry.size(),
      context.runtime_config.tracking.engineering.kalman_measurement_noise_std,
      &scratch.measurement_covariances);
  if (!SyncAssociationAndTrackFilterConfigs(context.runtime_config, &runtime.association_engine,
                                            &runtime.track_filter,
                                            &runtime.auto_lifecycle_manager)) {
    PROJECT_LOG_ERROR(
        "[SignalPipeline] environment phase aborted because runtime config sync failed.");
    return false;
  }

  return true;
}

// ---------------------------------------------------------------------------
// 检测阶段：写入 scratch 的 detection_* / target_geometry / measurement_covariances
// ---------------------------------------------------------------------------

bool RunDetectionPhase(const CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                       CycleExecutionScratch& scratch) {
  DetectionExecutionBuffers detection_buffers;
  detection_buffers.target_geometry = &scratch.target_geometry;
  detection_buffers.signal_term_db = &scratch.signal_term_db;
  detection_buffers.speed_penalty_db = &scratch.speed_penalty_db;
  detection_buffers.detection_margin_db = &scratch.detection_margin_db;
  detection_buffers.detection_succeeded = &scratch.detection_succeeded;
  detection_buffers.measurement_covariances = &scratch.measurement_covariances;

  return RunPhysicalDetectionPass(
      context.input_state, context.runtime_config, context.environment_snapshot,
      context.platform_altitude_m, context.rf_v2_detection_context, runtime.signal_detector,
      &detection_buffers);
}

// ---------------------------------------------------------------------------
// 关联阶段：写入 scratch.association_result / association_keys
// ---------------------------------------------------------------------------

void RunAssociationPhase(const CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                         CycleExecutionScratch& scratch) {
  scratch.association_result = runtime.association_engine.AssociateDetections(
      context.input_state, scratch.detection_succeeded, scratch.measurement_covariances,
      context.environment_snapshot.cycle_dt_sec);
  scratch.association_keys = scratch.association_result.target_keys;
}

// ---------------------------------------------------------------------------
// 量测构建阶段：写入 scratch.measurement_slots / track_measurements
// ---------------------------------------------------------------------------

// 将接收机干扰观测中的假目标标注按方位匹配到对应量测，供航迹起批鉴别抑制确认。
// 匹配口径：量测来自的目标 look angle（雷达局部系）与某条疑似假目标干扰观测的局部方位
// （estimated_bearing_*_local_deg，同系）落在同一波束宽度内。此前实现直接比较 ECEF
// 切平面方位与局部 look angle，平台不在 ECEF 原点或姿态非零时确定性漏标/误标。
// 无局部方位（pose 缺失）时回退 ECEF 方位并告警，避免静默误标。
void TagFalseTargetMeasurements(const CycleExecutionContext& context,
                                const ExecutionConfig& runtime_config,
                                CycleExecutionScratch& scratch) {
  if (context.interference_observations == nullptr ||
      context.interference_observations->empty() ||
      scratch.track_measurements.empty()) {
    return;
  }
  // 收集疑似假目标干扰观测的方位（优先雷达局部系，缺失时回退 ECEF 切平面）。
  std::vector<std::pair<double, double>> false_target_bearings;
  bool fell_back_to_ecef = false;
  for (const session::ArInterferenceObservation& obs : *context.interference_observations) {
    if (obs.deception_class == session::DeceptionClass::kLikelyFalseTarget) {
      if (obs.estimated_bearing_azimuth_local_deg != 0.0 ||
          obs.estimated_bearing_elevation_local_deg != 0.0) {
        false_target_bearings.emplace_back(obs.estimated_bearing_azimuth_local_deg,
                                          obs.estimated_bearing_elevation_local_deg);
      } else {
        fell_back_to_ecef = true;
        false_target_bearings.emplace_back(obs.estimated_bearing_azimuth_deg,
                                          obs.estimated_bearing_elevation_deg);
      }
    }
  }
  if (false_target_bearings.empty()) {
    return;
  }
  if (fell_back_to_ecef) {
    PROJECT_LOG_WARN(
        "[CycleExecutor] false-target bearing match fell back to ECEF tangent-plane "
        "bearings; platform frame was not available, cross-frame discrimination degraded.");
  }
  const double beamwidth_deg = std::max(
      static_cast<double>(runtime_config.detection.engineering.antenna.nominal_az_beamwidth_deg),
      static_cast<double>(runtime_config.detection.engineering.antenna.nominal_el_beamwidth_deg));
  for (tracking::TrackMeasurement& measurement : scratch.track_measurements) {
    const std::size_t source_index = measurement.raw_measurement.source_index;
    if (source_index >= scratch.target_geometry.size()) {
      continue;
    }
    const detection::TargetLookAnglesDeg& look =
        scratch.target_geometry[source_index].look_angles_deg;
    if (!look.has_look_angles) {
      continue;
    }
    for (const auto& bearing : false_target_bearings) {
      if (oneq::common::geometry::AreBearingsCoherent(
              static_cast<double>(look.look_az_deg), static_cast<double>(look.look_el_deg),
              bearing.first, bearing.second, beamwidth_deg)) {
        measurement.raw_measurement.classified_as_false_target = true;
        break;
      }
    }
  }
}

void RunMeasurementBuildPhase(const CycleExecutionContext& context,
                              CycleExecutionScratch& scratch) {
  BuildTrackMeasurementsPass(context.input_state, scratch);
  // 从欺骗干扰观测合成假目标量测，注入到 track_measurements。必须在真实量测构建之后、
  // 方位匹配标注之前，使鉴别真正作用于合成假目标而非真实场景目标。
  InjectDeceptionMeasurementsPass(context, context.runtime_config, scratch);
  TagFalseTargetMeasurements(context, context.runtime_config, scratch);
}

// ---------------------------------------------------------------------------
// 滤波阶段：写入 scratch.output_state / track_measurements.filtered_feature
// ---------------------------------------------------------------------------

void RunTrackFilterPhase(const CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                         CycleExecutionScratch& scratch) {
  ApplyTrackFilterPass(context.input_state, runtime.track_filter, scratch);
}

// ---------------------------------------------------------------------------
// 输出收尾
// ---------------------------------------------------------------------------

namespace {

AssociationQualityMetrics ToPipelineAssociationQualityMetrics(
    const association::AssociationQualityMetrics& source, float association_unassigned_cost) {
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
  const float normalized_cost_pressure =
      association_unassigned_cost > 1e-6f
          ? std::max(0.0f, std::min(1.0f, source.mean_match_cost / association_unassigned_cost))
          : 0.0f;
  const float operational_pressure =
      0.20f + 0.30f * std::max(0.0f, std::min(1.0f, 1.0f - source.match_rate)) +
      0.20f * source.new_track_rate + 0.15f * source.missed_track_rate +
      0.15f * normalized_cost_pressure;
  metrics.association_stress = std::max(0.0f, std::min(1.0f, operational_pressure));
  return metrics;
}

}  // namespace

void CollectCycleOutputs(std::uint32_t cycle_index, std::uint64_t batch_id,
                         const ExecutionConfig& runtime_config,
                         const session::EnvironmentSnapshot& environment_snapshot,
                         const session::ArSceneTargetList& input_state,
                         tracking::ITrackLifecycleManager* auto_lifecycle_manager,
                         CycleExecutionScratch& scratch) {
  if (auto_lifecycle_manager == nullptr) {
    PROJECT_LOG_ERROR(
        "[CycleExecutor] auto_lifecycle_manager is null; decision frame assembly aborted.");
    return;
  }

  scratch.association_quality_metrics = ToPipelineAssociationQualityMetrics(
      scratch.association_result.quality_metrics,
      runtime_config.association.policy.unassigned_cost);

  const session::AssociationQualityInfo association_quality_info =
      BuildAssociationQualityInfo(scratch.association_quality_metrics);
  const session::PerceptionQualityInfo perception_quality_info =
      BuildPerceptionQualityInfo(input_state.size(), scratch.association_quality_metrics);

  tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = environment_snapshot.cycle_dt_sec;
  cycle.extra_miss_tolerance = 0U;
  auto_lifecycle_manager->Update(cycle, scratch.track_measurements);
  scratch.decision_frame =
      session::DecisionInputFrame(auto_lifecycle_manager->BuildTrackStateSnapshots());
  scratch.decision_frame.cycle_index = cycle_index;
  scratch.decision_frame.batch_id = batch_id;
  scratch.decision_frame.association_quality_info = association_quality_info;
  scratch.decision_frame.perception_quality_info = perception_quality_info;
}

void AssembleOutputs(std::uint32_t cycle_index, std::uint64_t batch_id,
                     const CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                     CycleExecutionScratch& scratch) {
  CollectCycleOutputs(cycle_index, batch_id, context.runtime_config, context.environment_snapshot,
                      context.input_state, &runtime.auto_lifecycle_manager, scratch);
}

}  // namespace

// ---------------------------------------------------------------------------
// 顶层入口
// ---------------------------------------------------------------------------

bool ExecuteCycle(CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                  CycleExecutionScratch& cycle_scratch) {
  ResetCycleExecutionScratch(context.input_state, cycle_scratch);

  if (!RunEnvironmentPhase(context, runtime, cycle_scratch)) {
    return false;
  }
  PrepareAssociationSeeds(runtime);
  if (!RunDetectionPhase(context, runtime, cycle_scratch)) {
    return false;
  }
  RunAssociationPhase(context, runtime, cycle_scratch);
  RunMeasurementBuildPhase(context, cycle_scratch);
  RunTrackFilterPhase(context, runtime, cycle_scratch);
  AssembleOutputs(context.cycle_index, context.batch_id, context, runtime, cycle_scratch);
  return true;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
