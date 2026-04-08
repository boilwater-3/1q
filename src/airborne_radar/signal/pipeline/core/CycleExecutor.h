/**
 * @file CycleExecutor.h
 * @brief 定义 SignalPipeline 单周期执行器。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/extension/IEnvironmentService.h"
#include "airborne_radar/signal/assembly/IDataOutputManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/config/InternalSignalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 单周期执行的可复用缓冲区（仅存放可变工作数据）。
 */
struct CycleExecutionScratch {
  model::TargetFeatureList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;
  model::DecisionInputFrame decision_frame{};
  AssociationQualityMetrics association_quality_metrics{};

  std::vector<float> signal_term_db;
  std::vector<float> speed_penalty_db;
  std::vector<float> detection_margin_db;
  std::vector<std::uint8_t> detection_succeeded;
  association::AssociationResult association_result;
  std::vector<std::uint64_t> association_keys;
  std::vector<tracking::MeasurementCovariance> measurement_covariances;
  std::vector<int> measurement_slots;
  std::vector<detection::ResolvedTargetGeometry> target_geometry;
};

/**
 * @brief 单周期执行器所需的运行时依赖视图。
 */
struct CycleExecutionRuntime {
  const SignalPipelineConfig* base_config{nullptr};
  const InternalSignalPipelineConfig* base_internal_config{nullptr};
  const extension::control::RadarControlProfile* control_profile{nullptr};
  association::DataAssociationEngine* association_engine{nullptr};
  tracking::TrackFilter* track_filter{nullptr};
  detection::SignalDetector* signal_detector{nullptr};
  signal::assembly::IDataOutputManager* output_manager{nullptr};
  tracking::ITrackLifecycleManager* auto_lifecycle_manager{nullptr};
  const std::vector<tracking::AssociationTrackSeed>* manual_association_seeds{nullptr};
  bool has_manual_association_seeds{false};
};

/**
 * @brief 周期准备阶段输出（后续 phase 的稳定输入）。
 */
struct CycleSetupPhaseOutput {
  const model::TargetFeatureList* input_state{nullptr};
  const extension::IEnvironmentService* environment{nullptr};
  SignalPipelineConfig runtime_config{};
  InternalSignalPipelineConfig internal_runtime_config{};
};

/**
 * @brief 环境采样阶段输出。
 */
struct EnvironmentPhaseOutput {
  environment::EnvironmentSnapshot environment_snapshot{};
  model::JammingSemantic dominant_jamming_semantic{model::JammingSemantic::kNone};
  float jamming_severity{0.0f};
};

/**
 * @brief 探测阶段输出视图。
 */
struct DetectionPhaseOutput {
  const std::vector<std::uint8_t>* detection_succeeded{nullptr};
  const std::vector<float>* detection_margin_db{nullptr};
  const std::vector<tracking::MeasurementCovariance>* measurement_covariances{nullptr};
  const std::vector<detection::ResolvedTargetGeometry>* target_geometry{nullptr};
};

/**
 * @brief 关联阶段输出视图。
 */
struct AssociationPhaseOutput {
  const association::AssociationResult* association_result{nullptr};
  const std::vector<std::uint64_t>* association_keys{nullptr};
};

/**
 * @brief 量测构建阶段输出视图。
 */
struct MeasurementBuildPhaseOutput {
  const std::vector<int>* measurement_slots{nullptr};
  const std::vector<tracking::TrackMeasurement>* track_measurements{nullptr};
};

/**
 * @brief 执行 SignalPipeline 单周期全流程。
 * @param input_state 当前周期输入目标状态。
 * @param environment 当前环境服务。
 * @param cycle_index 当前周期号。
 * @param batch_id 当前批次号。
 * @param runtime 运行时依赖视图。
 * @param cycle_scratch 单周期可复用缓冲区。
 */
void ExecuteCycle(const model::TargetFeatureList& input_state,
                  const extension::IEnvironmentService& environment, std::uint32_t cycle_index,
                  std::uint64_t batch_id, const CycleExecutionRuntime& runtime,
                  CycleExecutionScratch* cycle_scratch);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
