/**
 * @file CycleExecutor.h
 * @brief 定义 SignalPipeline 单周期执行器。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "airborne_radar/signal/output/IDataOutputManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 单周期执行上下文缓存。
 */
struct CycleExecutionContext {
  const common::TargetFeatureList* input_state{nullptr};
  const environment::IEnvironmentService* environment{nullptr};
  SignalPipelineConfig runtime_config{};
  common::TargetFeatureList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;
  common::DecisionInputFrame decision_frame{};
  AssociationQualityMetrics association_quality_metrics{};

  environment::EnvironmentSnapshot environment_snapshot{};
  common::JammingSemantic dominant_jamming_semantic{
      common::JammingSemantic::kNone}; /**< 当前周期主导干扰语义（SampleEnvironment 后有效） */
  float jamming_severity{0.0f}; /**< 当前周期轨迹级残余干扰强度（SampleEnvironment 后有效） */

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
  const common::RadarControlProfile* control_profile{nullptr};
  association::DataAssociationEngine* association_engine{nullptr};
  tracking::TrackFilter* track_filter{nullptr};
  detection::SignalDetector* signal_detector{nullptr};
  signal::output::IDataOutputManager* output_manager{nullptr};
  tracking::ITrackLifecycleManager* auto_lifecycle_manager{nullptr};
  const std::vector<tracking::AssociationTrackSeed>* manual_association_seeds{nullptr};
  bool has_manual_association_seeds{false};
};

/**
 * @brief 执行 SignalPipeline 单周期全流程。
 * @param input_state 当前周期输入目标状态。
 * @param environment 当前环境服务。
 * @param cycle_index 当前周期号。
 * @param batch_id 当前批次号。
 * @param runtime 运行时依赖视图。
 * @param cycle_context 单周期缓存上下文。
 */
void ExecuteCycle(const common::TargetFeatureList& input_state,
                  const environment::IEnvironmentService& environment,
                  std::uint32_t cycle_index, std::uint64_t batch_id,
                  const CycleExecutionRuntime& runtime, CycleExecutionContext* cycle_context);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
