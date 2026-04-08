/**
 * @file CycleExecutor.h
 * @brief 定义 SignalPipeline 单周期执行器。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/extension/IEnvironmentService.h"
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
 * @note 所有字段均为非 owning 引用视图；其中 association/track_filter/lifecycle_manager
 *       持有跨周期状态，signal_detector 仅表示当前配置下的执行组件。
 */
struct CycleExecutionRuntime {
  CycleExecutionRuntime(const SignalPipelineConfig& base_config,
                        const InternalSignalPipelineConfig& base_internal_config,
                        const extension::control::RadarControlProfile& control_profile,
                        association::DataAssociationEngine& association_engine,
                        tracking::TrackFilter& track_filter,
                        tracking::ITrackLifecycleManager& auto_lifecycle_manager,
                        detection::SignalDetector* signal_detector,
                        const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds,
                        bool has_manual_association_seeds)
      : base_config(base_config),
        base_internal_config(base_internal_config),
        control_profile(control_profile),
        association_engine(association_engine),
        track_filter(track_filter),
        signal_detector(signal_detector),
        auto_lifecycle_manager(auto_lifecycle_manager),
        manual_association_seeds(manual_association_seeds),
        has_manual_association_seeds(has_manual_association_seeds) {}

  const SignalPipelineConfig& base_config;
  const InternalSignalPipelineConfig& base_internal_config;
  const extension::control::RadarControlProfile& control_profile;
  association::DataAssociationEngine& association_engine;
  tracking::TrackFilter& track_filter;
  detection::SignalDetector* signal_detector;
  tracking::ITrackLifecycleManager& auto_lifecycle_manager;
  const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds;
  bool has_manual_association_seeds{false};
};

/**
 * @brief 周期主链路各 phase 共享的稳定输入契约。
 * @details 该结构在 setup 阶段一次构造，之后仅由 environment phase 对
 *          runtime_config/internal_runtime_config 做显式更新。
 */
struct CycleExecutionContract {
  CycleExecutionContract(const model::TargetFeatureList& input_state,
                         const extension::IEnvironmentService& environment,
                         std::uint32_t cycle_index, std::uint64_t batch_id,
                         SignalPipelineConfig runtime_config,
                         InternalSignalPipelineConfig internal_runtime_config)
      : input_state(input_state),
        environment(environment),
        cycle_index(cycle_index),
        batch_id(batch_id),
        runtime_config(std::move(runtime_config)),
        internal_runtime_config(std::move(internal_runtime_config)) {}

  const model::TargetFeatureList& input_state;
  const extension::IEnvironmentService& environment;
  std::uint32_t cycle_index{0U};
  std::uint64_t batch_id{0U};
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
  DetectionPhaseOutput(const std::vector<std::uint8_t>& detection_succeeded,
                       const std::vector<float>& detection_margin_db,
                       const std::vector<tracking::MeasurementCovariance>& measurement_covariances,
                       const std::vector<detection::ResolvedTargetGeometry>& target_geometry)
      : detection_succeeded(detection_succeeded),
        detection_margin_db(detection_margin_db),
        measurement_covariances(measurement_covariances),
        target_geometry(target_geometry) {}

  const std::vector<std::uint8_t>& detection_succeeded;
  const std::vector<float>& detection_margin_db;
  const std::vector<tracking::MeasurementCovariance>& measurement_covariances;
  const std::vector<detection::ResolvedTargetGeometry>& target_geometry;
};

/**
 * @brief 关联阶段输出视图。
 */
struct AssociationPhaseOutput {
  AssociationPhaseOutput(const association::AssociationResult& association_result,
                         const std::vector<std::uint64_t>& association_keys)
      : association_result(association_result), association_keys(association_keys) {}

  const association::AssociationResult& association_result;
  const std::vector<std::uint64_t>& association_keys;
};

/**
 * @brief 量测构建阶段输出视图。
 */
struct MeasurementBuildPhaseOutput {
  MeasurementBuildPhaseOutput(const std::vector<int>& measurement_slots,
                              const std::vector<tracking::TrackMeasurement>& track_measurements)
      : measurement_slots(measurement_slots), track_measurements(track_measurements) {}

  const std::vector<int>& measurement_slots;
  const std::vector<tracking::TrackMeasurement>& track_measurements;
};

/**
 * @brief 执行 SignalPipeline 单周期全流程。
 * @param input_state 当前周期输入目标状态。
 * @param environment 当前环境服务。
 * @param cycle_index 当前周期号。
 * @param batch_id 当前批次号。
 * @param runtime 运行时依赖视图。
 * @param cycle_scratch 单周期可复用缓冲区。
 * @return 周期准备与执行全部成功时返回 true；若运行时同步失败则返回 false。
 */
bool ExecuteCycle(const model::TargetFeatureList& input_state,
                  const extension::IEnvironmentService& environment, std::uint32_t cycle_index,
                  std::uint64_t batch_id, const CycleExecutionRuntime& runtime,
                  CycleExecutionScratch& cycle_scratch);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
