/**
 * @file CycleExecutor.h
 * @brief 定义 SignalPipeline 单周期执行器与相关数据结构。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "airborne_radar/environment/EnvironmentTypes.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/SignalCycleAnnotations.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 单周期执行过程中所有中间产物和最终输出的共享暂存区。
 *
 * 各阶段函数直接读写此结构，不再通过 PhaseOutput 传递引用。
 */
struct CycleExecutionScratch {
  // 最终输出
  session::ArSceneTargetList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;
  session::DecisionInputFrame decision_frame{};
  AssociationQualityMetrics association_quality_metrics{};

  // 检测阶段中间数据
  std::vector<float> signal_term_db;
  std::vector<float> speed_penalty_db;
  std::vector<float> detection_margin_db;
  std::vector<std::uint8_t> detection_succeeded;
  std::vector<detection::ResolvedTargetGeometry> target_geometry;
  std::vector<tracking::MeasurementCovariance> measurement_covariances;

  // 关联阶段中间数据
  association::AssociationResult association_result;
  std::vector<std::uint64_t> association_keys;

  // 欺骗候选关联阶段中间数据
  std::vector<std::uint64_t> deception_candidate_keys;

  // 量测构建阶段中间数据
  std::vector<int> measurement_slots;
};

/**
 * @brief 单周期执行所需的引擎、配置与状态引用集合。
 *
 * 所有成员为对长期存活对象的外部引用，调用方需保证生命周期覆盖整个 ExecuteCycle。
 */
struct CycleExecutionRuntime {
  CycleExecutionRuntime(const ExecutionConfig& base_config,
                        const session::ArControlProfile& control_profile,
                        association::DataAssociationEngine& association_engine,
                        tracking::TrackFilter& track_filter,
                        tracking::ITrackLifecycleManager& auto_lifecycle_manager,
                        detection::SignalDetector* signal_detector,
                        const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds,
                        bool has_manual_association_seeds)
      : base_config(base_config),
        control_profile(control_profile),
        association_engine(association_engine),
        track_filter(track_filter),
        signal_detector(signal_detector),
        auto_lifecycle_manager(auto_lifecycle_manager),
        manual_association_seeds(manual_association_seeds),
        has_manual_association_seeds(has_manual_association_seeds) {}

  const ExecutionConfig& base_config;
  const session::ArControlProfile& control_profile;
  association::DataAssociationEngine& association_engine;
  tracking::TrackFilter& track_filter;
  detection::SignalDetector* signal_detector;
  tracking::ITrackLifecycleManager& auto_lifecycle_manager;
  const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds;
  bool has_manual_association_seeds{false};
};

/**
 * @brief 单周期执行的输入上下文（场景输入、环境、周期编号与运行时配置）。
 */
struct CycleExecutionContext {
  CycleExecutionContext(
      const session::ArSceneTargetList& input_state,
      const session::EnvironmentSnapshot& environment_snapshot, std::uint32_t cycle_index,
      std::uint64_t batch_id, ExecutionConfig runtime_config, float platform_altitude_m,
      const RfV2DetectionContext* rf_v2_detection_context = nullptr,
      const SignalCycleAnnotations* annotations = nullptr)
      : input_state(input_state),
        environment_snapshot(environment_snapshot),
        cycle_index(cycle_index),
        batch_id(batch_id),
        platform_altitude_m(platform_altitude_m),
        runtime_config(std::move(runtime_config)),
        rf_v2_detection_context(rf_v2_detection_context),
        annotations(annotations) {}

  const session::ArSceneTargetList& input_state;
  const session::EnvironmentSnapshot& environment_snapshot;
  std::uint32_t cycle_index{0U};
  std::uint64_t batch_id{0U};
  float platform_altitude_m{0.0f};
  ExecutionConfig runtime_config{};
  // 以下裸指针表达"可选的非拥有引用"：nullptr 表示该周期无此输入，非空时指向调用方
  // 拥有的、生命周期覆盖整个 ExecuteCycle 的对象。ExecuteCycle 内只读访问，不释放。
  const RfV2DetectionContext* rf_v2_detection_context{nullptr};
  const SignalCycleAnnotations* annotations{nullptr};
};

/**
 * @brief 执行一个完整的处理周期（环境→探测→关联→量测→滤波→汇总输出）。
 * @param[in,out] context 周期输入上下文；其 runtime_config 在执行过程中可能被各阶段读取。
 * @param[in] runtime 引擎、配置与状态引用集合。
 * @param[in,out] cycle_scratch 周期暂存区，承载各阶段中间产物与最终输出。
 * @return 环境阶段失败返回 false；正常完成返回 true。
 */
bool ExecuteCycle(CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                  CycleExecutionScratch& cycle_scratch);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
