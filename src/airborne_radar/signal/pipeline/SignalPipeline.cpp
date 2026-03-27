#include "airborne_radar/signal/pipeline/SignalPipeline.h"


#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/core/output/DataOutputManager.h"
#include "airborne_radar/core/output/IDataOutputManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/AssociationExecutionSupport.h"
#include "airborne_radar/signal/pipeline/ContextBindingSupport.h"
#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/DecisionFrameBuilders.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/signal/pipeline/OutputAssemblySupport.h"
#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"
#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"
#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {
/**
 * @brief 单周期缓存上下文。
 */
struct SignalCycleContext {
  const common::TargetFeatureList* input_state{nullptr};
  const environment::IEnvironmentService* environment{nullptr};
  SignalPipelineConfig runtime_config{};
  common::TargetFeatureList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;
  common::DecisionInputFrame decision_frame{};
  AssociationQualityMetrics association_quality_metrics{};

  environment::EnvironmentSnapshot environment_snapshot{};
  common::JammingSemantic dominant_jamming_semantic{common::JammingSemantic::kNone}; /**< 当前周期主导干扰语义（SampleEnvironment 后有效） */
  float jamming_severity{0.0f};                                                      /**< 当前周期轨迹级残余干扰强度（SampleEnvironment 后有效） */

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

}  // namespace
/**
 * @brief SignalPipeline 私有实现。
 */
struct SignalPipeline::Impl {
  /**
   * @brief 使用顶层配置构造私有实现。
   * @param initial_config 顶层配置。
   */
  explicit Impl(SignalPipelineConfig initial_config)
      : config(std::move(initial_config)),
        association_engine(internal::SignalComponentFactory::BuildAssociationConfig(config)),
        track_filter(internal::SignalComponentFactory::BuildTrackFilterConfig(config)),
        output_manager_(std::unique_ptr<core::output::IDataOutputManager>(
            new core::output::DataOutputManager())) {
    RebuildOwnedComponents();
  }
  /**
   * @brief 执行一次信号处理周期。
   * @param input_state 输入目标列表。
   * @param environment 环境服务。
   * @return 输出目标列表。
   */
  SignalCycleResult RunCycle(const common::TargetFeatureList& input_state,
                             const environment::IEnvironmentService& environment) {
    PrepareCycleContext(input_state, environment);
    SampleEnvironment();
    PrepareAssociationSeeds();
    RunDetection();
    RunAssociation();
    BuildTrackMeasurements();
    ApplyTrackFilter();
    CollectOutputs();
    SignalCycleResult result;
    result.updated_features = cached_context.output_state;
    result.decision_frame = cached_context.decision_frame;
    result.association_quality_metrics = cached_context.association_quality_metrics;
    ++cycle_index_;
    ++batch_id_;
    return result;
  }
  /**
   * @brief 获取最近一次处理周期导出的跟踪量测。
   * @return 跟踪量测列表。
   */
  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const {
    return cached_context.track_measurements;
  }
  /**
   * @brief 获取最近一次处理周期的关联质量观测指标。
   * @return 关联质量观测指标。
   */
  AssociationQualityMetrics GetLastAssociationQualityMetrics() const {
    return cached_context.association_quality_metrics;
  }
  /**
   * @brief 设置本周期关联阶段应使用的轨迹种子。
   * @param seeds 外部种子。
   */
  void SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
    has_manual_association_seeds_ = true;
    manual_association_seeds_ = seeds;
    association_engine.SetAssociationSeeds(manual_association_seeds_);
  }
  /**
   * @brief 清理外部 seeds 状态并恢复无先验模式。
   */
  void ResetAssociationSeedModeToStateless() {
    has_manual_association_seeds_ = false;
    manual_association_seeds_.clear();
    association_engine.ResetAssociationSeedModeToStateless();
  }
  /**
   * @brief 按当前配置自动装配生命周期管理器。
   * @return 若未启用则返回空指针。
   */
  std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManager() const {
    return internal::CreateAutoLifecycleManagerForRuntimeConfig(BuildRuntimeConfig());
  }
  /**
   * @brief 更新顶层配置。
   * @param new_config 新配置。
   */
  void UpdateConfig(SignalPipelineConfig new_config) {
    config = std::move(new_config);
    internal::SyncAssociationAndTrackFilterConfigs(config, &association_engine, &track_filter);
    RebuildOwnedComponents();
  }
  /**
   * @brief 更新当前平台姿态。
   * @param platform_attitude_deg 平台姿态角。
   */
  void UpdatePlatformAttitude(const common::PlatformAttitudeDeg& platform_attitude_deg) {
    config.beam_control.platform_attitude_deg = platform_attitude_deg;
  }
  /**
   * @brief 获取当前平台姿态。
   * @return 当前缓存的平台姿态角。
   */
  common::PlatformAttitudeDeg GetPlatformAttitude() const {
    return config.beam_control.platform_attitude_deg;
  }
  /**
   * @brief 构建本周期生效的运行时配置。
   */
  SignalPipelineConfig BuildRuntimeConfig() const {
    return internal::BuildRuntimeConfigFromControlProfile(config, control_profile_);
  }
  /**
   * @brief 更新当前控制真值。
   */
  void SetControlProfile(const common::RadarControlProfile& control_profile) {
    control_profile_ = control_profile;
  }
  /**
   * @brief 获取当前控制真值。
   */
  common::RadarControlProfile GetControlProfile() const { return control_profile_; }
  /**
   * @brief 初始化单周期缓存。
   * @param input_state 输入目标列表。
   * @param environment 环境服务。
   */
  void PrepareCycleContext(const common::TargetFeatureList& input_state,
                           const environment::IEnvironmentService& environment) {
    cached_context.input_state = &input_state;
    cached_context.environment = &environment;
    cached_context.runtime_config = BuildRuntimeConfig();
    internal::ApplyScanScheduleToRuntimeConfig(cycle_index_, &cached_context.runtime_config);
    internal::CycleWorkspace cycle_workspace = BuildContextWorkspace();
    internal::ResetCycleWorkspace(input_state, cached_context.runtime_config, &cycle_workspace);
    internal::SyncAssociationAndTrackFilterConfigs(cached_context.runtime_config, &association_engine,
                                                   &track_filter);
  }
  /**
   * @brief 预配置本周期关联使用的先验种子。
   */
  void PrepareAssociationSeeds() {
    internal::PrepareAssociationSeedsForCycle(has_manual_association_seeds_,
                                              manual_association_seeds_,
                                              auto_lifecycle_manager_.get(),
                                              &association_engine);
  }
  /**
   * @brief 采样本周期环境快照。
   */
  void SampleEnvironment() {
    cached_context.environment_snapshot = cached_context.environment->SampleEnvironment();
    internal::ApplyEnvironmentJammingFactsToRuntimeConfig(
        cached_context.runtime_config.jamming_effects, control_profile_,
        cached_context.environment_snapshot, &cached_context.runtime_config);
    internal::RefreshMeasurementCovariances(
        cached_context.target_geometry.size(),
        cached_context.runtime_config.tracking.kalman_measurement_noise_std,
        &cached_context.measurement_covariances);
    // 干扰事实已修正 runtime_config（association.unassigned_cost / tracking 噪声系数），
    // 必须再次 Sync 才能让关联引擎和跟踪滤波器使用更新后的参数。
    internal::SyncAssociationAndTrackFilterConfigs(cached_context.runtime_config, &association_engine,
                                                   &track_filter);
    // 在环境快照与 runtime_config 均稳定后，统一解析干扰语义与强度，
    // 供后续 BuildTrackMeasurements / ApplyTrackFilter 直接复用。
    cached_context.dominant_jamming_semantic =
        internal::ResolveDominantJammingSemantic(control_profile_, cached_context.environment_snapshot);
    cached_context.jamming_severity =
        internal::ComputeTrackLevelJammingSeverity(control_profile_, cached_context.environment_snapshot);
  }
  /**
   * @brief 根据配置分派经验或物理探测通道，共享一次 buffer 构造。
   */
  void RunDetection() {
    internal::DetectionExecutionBuffers detection_buffers = internal::BuildDetectionExecutionBuffers(
        &cached_context.target_geometry, &cached_context.signal_term_db,
        &cached_context.speed_penalty_db, &cached_context.detection_margin_db,
        &cached_context.detection_succeeded, &cached_context.measurement_covariances);
    if (cached_context.runtime_config.detection.enable_physics_detection) {
      internal::RunPhysicalDetectionPass(*cached_context.input_state, cached_context.runtime_config,
                                         control_profile_, cached_context.environment_snapshot,
                                         signal_detector.get(), &detection_buffers);
    } else {
      internal::RunHeuristicDetectionPass(*cached_context.input_state, cached_context.runtime_config,
                                          control_profile_, cached_context.environment_snapshot,
                                          &detection_buffers);
    }
  }
  /**
   * @brief 执行位置关联。
   */
  void RunAssociation() {
    internal::RunAssociationPass(*cached_context.input_state,
                                 cached_context.detection_succeeded,
                                 cached_context.measurement_covariances,
                                 &association_engine,
                                 &cached_context.association_result,
                                 &cached_context.association_keys);
  }
  /**
   * @brief 构建跟踪量测骨架。
   */
  void BuildTrackMeasurements() {
    internal::TrackMeasurementBuildContext measurement_build_context =
        internal::BuildTrackMeasurementBuildContextBindings(
            cached_context.input_state, &cached_context.association_result,
            &cached_context.detection_succeeded, &cached_context.association_keys,
            &cached_context.detection_margin_db, &cached_context.target_geometry,
            &cached_context.measurement_covariances, cached_context.environment_snapshot.jamming_detected,
            cached_context.dominant_jamming_semantic, cached_context.jamming_severity, &cached_context.measurement_slots,
            &cached_context.track_measurements);
    internal::BuildTrackMeasurementsPass(measurement_build_context);
  }
  /**
   * @brief 执行跟踪滤波并补全量测动态属性。
   */
  void ApplyTrackFilter() {
    internal::TrackFilterApplyContext track_filter_apply_context =
        internal::BuildTrackFilterApplyContextBindings(
            cached_context.input_state, &cached_context.output_state, &cached_context.detection_succeeded,
            &cached_context.detection_margin_db, cached_context.environment_snapshot.jamming_detected,
            cached_context.dominant_jamming_semantic, cached_context.jamming_severity, &track_filter,
            &cached_context.measurement_slots, &cached_context.track_measurements);
    internal::ApplyTrackFilterPass(track_filter_apply_context);
  }
  /**
   * @brief 收尾当前周期输出。
   */
  void CollectOutputs() {
    internal::CollectCycleOutputs(
        control_profile_, cycle_index_, batch_id_, cached_context.runtime_config,
        cached_context.environment_snapshot, *cached_context.input_state, cached_context.output_state,
        cached_context.association_result, cached_context.track_measurements, output_manager_.get(),
        auto_lifecycle_manager_.get(), &cached_context.association_quality_metrics,
        &cached_context.decision_frame);
  }
  /**
   * @brief 将 cached_context 各成员地址封装为 CycleWorkspace 视图。
   * @return 指向当前 cached_context 各缓存字段的 workspace 绑定。
   */
  internal::CycleWorkspace BuildContextWorkspace() {
    return internal::BuildCycleWorkspaceBindings(
        &cached_context.output_state, &cached_context.decision_frame,
        &cached_context.association_quality_metrics, &cached_context.track_measurements,
        &cached_context.signal_term_db, &cached_context.speed_penalty_db,
        &cached_context.detection_margin_db, &cached_context.detection_succeeded,
        &cached_context.association_keys, &cached_context.measurement_slots,
        &cached_context.target_geometry, &cached_context.measurement_covariances,
        &cached_context.association_result);
  }
  /**
   * @brief 依据当前配置重建自持有组件。
   */
  void RebuildOwnedComponents() {
    internal::OwnedComponentSlots component_slots;
    component_slots.kalman_predictor = &kalman_predictor;
    component_slots.kalman_updater = &kalman_updater;
    component_slots.signal_detector = &signal_detector;
    component_slots.auto_lifecycle_manager = &auto_lifecycle_manager_;
    internal::RebuildOwnedComponentsForPipeline(config, control_profile_, &component_slots);
  }

  SignalPipelineConfig config{};
  common::RadarControlProfile control_profile_{};
  association::DataAssociationEngine association_engine{};
  tracking::TrackFilter track_filter{};
  std::unique_ptr<core::output::IDataOutputManager> output_manager_;
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater;
  std::unique_ptr<detection::SignalDetector> signal_detector;
  std::unique_ptr<tracking::ITrackLifecycleManager> auto_lifecycle_manager_;
  std::vector<tracking::AssociationTrackSeed> manual_association_seeds_;
  bool has_manual_association_seeds_{false};
  std::uint32_t cycle_index_{1};
  std::uint64_t batch_id_{1};
  SignalCycleContext cached_context{};
};

SignalPipeline::SignalPipeline(SignalPipelineConfig config)
    : impl_(std::unique_ptr<Impl>(new Impl(std::move(config)))) {}

SignalPipeline::~SignalPipeline() = default;

SignalCycleResult SignalPipeline::RunCycle(const common::TargetFeatureList& input_state,
                                           const environment::IEnvironmentService& environment) {
  return impl_->RunCycle(input_state, environment);
}

std::vector<tracking::TrackMeasurement> SignalPipeline::GetLastTrackMeasurements() const {
  return impl_->GetLastTrackMeasurements();
}

AssociationQualityMetrics SignalPipeline::GetLastAssociationQualityMetrics() const {
  return impl_->GetLastAssociationQualityMetrics();
}

void SignalPipeline::SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
  impl_->SetAssociationSeeds(seeds);
}

void SignalPipeline::ResetAssociationSeedModeToStateless() {
  impl_->ResetAssociationSeedModeToStateless();
}

std::unique_ptr<tracking::ITrackLifecycleManager> SignalPipeline::CreateAutoLifecycleManager()
    const {
  return impl_->CreateAutoLifecycleManager();
}

void SignalPipeline::UpdatePlatformAttitude(
    const common::PlatformAttitudeDeg& platform_attitude_deg) {
  impl_->UpdatePlatformAttitude(platform_attitude_deg);
}

common::PlatformAttitudeDeg SignalPipeline::GetPlatformAttitude() const {
  return impl_->GetPlatformAttitude();
}

void SignalPipeline::SetControlProfile(const common::RadarControlProfile& control_profile) {
  impl_->SetControlProfile(control_profile);
}

common::RadarControlProfile SignalPipeline::GetControlProfile() const {
  return impl_->GetControlProfile();
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  impl_->UpdateConfig(std::move(config));
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
