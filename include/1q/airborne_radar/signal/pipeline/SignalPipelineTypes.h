// Copyright 2026. All Rights Reserved.
//
// @file SignalPipelineTypes.h
// @brief 定义信号流水线公共配置与结果类型。

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_TYPES_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/detection/DetectionTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/// @brief 关联质量观测指标（Pipeline 对外公开版本）。
struct AssociationQualityMetrics {
  /// @brief 进入关联阶段的历史先验轨迹数。
  std::size_t prior_track_count{0};
  /// @brief 本周期探测成功并参与关联的量测数。
  std::size_t detection_count{0};
  /// @brief 命中已有轨迹的关联数。
  std::size_t matched_count{0};
  /// @brief 触发新建轨迹键的量测数。
  std::size_t new_track_count{0};
  /// @brief 未命中任何量测的历史轨迹数。
  std::size_t missed_track_count{0};
  /// @brief 命中率（matched_count / detection_count）。
  float match_rate{0.0f};
  /// @brief 新生率（new_track_count / detection_count）。
  float new_track_rate{0.0f};
  /// @brief 漏失率（missed_track_count / prior_track_count）。
  float missed_track_rate{0.0f};
  /// @brief 命中关联代价均值（仅统计 matches）。
  float mean_match_cost{0.0f};
  /// @brief 命中关联代价 P95（仅统计 matches）。
  float p95_match_cost{0.0f};
  /// @brief 当前周期关联质量对应的主导干扰摘要类型。
  common::JammingSemantic dominant_jamming_semantic{
      common::JammingSemantic::kNone};
  /// @brief 当前周期关联质量对应的残余干扰强度摘要，范围 [0, 1]。
  float jamming_severity{0.0f};
  /// @brief 当前周期的归一化关联压力，范围 [0, 1]。
  float association_stress{0.0f};
};

/// @brief TrackPoolThreadSafetyMode 定义对象池线程安全策略。
enum class TrackPoolThreadSafetyMode {
  /// @brief 默认无锁模式，适用于当前单线程生命周期更新。
  kSingleThreadNoLock = 0,
  /// @brief 使用全局互斥保护对象池接口。
  kMultiThreadGlobalLock
};

/// @brief ImmActivationPolicy 定义 IMM 多模型路径的激活策略。
enum class ImmActivationPolicy {
  /// @brief 所有轨迹都按当前 IMM 语义创建和使用多模型路径。
  kAllTracks = 0,
  /// @brief 仅已确认轨迹在再次命中时懒创建并启用 IMM。
  kConfirmedTracksOnly
};

/// @brief LifecycleConfig 定义轨迹状态机阈值配置。
struct LifecycleConfig {
  /// @brief 候选轨迹转已确认所需最小命中次数。
  std::uint32_t confirm_hits{3};
  /// @brief 已确认轨迹转丢失前允许的最大连续失配次数。
  std::uint32_t max_miss_before_lost{2};
  /// @brief 丢失轨迹可保留的最大周期数，超出则回收。
  std::uint32_t max_lost_cycles{5};
  /// @brief IMM 激活策略。
  ImmActivationPolicy imm_activation_policy{
      ImmActivationPolicy::kConfirmedTracksOnly};
  /// @brief 对象池线程安全策略。
  TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      TrackPoolThreadSafetyMode::kSingleThreadNoLock};
};

/// @brief SignalDetectionConfig 描述信号探测域配置。
struct SignalDetectionConfig {
  bool enable_physics_detection{false};
  detection::RadarSystemConfig radar_system{};
  float min_detection_margin_db{-2.0f};
  int pulse_count{10};
  bool coherent_integration{true};
};

/// @brief SignalBeamControlConfig 描述波束控制域配置。
struct SignalBeamControlConfig {
  common::RadarOrientationConfig radar_orientation{};
  common::PlatformAttitudeDeg platform_attitude_deg{};
};

/// @brief SignalAssociationConfig 描述信号层关联域配置。
struct SignalAssociationConfig {
  float unassigned_cost{9.0f};
};

/// @brief SignalTrackingConfig 描述信号层跟踪域配置。
struct SignalTrackingConfig {
  bool enable_kalman_filter{true};
  float kalman_noise_diff_coeff{1.0f};
  float kalman_measurement_noise_std{10.0f};
  float speed_decay_ratio_on_loss{0.90f};
  float rcs_decay_ratio_on_loss{0.85f};
  float jamming_acceleration_penalty{0.5f};
  float stable_acceleration_gain{0.05f};
};

/// @brief SignalLifecycleConfig 描述生命周期域配置。
struct SignalLifecycleConfig {
  bool enable_auto_lifecycle_manager{false};
  LifecycleConfig lifecycle_config{};
  bool enable_imm_lifecycle{false};
  std::vector<float> imm_model_noise_diff_coeffs;
  std::vector<float> imm_initial_weights;
  std::vector<float> imm_transition_probability;
  std::size_t lifecycle_track_pool_initial_chunk{64};
  std::size_t lifecycle_track_pool_max_chunks{256};
};

/// @brief SignalPipelineConfig 描述信号处理流水线顶层配置。
struct SignalPipelineConfig {
  SignalDetectionConfig detection{};
  SignalBeamControlConfig beam_control{};
  SignalAssociationConfig association{};
  SignalTrackingConfig tracking{};
  SignalLifecycleConfig lifecycle{};
};

/// @brief SignalCycleResult 描述信号流水线单周期的稳定输出。
struct SignalCycleResult {
  /// @brief 当前周期更新后的目标特征列表。
  common::TargetFeatureList updated_features{};
  /// @brief 当前周期决策输入帧。
  common::DecisionInputFrame decision_frame{};
  /// @brief 当前周期关联质量观测指标。
  AssociationQualityMetrics association_quality_metrics{};
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_TYPES_H_
