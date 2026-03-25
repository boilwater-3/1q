/**
 * @file SignalPipelineTypes.h
 * @brief 定义信号流水线公共配置与结果类型。
 */

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

/**
 * @brief 关联质量观测指标（Pipeline 对外公开版本）。
 */
struct AssociationQualityMetrics {
  std::size_t prior_track_count{0};  /**< 进入关联阶段的历史先验轨迹数 */
  std::size_t detection_count{0};    /**< 本周期探测成功并参与关联的量测数 */
  std::size_t matched_count{0};      /**< 命中已有轨迹的关联数 */
  std::size_t new_track_count{0};    /**< 触发新建轨迹键的量测数 */
  std::size_t missed_track_count{0}; /**< 未命中任何量测的历史轨迹数 */
  float match_rate{0.0f};            /**< 命中率（matched_count / detection_count） */
  float new_track_rate{0.0f};        /**< 新生率（new_track_count / detection_count） */
  float missed_track_rate{0.0f};     /**< 漏失率（missed_track_count / prior_track_count） */
  float mean_match_cost{0.0f};       /**< 命中关联代价均值（仅统计 matches） */
  float p95_match_cost{0.0f};        /**< 命中关联代价 P95（仅统计 matches） */
  common::JammingSemantic dominant_jamming_semantic{
      common::JammingSemantic::kNone}; /**< 当前周期关联质量对应的主导干扰摘要类型 */
  float jamming_severity{0.0f};        /**< 当前周期关联质量对应的残余干扰强度摘要，范围 [0, 1] */
  float association_stress{0.0f};      /**< 当前周期的归一化关联压力，范围 [0, 1] */
};

/**
 * @brief TrackPoolThreadSafetyMode 定义对象池线程安全策略。
 */
enum class TrackPoolThreadSafetyMode {
  kSingleThreadNoLock = 0, /**< 默认无锁模式，适用于当前单线程生命周期更新 */
  kMultiThreadGlobalLock   /**< 使用全局互斥保护对象池接口 */
};

/**
 * @brief ImmActivationPolicy 定义 IMM 多模型路径的激活策略。
 */
enum class ImmActivationPolicy {
  kAllTracks = 0,      /**< 所有轨迹都按当前 IMM 语义创建和使用多模型路径 */
  kConfirmedTracksOnly /**< 仅已确认轨迹在再次命中时懒创建并启用 IMM */
};

/**
 * @brief LifecycleConfig 定义轨迹状态机阈值配置。
 */
struct LifecycleConfig {
  std::uint32_t confirm_hits{3};         /**< 候选轨迹转已确认所需最小命中次数 */
  std::uint32_t max_miss_before_lost{2}; /**< 已确认轨迹转丢失前允许的最大连续失配次数 */
  std::uint32_t max_lost_cycles{5};      /**< 丢失轨迹可保留的最大周期数，超出则回收 */
  ImmActivationPolicy imm_activation_policy{
      ImmActivationPolicy::kConfirmedTracksOnly}; /**< IMM 激活策略 */
  TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      TrackPoolThreadSafetyMode::kSingleThreadNoLock}; /**< 对象池线程安全策略 */
};

/**
 * @brief SignalDetectionConfig 描述信号探测域配置。
 */
struct SignalDetectionConfig {
  bool enable_physics_detection{false};        /**< 是否启用物理层检测 */
  detection::RadarSystemConfig radar_system{}; /**< 雷达系统配置 */
  float min_detection_margin_db{-2.0f};        /**< 最小检测裕量（dB） */
  int pulse_count{10};                         /**< 脉冲数 */
  bool coherent_integration{true};             /**< 是否启用相干积累 */
};

/**
 * @brief SignalBeamControlConfig 描述波束控制域配置。
 */
struct SignalBeamControlConfig {
  common::RadarOrientationConfig radar_orientation{};  /**< 雷达方向配置 */
  common::PlatformAttitudeDeg platform_attitude_deg{}; /**< 平台姿态角（单位：度） */
};

/**
 * @brief SignalAssociationConfig 描述信号层关联域配置。
 */
struct SignalAssociationConfig {
  float unassigned_cost{9.0f}; /**< 未分配代价 */
};

/**
 * @brief SignalTrackingConfig 描述信号层跟踪域配置。
 */
struct SignalTrackingConfig {
  bool enable_kalman_filter{true};           /**< 是否启用卡尔曼滤波器 */
  float kalman_noise_diff_coeff{1.0f};       /**< 卡尔曼噪声扩散系数 */
  float kalman_measurement_noise_std{10.0f}; /**< 卡尔曼测量噪声标准差 */
  float speed_decay_ratio_on_loss{0.90f};    /**< 丢失时速度衰减比例 */
  float rcs_decay_ratio_on_loss{0.85f};      /**< 丢失时 RCS 衰减比例 */
};

/**
 * @brief SignalLifecycleConfig 描述生命周期域配置。
 */
struct SignalLifecycleConfig {
  bool enable_auto_lifecycle_manager{false};          /**< 是否启用自动生命周期管理 */
  LifecycleConfig lifecycle_config{};                 /**< 生命周期配置 */
  bool enable_imm_lifecycle{false};                   /**< 是否启用 IMM 生命周期管理 */
  std::vector<float> imm_model_noise_diff_coeffs;     /**< IMM 模型噪声扩散系数列表 */
  std::vector<float> imm_initial_weights;             /**< IMM 初始权重列表 */
  std::vector<float> imm_transition_probability;      /**< IMM 状态转移概率列表 */
  std::size_t lifecycle_track_pool_initial_chunk{64}; /**< 轨迹池初始块大小 */
  std::size_t lifecycle_track_pool_max_chunks{256};   /**< 轨迹池最大块数 */
};

/**
 * @brief SignalPipelineConfig 描述信号处理流水线顶层配置。
 */
struct SignalPipelineConfig {
  SignalDetectionConfig detection{};      /**< 探测配置 */
  SignalBeamControlConfig beam_control{}; /**< 波束控制配置 */
  SignalAssociationConfig association{};  /**< 关联配置 */
  SignalTrackingConfig tracking{};        /**< 跟踪配置 */
  SignalLifecycleConfig lifecycle{};      /**< 生命周期配置 */
};

/**
 * @brief SignalCycleResult 描述信号流水线单周期的稳定输出。
 */
struct SignalCycleResult {
  common::TargetFeatureList updated_features{};            /**< 当前周期更新后的目标特征列表 */
  common::DecisionInputFrame decision_frame{};             /**< 当前周期决策输入帧 */
  AssociationQualityMetrics association_quality_metrics{}; /**< 当前周期关联质量观测指标 */
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_TYPES_H_
