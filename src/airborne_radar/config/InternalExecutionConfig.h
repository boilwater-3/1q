/**
 * @file InternalExecutionConfig.h
 * @brief 定义唯一内部执行配置真值及其 domain 子配置。
 *
 * 本类型合并了 legacy PipelineConfig（四域公开参数 + orientation 运行态）
 * 与 InternalPipelineConfig（信号链路内部扩展参数）的全部运行期字段，
 * 作为 signal/runtime 各模块的唯一配置消费入口。
 *
 * InternalExecutionConfig 按 domain 拆分为四个子配置：
 * - DetectionExecutionConfig：检测与波束调度
 * - AssociationExecutionConfig：数据关联
 * - TrackingExecutionConfig：航迹滤波与卡尔曼
 * - LifecycleExecutionConfig：航迹生命周期与 IMM
 *
 * 跨域共享的 ControlProfileEffectsConfig 保留在顶层。
 *
 * @note 环境域（ArEnvironmentConfig）不包含在本类型中。环境观测数据通过
 *       独立路径传递给 EnvironmentService 管理，不经过 MapSessionToExecution()。
 *       设计意图：environment 仅携带自然环境事实（大气物理、植被散射），
 *       不含算法调参字段，因此无需参与执行配置的合并映射。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_EXECUTION_INTERNAL_EXECUTION_CONFIG_H_
#define AIRBORNE_RADAR_SRC_CONFIG_EXECUTION_INTERNAL_EXECUTION_CONFIG_H_

#include <cstddef>
#include <vector>

#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArMissionConfig.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"

namespace airborne_radar {
namespace config {
namespace execution {

// ---------------------------------------------------------------------------
// 固定运行时常量（不通过四域配置暴露，由算法内部决定）
// ---------------------------------------------------------------------------

/** Track pool 默认初始分块数。 */
inline constexpr std::size_t kDefaultTrackPoolInitialChunk{64};

/** Track pool 默认最大分块数。 */
inline constexpr std::size_t kDefaultTrackPoolMaxChunks{256};

/** IMM 默认激活策略：仅已确认航迹参与 IMM 融合。 */
inline constexpr signal::tracking::ImmActivationPolicy kDefaultImmActivationPolicy{
    signal::tracking::ImmActivationPolicy::kConfirmedTracksOnly};

/** Track pool 默认线程安全模式：单线程无锁。 */
inline constexpr signal::tracking::TrackPoolThreadSafetyMode kDefaultTrackPoolThreadSafetyMode{
    signal::tracking::TrackPoolThreadSafetyMode::kSingleThreadNoLock};

// ---------------------------------------------------------------------------
// 子配置类型
// ---------------------------------------------------------------------------

/**
 * @brief 控制策略 (control profile) 对应的天线/波束增益效果参数 (POD)。
 *
 * 描述在自适应波束、LPI 等控制策略下，旁瓣电平抑制量、波束宽度缩放、
 * 信号增益补偿，以及 ECCM 速度/RCS 衰减奖励等量值，供 ControlProfileEffects
 * 模块在评估控制收益时使用。
 */
struct ControlProfileEffectsConfig {
  float sidelobe_level_reduction_db{6.0f};
  float adaptive_beam_gain_boost_db{2.0f};
  float adaptive_beamwidth_scale{0.60f};
  float lpi_beamwidth_scale{0.75f};
};

/** 控制策略天线/波束增益效果默认参数。 */
inline constexpr ControlProfileEffectsConfig kDefaultControlProfileEffects{};

/**
 * @brief 检测与波束调度相关执行配置。
 */
struct DetectionExecutionConfig {
  BeamControlConfig beam_control{};
  config::ArOrientationConfig orientation{};
  config::PlatformAttitudeDeg platform_attitude_deg{};
  engineering::DetectionConfig engineering{};
};

struct AssociationPolicyConfig {
  /** 归一化距离关联代价，由 distance_gate_sigma² 映射而来。 */
  float unassigned_cost{9.0f};
};

/**
 * @brief 数据关联相关执行配置。
 */
struct AssociationExecutionConfig {
  AssociationPolicyConfig policy{};
};

/**
 * @brief 航迹滤波与卡尔曼相关执行配置。
 */
struct TrackingExecutionConfig {
  TrackingConfig policy{};
  engineering::TrackingConfig engineering{};
  /** Kalman 过程噪声差异系数，由 ArPolicyConfig::TrackingConfig 映射而来。 */
  float kalman_noise_diff_coeff{1.0f};
};

/**
 * @brief 航迹生命周期与 IMM 相关执行配置。
 */
struct LifecycleExecutionConfig {
  LifecycleConfig policy{};
  engineering::LifecycleRuntimeConfig engineering{};
  /** IMM 激活策略（固定默认值，不通过四域配置暴露）。 */
  signal::tracking::ImmActivationPolicy imm_activation_policy{kDefaultImmActivationPolicy};
  /** Track pool 线程安全模式（固定默认值，不通过四域配置暴露）。 */
  signal::tracking::TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      kDefaultTrackPoolThreadSafetyMode};
  std::vector<float> imm_model_noise_diff_coeffs;
  std::vector<float> imm_initial_weights;
  std::vector<float> imm_transition_probability;
  /** Track pool 初始分块数（固定默认值，不通过四域配置暴露）。 */
  std::size_t track_pool_initial_chunk{kDefaultTrackPoolInitialChunk};
  /** Track pool 最大分块数（固定默认值，不通过四域配置暴露）。 */
  std::size_t track_pool_max_chunks{kDefaultTrackPoolMaxChunks};
};

/**
 * @brief 唯一内部执行配置真值。
 *
 * 按 domain 将字段组织为四个子配置，外加跨域共享的控制效果配置。
 * 各 pipeline phase 函数应只接收其所需的子配置引用，而非整个 InternalExecutionConfig。
 *
 * @note enable_anti_vgpo_acceleration_bound 和 enable_anti_false_target_discrimination
 *       由 ControlProfileEffects 每周期从 ArControlProfile 写入，不经过映射层。
 *       anti_vgpo_max_acceleration_mps2 由 DecisionControlConfig 映射而来。
 */
struct InternalExecutionConfig {
  bool sensor_enabled{true};                        /**< 设备开关机状态 */
  config::DecisionControlConfig decision_control{}; /**< 跨周期 LPI/ECCM 保持与冷却策略 */
  bool enable_anti_vgpo_acceleration_bound{false};  /**< 加速度限幅对抗 VGPO（运行时由 ControlProfile 写入） */
  double anti_vgpo_max_acceleration_mps2{100.0};   /**< 加速度限幅阈值（m/s²），由 DecisionControlConfig 映射 */
  bool enable_anti_false_target_discrimination{false}; /**< 假目标鉴别（运行时由 ControlProfile 写入） */
  DetectionExecutionConfig detection{};
  AssociationExecutionConfig association{};
  TrackingExecutionConfig tracking{};
  LifecycleExecutionConfig lifecycle{};
  /** 控制策略增益效果参数（固定默认值，不通过四域配置暴露）。 */
  ControlProfileEffectsConfig control_profile_effects{kDefaultControlProfileEffects};
};

}  // namespace execution
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_EXECUTION_INTERNAL_EXECUTION_CONFIG_H_
