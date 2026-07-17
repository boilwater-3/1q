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
 * 跨域共享的 JammingEffectsConfig 与 ControlProfileEffectsConfig
 * 保留在顶层。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_EXECUTION_INTERNAL_EXECUTION_CONFIG_H_
#define AIRBORNE_RADAR_SRC_CONFIG_EXECUTION_INTERNAL_EXECUTION_CONFIG_H_

#include <cstddef>
#include <vector>

#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArMissionConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"

namespace airborne_radar {
namespace config {
namespace execution {

/**
 * @brief ECM 干扰效果调参集合 (POD)。
 *
 * 汇总各类干扰源 (噪声/欺骗/转发/未知) 下的启发式置信度门限、惩罚权重，
 * 以及在 association/tracking/measurement 各环节施加的渐进式缩放步长与
 * 协方差膨胀步长，供 JammingEffects 模块逐周期计算干扰退化量使用。
 * @note 所有字段均为经验标定参数，直接修改会改变信号链路抗干扰行为。
 */
struct JammingEffectsConfig {
  float confidence_weight_min{0.25f};
  float heuristic_base_penalty_db{0.8f};
  float heuristic_power_penalty_slope{0.18f};
  float heuristic_noise_sidelobe_penalty{0.9f};
  float heuristic_noise_frontlobe_ratio{0.4f};
  float heuristic_noise_js_slope{0.4f};
  float heuristic_deception_freq_penalty{1.8f};
  float heuristic_deception_prf_penalty{1.2f};
  float heuristic_repeater_prf_penalty{1.0f};
  float heuristic_repeater_freq_penalty{0.8f};
  float heuristic_unknown_freq_penalty{1.1f};
  float heuristic_unknown_prf_penalty{0.9f};
  float heuristic_unknown_sidelobe_penalty{0.6f};
  float heuristic_unknown_frontlobe_penalty{0.2f};
  float association_scale_max{2.5f};
  float tracking_noise_scale_max{2.0f};
  float measurement_noise_scale_max{1.8f};
  float deception_association_step{0.18f};
  float deception_tracking_step{0.12f};
  float deception_measurement_step{0.10f};
  float repeater_association_step{0.12f};
  float repeater_tracking_step{0.15f};
  float repeater_measurement_step{0.08f};
  float noise_measurement_step{0.06f};
  float unknown_association_step{0.08f};
  float unknown_tracking_step{0.08f};
  float unknown_measurement_step{0.05f};
  float covariance_inflation_max{2.5f};
  float covariance_deception_inflation_step{0.20f};
  float covariance_repeater_inflation_step{0.16f};
  float covariance_noise_inflation_step{0.08f};
  float covariance_unknown_inflation_step{0.10f};
};

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
  float lpi_beam_signal_gain_db{1.0f};
  float adaptive_beam_signal_gain_db{1.5f};
  float eccm_speed_decay_bonus{0.05f};
  float eccm_rcs_decay_bonus{0.08f};
};

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
  float kalman_noise_diff_coeff{1.0f};
};

/**
 * @brief 航迹生命周期与 IMM 相关执行配置。
 */
struct LifecycleExecutionConfig {
  LifecycleConfig policy{};
  engineering::LifecycleRuntimeConfig engineering{};
  signal::tracking::ImmActivationPolicy imm_activation_policy{
      signal::tracking::ImmActivationPolicy::kConfirmedTracksOnly};
  signal::tracking::TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      signal::tracking::TrackPoolThreadSafetyMode::kSingleThreadNoLock};
  std::vector<float> imm_model_noise_diff_coeffs;
  std::vector<float> imm_initial_weights;
  std::vector<float> imm_transition_probability;
  std::size_t track_pool_initial_chunk{64};
  std::size_t track_pool_max_chunks{256};
};

/**
 * @brief 唯一内部执行配置真值。
 *
 * 按 domain 将字段组织为四个子配置，外加跨域共享的干扰/控制效果配置。
 * 各 pipeline phase 函数应只接收其所需的子配置引用，而非整个 InternalExecutionConfig。
 */
struct InternalExecutionConfig {
  bool sensor_enabled{true};  /**< 设备开关机状态 */
  DetectionExecutionConfig detection{};
  AssociationExecutionConfig association{};
  TrackingExecutionConfig tracking{};
  LifecycleExecutionConfig lifecycle{};
  JammingEffectsConfig jamming_effects{};
  ControlProfileEffectsConfig control_profile_effects{};
};

}  // namespace execution
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_EXECUTION_INTERNAL_EXECUTION_CONFIG_H_
