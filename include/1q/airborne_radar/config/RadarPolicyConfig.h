/**
 * @file RadarPolicyConfig.h
 * @brief 定义雷达策略域公开配置。
 *
 * 策略域承载调度、关联、跟踪与生命周期策略。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_POLICY_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_POLICY_CONFIG_H_

#include <cstdint>

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {
namespace beam {

/**
 * @brief 波束指向基线配置。
 */
struct ONEQ_API BeamPointingConfig {
  model::AzimuthElevationDeg default_scan_center_deg{}; /**< 默认扫描中心。 */
  model::CommandedBeamwidthDeg nominal_beamwidth_deg{}; /**< 名义指令态波束宽度。 */
};

/**
 * @brief 波束扫描调度提示配置。
 */
struct ONEQ_API BeamSchedulerConfig {
  std::uint32_t azimuth_step_count_hint{0U}; /**< 方位步进数提示。 */
  std::uint32_t elevation_step_count_hint{0U}; /**< 俯仰步进数提示。 */
  bool prefer_dense_tas_sampling{false}; /**< 是否偏好更密的 TAS 采样。 */
};

/**
 * @brief 波束控制聚合配置。
 */
struct ONEQ_API BeamControlConfig {
  BeamPointingConfig pointing{}; /**< 波束指向配置。 */
  BeamSchedulerConfig scheduler{}; /**< 波束调度配置。 */
};

}  // namespace beam

namespace lifecycle {

/**
 * @brief 航迹生命周期参数。
 */
struct ONEQ_API LifecycleConfig {
  std::uint32_t confirm_hits{3U}; /**< 航迹确认所需命中数。 */
  std::uint32_t max_miss_before_lost{2U}; /**< 丢失前最大连续失配数。 */
  std::uint32_t max_lost_cycles{5U}; /**< 允许保留的最大 lost 周期数。 */
  bool enable_imm_lifecycle{false}; /**< 是否启用 IMM 生命周期路径。 */
};

/**
 * @brief IMM 提示配置。
 */
struct ONEQ_API ImmConfig {
  bool enable_imm_lifecycle{false}; /**< 是否启用 IMM 生命周期路径。 */
  std::uint32_t model_count_hint{2U}; /**< IMM 模型数提示值。 */
};

}  // namespace lifecycle

namespace tracking {

/**
 * @brief Kalman 更新后端类型。
 */
enum class ONEQ_API KalmanUpdateBackend {
  kStandardKfJoseph = 0, /**< 标准 Joseph 形式 KF。 */
  kUdKf = 1, /**< UD 分解 KF。 */
  kSrif = 2, /**< SRIF 后端。 */
  kEkf = 3 /**< 扩展 Kalman 滤波器。 */
};

/**
 * @brief 跟踪参数。
 */
struct ONEQ_API TrackingConfig {
  bool enable_kalman_filter{true}; /**< 是否启用 Kalman 滤波。 */
  float kalman_measurement_noise_std{10.0f}; /**< 量测噪声标准差。 */
  KalmanUpdateBackend kalman_update_backend{KalmanUpdateBackend::kStandardKfJoseph}; /**< 更新后端。 */
  float speed_decay_ratio_on_loss{1.0f}; /**< 丢失周期速度衰减系数（默认无衰减）。 */
  float rcs_decay_ratio_on_loss{1.0f}; /**< 丢失周期 RCS 衰减系数（默认无衰减）。 */
};

/**
 * @brief 量测关联参数。
 */
struct ONEQ_API AssociationConfig {
  float unassigned_cost{9.0f}; /**< 未分配量测代价。 */
  bool use_distance_gate_hint{false}; /**< 是否启用距离门限 sigma 提示。为 false 时库内自适应计算门限。 */
  float distance_gate_sigma_hint{0.0f}; /**< 距离门限 sigma 提示值（单位：m）。仅当 use_distance_gate_hint=true 时生效。 */
};

}  // namespace tracking

using beam::BeamControlConfig;
using beam::BeamPointingConfig;
using beam::BeamSchedulerConfig;
using lifecycle::ImmConfig;
using lifecycle::LifecycleConfig;
using tracking::AssociationConfig;
using tracking::KalmanUpdateBackend;
using tracking::TrackingConfig;

/**
 * @brief 雷达策略域配置。
 *
 * 当前阶段策略域承载调度、关联、跟踪与生命周期策略。
 */
struct ONEQ_API RadarPolicyConfig {
  BeamControlConfig beam_control{};
  AssociationConfig association{};
  TrackingConfig tracking{};
  LifecycleConfig lifecycle{};
  ImmConfig imm{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_POLICY_CONFIG_H_
