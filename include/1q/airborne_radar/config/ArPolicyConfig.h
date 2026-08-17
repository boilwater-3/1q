/**
 * @file ArPolicyConfig.h
 * @brief 机载雷达策略域主配置类型。
 *
 * 策略域（探测、波束调度、关联、跟踪、生命周期、决策控制）配置的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_POLICY_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_POLICY_CONFIG_H_

#include <cstdint>

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

namespace detection {

/**
 * @brief 探测判决策略配置。
 *
 * 统一承载与用户探测意图相关的判决门限和积累参数；硬件域仅描述物理能力。
 */
struct ONEQ_API ArDetectionPolicyConfig {
  float minimum_snr_db{-10.0f};             /**< 最低信噪比门限。 */
  float pfa{1e-6f};                         /**< 单次判决目标虚警概率。 */
  int pulse_count{10};                      /**< 相干或非相干积累脉冲数。 */
  float minimum_detection_margin_db{-2.0f}; /**< 最低探测裕量。 */
};

}  // namespace detection

namespace decision {

/**
 * @brief 跨周期 LPI/ECCM 控制保持与冷却策略。
 *
 * 所有字段以成功执行周期计数。0 表示关闭对应窗口，保持既有立即切换行为。
 */
struct ONEQ_API DecisionControlConfig {
  std::uint32_t lpi_hold_cycles_after_request{0U}; /**< LPI proposal 消失后额外保持的成功周期数。 */
  std::uint32_t eccm_hold_cycles_after_request{0U}; /**< ECCM proposal 消失后额外保持的成功周期数。 */
  std::uint32_t lpi_cooldown_cycles_after_release{0U}; /**< LPI 释放后阻止重新激活的成功周期数。 */
  std::uint32_t eccm_cooldown_cycles_after_release{0U}; /**< ECCM 释放后阻止重新激活的成功周期数。 */
  /**
   * 对抗 VGPO 时的加速度限幅阈值（m/s²）。
   *
   * 当控制策略启用 enable_anti_vgpo_acceleration_bound 时，
   * 航迹生命周期管理器使用此值作为加速度上限，超出此阈值的航迹将被标记为可疑。
   * 该开关由 ArControlProfile 在运行期每周期控制，此处仅设置阈值。
   */
  double anti_vgpo_max_acceleration_mps2{100.0};
};

}  // namespace decision

namespace beam {

/**
 * @brief 波束指向基线配置。
 */
struct ONEQ_API BeamPointingConfig {
  config::CommandedBeamwidthDeg nominal_beamwidth_deg{}; /**< 名义指令态波束宽度。 */
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
  std::uint32_t model_count_hint{2U}; /**< IMM 模型数提示值。 */
};

}  // namespace lifecycle

namespace tracking {

/**
 * @brief 跟踪参数。
 */
struct ONEQ_API TrackingConfig {
  bool enable_kalman_filter{false}; /**< 是否启用 Kalman 滤波（语义默认关闭，需显式开启）。 */
  float kalman_measurement_noise_std{10.0f}; /**< 量测噪声标准差。 */
  float speed_decay_ratio_on_loss{1.0f}; /**< 丢失周期速度衰减系数（默认无衰减）。 */
  float rcs_decay_ratio_on_loss{1.0f}; /**< 丢失周期 RCS 衰减系数（默认无衰减）。 */
  /** Kalman 过程噪声差异系数，控制预测器对状态变化的敏感度。值越大跟踪越平滑但响应越慢。 */
  float kalman_noise_diff_coeff{1.0f};
};

/**
 * @brief 量测关联参数。
 */
struct ONEQ_API AssociationConfig {
  /**
   * 归一化距离关联门限的 sigma 倍数。
   *
   * 内部映射时通过 MapSessionToExecution() / ApplyRuntimePatch() 转换为
   * unassigned_cost = sigma² 用于归一化距离代价计算。逆映射时取
   * std::sqrt(unassigned_cost) 恢复为 sigma 倍数。
   *
   * @see MappingTransforms.h 中的 SigmaToSquaredCost() / SquaredCostToSigma()。
   */
  float distance_gate_sigma{3.0f};
};

}  // namespace tracking

using beam::BeamControlConfig;
using beam::BeamPointingConfig;
using beam::BeamSchedulerConfig;
using decision::DecisionControlConfig;
using detection::ArDetectionPolicyConfig;
using lifecycle::LifecycleConfig;
using tracking::AssociationConfig;
using tracking::TrackingConfig;

/**
 * @brief ArPolicyConfig 雷达策略域配置。
 *
 * 当前阶段策略域承载探测、调度、关联、跟踪、生命周期与决策控制。
 */
struct ONEQ_API ArPolicyConfig {
  ArDetectionPolicyConfig detection{};
  BeamControlConfig beam_control{};
  AssociationConfig association{};
  TrackingConfig tracking{};
  LifecycleConfig lifecycle{};
  DecisionControlConfig decision_control{};
};


}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_POLICY_CONFIG_H_
