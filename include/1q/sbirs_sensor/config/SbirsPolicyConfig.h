/**
 * @file SbirsPolicyConfig.h
 * @brief 定义 SBIRS-inspired 检测、误差和调度策略。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_

#include <vector>

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

/** @brief 检测门限策略（线性 IR SNR 门限，对应 `T = μ + k·σ` 中的门限取值）。 */
struct ONEQ_API SbirsDetectionPolicyConfig {
  float wide_min_snr_linear{4.0f};  /**< WFOV 检测门限（k≈4） */
  float narrow_min_snr_linear{6.0f}; /**< NFOV 捕获门限（k≈5~6，要求更低虚警率） */
};

// 2.10 误差模型：5 类物理误差各自的 1-σ。
// sigma 单位：角度类为 deg，距离类为无量纲比例。
// random_seed 驱动可注入的确定性高斯采样源（保证 replay 可复现）。
/**
 * @brief 误差模型配置，定义 WFOV 带误差位置生成所用的 5 类物理误差 1-σ。
 * @note 当轨道/姿态/视场三项高斯 sigma 均为 0 时，回退到合并的 `angular_sigma_deg`
 *       以保持向后兼容；`random_seed` 驱动确定性随机源，保证 replay 可复现。
 */
struct ONEQ_API SbirsErrorModelConfig {
  float angular_sigma_deg{0.05f};        /**< 合成 1-σ（向后兼容：轨道+姿态+视场合并），单位 deg */
  float range_fraction_sigma{0.001f};    /**< 距离乘法误差比例 1-σ（约 0.1%），无量纲 */
  unsigned int random_seed{1U};          /**< 随机源种子，驱动可注入高斯采样 */
  float orbit_sigma_deg{0.0f};            /**< 卫星轨道误差角度 1-σ，单位 deg */
  float attitude_sigma_deg{0.01f};        /**< 卫星姿态误差角度 1-σ（典型 ≈0.01°），单位 deg */
  float fov_sigma_deg{0.0f};              /**< 探测器视场（像元/畸变）误差 1-σ，单位 deg */
  float detector_bandwidth_hz{100.0f};    /**< 探测器带宽，用于动态滞后误差，单位 Hz */
};

/**
 * @brief NFOV 资源调度器配置。
 * @note 第一版采用单目标锁定策略，任一时刻至多一个目标占用 NFOV 资源。
 */
struct ONEQ_API SbirsSchedulerConfig {
  bool single_narrow_resource{true}; /**< 是否启用单 NFOV 资源锁定 */
};

/**
 * @brief EKF 滤波测量跟踪配置。
 * @details 启用时，首次 NFOV 捕获成功后进入 `kEstimatedTracking` 状态，用 EKF 估计生成 NFOV 指向
 *          与检测输出角度（SNR/可探测性仍用真值链路，不受滤波发散影响）。禁用时回退真值辅助跟踪
 *          （`kTruthAssistedTracking`）。初始协方差 P0 由位置/速度 1-σ 构造为对角阵。
 */
struct ONEQ_API SbirsTrackingConfig {
  bool enable_estimated_tracking{true};        /**< 是否启用 EKF 滤波测量跟踪（默认开启） */
  float process_noise_diff_coeff{1.0f};        /**< 过程噪声扩散系数 q（CV 模型加速度白噪声强度） */
  float initial_position_std_m{1000.0f};       /**< 初始位置 1-σ（米），构造 P0 位置对角元 */
  float initial_velocity_std_m_per_s{100.0f};  /**< 初始速度 1-σ（m/s），构造 P0 速度对角元 */
  unsigned int nis_gate_loss_cycles{0U}; /**< 连续 NIS 超过 2 维 95% 门限后丢锁的周期数；0 表示禁用 */
  bool enable_imm_tracking{false}; /**< 是否启用 IMM 替代单 EKF（enable_estimated_tracking=true 时有效） */
  std::vector<float> imm_model_noise_diff_coeffs{}; /**< IMM 各模型的过程噪声扩散系数；空向量表示使用默认 [1.0, 100.0] */
};

/**
 * @brief SBIRS-inspired 策略聚合配置，组合检测、误差、调度和跟踪四域策略。
 * @note 纯数据类型 (POD)，作为 `SbirsSessionConfig::policy` 的子配置。
 */
struct ONEQ_API SbirsPolicyConfig {
  SbirsDetectionPolicyConfig detection{};  /**< 检测门限策略 */
  SbirsErrorModelConfig error_model{};     /**< 误差模型策略 */
  SbirsSchedulerConfig scheduler{};        /**< NFOV 资源调度策略 */
  SbirsTrackingConfig tracking{};          /**< EKF 滤波测量跟踪策略 */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
