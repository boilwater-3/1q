/**
 * @file SbirsPolicyConfig.h
 * @brief 定义 SBIRS-inspired 检测、误差和调度策略。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_

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
 * @brief SBIRS-inspired 策略聚合配置，组合检测、误差和调度三域策略。
 * @note 纯数据类型 (POD)，作为 `SbirsSessionConfig::policy` 的子配置。
 */
struct ONEQ_API SbirsPolicyConfig {
  SbirsDetectionPolicyConfig detection{};  /**< 检测门限策略 */
  SbirsErrorModelConfig error_model{};     /**< 误差模型策略 */
  SbirsSchedulerConfig scheduler{};        /**< NFOV 资源调度策略 */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
