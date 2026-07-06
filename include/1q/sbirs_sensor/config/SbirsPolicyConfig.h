/**
 * @file SbirsPolicyConfig.h
 * @brief 定义 SBIRS-inspired 检测、误差和调度策略。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

struct ONEQ_API SbirsDetectionPolicyConfig {
  float wide_min_snr_linear{4.0f};
  float narrow_min_snr_linear{6.0f};
};

// 2.10 误差模型：5 类物理误差各自的 1-σ。
// sigma 单位：角度类为 deg，距离类为无量纲比例。
// random_seed 驱动可注入的确定性高斯采样源（保证 replay 可复现）。
struct ONEQ_API SbirsErrorModelConfig {
  float angular_sigma_deg{0.05f};        // 合成 1-σ（向后兼容：轨道+姿态+视场合并）
  float range_fraction_sigma{0.001f};    // 距离乘法误差比例 1-σ
  unsigned int random_seed{1U};
  float orbit_sigma_deg{0.0f};            // 卫星轨道误差角度 1-σ
  float attitude_sigma_deg{0.01f};        // 卫星姿态误差角度 1-σ（典型 ≈0.01°）
  float fov_sigma_deg{0.0f};              // 探测器视场（像元/畸变）误差 1-σ
  float detector_bandwidth_hz{100.0f};    // 探测器带宽（动态滞后误差用）
};

struct ONEQ_API SbirsSchedulerConfig {
  bool single_narrow_resource{true};
};

struct ONEQ_API SbirsPolicyConfig {
  SbirsDetectionPolicyConfig detection{};
  SbirsErrorModelConfig error_model{};
  SbirsSchedulerConfig scheduler{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
