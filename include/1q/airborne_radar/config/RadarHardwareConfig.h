/**
 * @file RadarHardwareConfig.h
 * @brief 定义雷达硬件域公开配置。
 *
 * 硬件域承载探测链路固有能力参数。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_HARDWARE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_HARDWARE_CONFIG_H_

#include "1q/airborne_radar/config/semantic/DetectionProfiles.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert 方向图模型类型。
 */
enum class AntennaPatternModelType {
  kGaussianMainLobe = 0, /**< 高斯主瓣近似。 */
  kParabolicMainLobe = 1, /**< 抛物线主瓣近似。 */
  kCosinePower = 2 /**< 余弦幂方向图近似。 */
};

/**
 * @brief expert 天线方向图参数。
 */
struct AntennaPatternConfig {
  AntennaPatternModelType model_type{AntennaPatternModelType::kGaussianMainLobe}; /**< 主瓣模型类型。 */
  float max_sidelobe_level_db{-20.0f}; /**< 最大旁瓣电平。 */
  float backlobe_level_db{-35.0f}; /**< 后瓣电平。 */
  float scan_loss_coeff_db_per_deg2{0.0f}; /**< 扫描损失系数。 */
  float max_scan_loss_db{6.0f}; /**< 扫描损失上限。 */
  model::AzimuthElevationDeg boresight_offset_deg{}; /**< 方向图相对安装轴偏置。 */
};

/**
 * @brief expert 天线工程参数。
 */
struct AntennaConfig {
  float main_beam_gain_db{35.0f}; /**< 主瓣峰值增益。 */
  float nominal_az_beamwidth_deg{4.0f}; /**< 名义方位波束宽度。 */
  float nominal_el_beamwidth_deg{4.0f}; /**< 名义俯仰波束宽度。 */
  AntennaPatternConfig pattern{}; /**< 方向图参数。 */
  bool enable_directional_pattern{false}; /**< 是否启用离轴方向图评估。 */
};

/**
 * @brief expert 探测门限与判决配置。
 */
struct DetectionPolicyConfig {
  float cfar_pfa{1e-6f}; /**< CFAR 虚警概率目标值。 */
  float min_snr_db{-10.0f}; /**< 最小信噪比门限。 */
};

/**
 * @brief expert 接收机工程参数。
 */
struct ReceiverConfig {
  float noise_figure_db{4.0f}; /**< 接收机噪声系数。 */
  float receive_loss_db{2.0f}; /**< 接收链路损耗。 */
};

/**
 * @brief expert 发射机工程参数。
 */
struct TransmitterConfig {
  float peak_power_w{1e6f}; /**< 峰值发射功率。 */
  float frequency_hz{3e9f}; /**< 工作频率。 */
  float bandwidth_hz{4.5e6f}; /**< 发射带宽。 */
  float pulse_width_s{13e-6f}; /**< 脉宽。 */
  float prf_hz{300.0f}; /**< 脉冲重复频率。 */
  float transmit_loss_db{3.5f}; /**< 发射链路损耗。 */
};

/**
 * @brief expert RCS 物理建模参数。
 */
struct RcsPhysicsConfig {
  bool enable_physical_rcs{false}; /**< 是否启用物理 RCS 估计。 */
  float frequency_hz{0.0f}; /**< 物理 RCS 估计使用的频率。 */
  float physics_mix_ratio{0.0f}; /**< 物理估计与经验值的混合比例。 */
  float cylinder_weight{0.5f}; /**< 圆柱散射模型权重。 */
  float min_equivalent_radius_m{0.05f}; /**< 等效半径下界。 */
  float max_equivalent_radius_m{5.0f}; /**< 等效半径上界。 */
  float min_rcs_m2{0.01f}; /**< RCS 裁剪下界。 */
  float max_rcs_m2{1000.0f}; /**< RCS 裁剪上界。 */
  float bistatic_psi_offset_deg{5.0f}; /**< 双站角偏移补偿。 */
};

/**
 * @brief expert 探测聚合配置。
 */
struct DetectionConfig {
  bool enable_physics_detection{false}; /**< 是否启用物理雷达方程检测链。 */
  TransmitterConfig transmitter{}; /**< 发射机参数。 */
  AntennaConfig antenna{}; /**< 天线参数。 */
  ReceiverConfig receiver{}; /**< 接收机参数。 */
  DetectionPolicyConfig detection_policy{}; /**< 探测判决参数。 */
  RcsPhysicsConfig rcs_physics{}; /**< RCS 物理建模参数。 */
  float min_detection_margin_db{-2.0f}; /**< 最小探测裕量。 */
  int pulse_count{10}; /**< 脉冲积累数。 */
  semantic::SwerlingModel swerling_model{semantic::SwerlingModel::kSwerling0}; /**< 目标起伏模型。 */
};

}  // namespace detection

using detection::AntennaConfig;
using detection::AntennaPatternConfig;
using detection::AntennaPatternModelType;
using detection::DetectionConfig;
using detection::DetectionPolicyConfig;
using detection::RcsPhysicsConfig;
using detection::ReceiverConfig;
using detection::TransmitterConfig;

}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

namespace airborne_radar {
namespace config {

using expert::AntennaConfig;
using expert::AntennaPatternConfig;
using expert::AntennaPatternModelType;
using expert::DetectionConfig;
using expert::DetectionPolicyConfig;
using expert::RcsPhysicsConfig;
using expert::ReceiverConfig;
using expert::TransmitterConfig;

/**
 * @brief 雷达硬件域配置。
 *
 * 当前阶段硬件域承载探测链路固有能力参数。
 */
struct ONEQ_API RadarHardwareConfig {
  DetectionConfig detection{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_HARDWARE_CONFIG_H_
