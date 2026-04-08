/**
 * @file SignalDetectionConfig.h
 * @brief 定义探测域关键配置（config 主入口）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_DETECTION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_DETECTION_CONFIG_H_

#include "1q/airborne_radar/config/AntennaPatternConfig.h"

namespace airborne_radar {
namespace config {

/**
 * @brief 发射机配置参数。
 */
struct TransmitterConfig {
  float peak_power_w{1e6f};     /**< 峰值发射功率 (W) */
  float frequency_hz{3e9f};     /**< 工作载频 (Hz) */
  float bandwidth_hz{4.5e6f};   /**< 信号带宽 (Hz) */
  float pulse_width_s{13e-6f};  /**< 脉冲宽度 (s)，仅物理检测路径参与能量链路 */
  float prf_hz{300.0f};         /**< 脉冲重复频率 (Hz) */
  float transmit_loss_db{3.5f}; /**< 馈线/发射系统损耗 (dB) */
};

/**
 * @brief 天线配置参数。
 */
struct AntennaConfig {
  float main_beam_gain_db{35.0f};         /**< 波束中心名义峰值增益 (dB) */
  float nominal_az_beamwidth_deg{4.0f};   /**< 名义方位波束宽度 (°) */
  float nominal_el_beamwidth_deg{4.0f};   /**< 名义俯仰波束宽度 (°) */
  AntennaPatternConfig pattern;           /**< 天线方向图形状配置 */
  bool enable_directional_pattern{false}; /**< 是否启用离轴方向图增益修正 */
};

/**
 * @brief 接收机配置参数。
 */
struct ReceiverConfig {
  float noise_figure_db{4.0f}; /**< 噪声系数 (dB) */
  float receive_loss_db{2.0f}; /**< 接收系统损耗 (dB) */
};

/**
 * @brief 检测策略参数。
 */
struct DetectionPolicy {
  float cfar_pfa{1e-6f};    /**< 恒虚警概率（仅 enable_physics_detection=true 时生效） */
  float min_snr_db{-10.0f}; /**< SNR 硬截断下限 (dB)（仅 enable_physics_detection=true 时生效） */
};

/**
 * @brief RcsPhysicsConfig 描述可选物理 RCS 覆盖参数。
 * @note 默认关闭，保持现有目标 RCS 使用方式不变。
 */
struct RcsPhysicsConfig {
  bool enable_physical_rcs{false};      /**< 是否启用物理 RCS 覆盖 */
  float frequency_hz{0.0f};             /**< 物理模型频率，<=0 时回退发射机频率 */
  float physics_mix_ratio{0.0f};        /**< 物理 RCS 混合比例 [0,1]，0=纯输入RCS */
  float cylinder_weight{0.5f};          /**< 圆柱散射权重 [0,1] */
  float min_equivalent_radius_m{0.05f}; /**< 目标等效半径下限（m） */
  float max_equivalent_radius_m{5.0f};  /**< 目标等效半径上限（m） */
  float min_rcs_m2{0.01f};              /**< 物理 RCS 输出下限（m^2） */
  float max_rcs_m2{1000.0f};            /**< 物理 RCS 输出上限（m^2） */
  float bistatic_psi_offset_deg{5.0f};  /**< 双基地角偏移（deg） */
};

/**
 * @brief Swerling RCS 起伏模型类型。
 */
enum SwerlingModel {
  kSwerling0 = 0, /**< 无起伏（确定性 RCS / Swerling V） */
  kSwerling1 = 1, /**< 扫描间慢起伏，Rayleigh */
  kSwerling2 = 2, /**< 脉冲间快起伏，Rayleigh */
  kSwerling3 = 3, /**< 扫描间慢起伏，卡方 k=4 */
  kSwerling4 = 4  /**< 脉冲间快起伏，卡方 k=4 */
};

/**
 * @brief SignalDetectionConfig 描述信号探测域关键配置。
 */
struct SignalDetectionConfig {
  bool enable_physics_detection{false}; /**< 是否启用物理层检测 */
  TransmitterConfig transmitter{};      /**< 发射机配置 */
  AntennaConfig antenna{};              /**< 天线配置 */
  ReceiverConfig receiver{};            /**< 接收机配置 */
  DetectionPolicy detection_policy{};   /**< 检测策略 */
  RcsPhysicsConfig rcs_physics{};       /**< 可选物理 RCS 覆盖配置 */
  float min_detection_margin_db{-2.0f}; /**< 最小检测裕量（dB） */
  int pulse_count{10};                  /**< 脉冲积累数量（物理检测路径要求 >=1） */
};

/**
 * @brief 兼容命名别名（带 Signal 前缀）。
 */
using SignalTransmitterConfig = TransmitterConfig;
using SignalAntennaConfig = AntennaConfig;
using SignalReceiverConfig = ReceiverConfig;
using SignalDetectionPolicyConfig = DetectionPolicy;

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_DETECTION_CONFIG_H_
