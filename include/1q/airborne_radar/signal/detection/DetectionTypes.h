/**
 * @file DetectionTypes.h
 * @brief 定义雷达探测通用配置结构与目标起伏模型枚举。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_DETECTION_TYPES_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_DETECTION_TYPES_H_

#include "1q/airborne_radar/config/AntennaPatternConfig.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief 发射机配置参数。
 */
struct TransmitterConfig {
  float peak_power_w{1e6f};     /**< 峰值发射功率 (W) */
  float frequency_hz{3e9f};     /**< 工作载频 (Hz) */
  float bandwidth_hz{4.5e6f};   /**< 信号带宽 (Hz) */
  float pulse_width_s{13e-6f};  /**< 脉冲宽度 (s) */
  float prf_hz{300.0f};         /**< 脉冲重复频率 (Hz) */
  float transmit_loss_db{3.5f}; /**< 馈线/发射系统损耗 (dB) */
};

/**
 * @brief 天线配置参数。
 * @note 当前宽度字段表示探测链路使用的名义波束宽度。
 *       若后续引入动态波束控制，应额外定义命令态波束宽度配置，
 *       避免与姿态/扫描层中的瞬时波束宽度混淆。
 */
struct AntennaConfig {
  float main_beam_gain_db{35.0f};         /**< 波束中心名义峰值增益 (dB) */
  float nominal_az_beamwidth_deg{4.0f};   /**< 名义方位波束宽度 (°) */
  float nominal_el_beamwidth_deg{4.0f};   /**< 名义俯仰波束宽度 (°) */
  common::config::AntennaPatternConfig pattern;   /**< 天线方向图形状配置 */
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
  float cfar_pfa{1e-6f};    /**< 恒虚警概率 */
  float min_snr_db{-10.0f}; /**< SNR 硬截断下限 (dB) */
};

/**
 * @brief 完整雷达系统配置（组合上述子配置）。
 */
struct RadarSystemConfig {
  TransmitterConfig transmitter; /**< 发射机 */
  AntennaConfig antenna;         /**< 天线 */
  ReceiverConfig receiver;       /**< 接收机 */
  DetectionPolicy detection;     /**< 检测策略 */
};

/**
 * @brief Swerling RCS 起伏模型类型。
 * - 0/1/3: 扫描间慢起伏（一个驻留期内 RCS 恒定）
 * - 2/4:   脉冲间快起伏（每个脉冲独立 RCS）
 * - 1/2:   Rayleigh 分布（多个等强散射体）, PDF: (1/σ̄)·exp(-σ/σ̄)
 * - 3/4:   卡方 k=4 分布（一个主散射体+多个小散射体）, PDF: (4σ/σ̄²)·exp(-2σ/σ̄)
 */
enum SwerlingModel {
  kSwerling0 = 0, /**< 无起伏（确定性 RCS / Swerling V） */
  kSwerling1 = 1, /**< 扫描间慢起伏，Rayleigh */
  kSwerling2 = 2, /**< 脉冲间快起伏，Rayleigh */
  kSwerling3 = 3, /**< 扫描间慢起伏，卡方 k=4 */
  kSwerling4 = 4  /**< 脉冲间快起伏，卡方 k=4 */
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_DETECTION_TYPES_H_
