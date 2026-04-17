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
 * @brief RadarHardwareProfile 表示外部输入的雷达硬件能力档位。
 */
enum class RadarHardwareProfile {
  kGenericAirborneXBand = 0, /**< 通用机载 X 波段能力 */
  kLongRangeHighPower = 1,   /**< 远程高功率能力 */
  kLightweightLpi = 2        /**< 轻量低截获概率能力 */
};

/**
 * @brief DetectionIntentProfile 表示外部输入的探测策略意图。
 */
enum class DetectionIntentProfile {
  kBalanced = 0,             /**< 平衡探测/跟踪 */
  kDetectionPriority = 1,    /**< 探测优先 */
  kTrackStabilityPriority = 2 /**< 跟踪稳定优先 */
};

/**
 * @brief RcsFusionProfile 表示目标 RCS 物理融合语义档位。
 */
enum class RcsFusionProfile {
  kDisabled = 0,     /**< 关闭物理 RCS 融合 */
  kConservative = 1, /**< 保守融合 */
  kEnhanced = 2      /**< 增强融合 */
};

/**
 * @brief SignalDetectionConfig 描述探测域对外语义输入。
 */
struct SignalDetectionConfig {
  bool enable_physics_detection{false}; /**< 是否启用物理检测路径 */
  RadarHardwareProfile hardware_profile{
      RadarHardwareProfile::kGenericAirborneXBand}; /**< 雷达硬件能力档位 */
  DetectionIntentProfile intent_profile{
      DetectionIntentProfile::kBalanced}; /**< 探测意图档位 */
  AntennaPatternConfig antenna_pattern{}; /**< 天线方向图语义输入 */
  RcsFusionProfile rcs_fusion_profile{
      RcsFusionProfile::kDisabled}; /**< RCS 融合语义档位 */
  float min_detection_margin_db{-2.0f};   /**< 启发式检测最小裕量门限（语义层） */
};

namespace engineering {

struct TransmitterConfig {
  float peak_power_w{1e6f};
  float frequency_hz{3e9f};
  float bandwidth_hz{4.5e6f};
  float pulse_width_s{13e-6f};
  float prf_hz{300.0f};
  float transmit_loss_db{3.5f};
};

struct AntennaConfig {
  float main_beam_gain_db{35.0f};
  float nominal_az_beamwidth_deg{4.0f};
  float nominal_el_beamwidth_deg{4.0f};
  engineering::AntennaPatternConfig pattern{};
  bool enable_directional_pattern{false};
};

struct ReceiverConfig {
  float noise_figure_db{4.0f};
  float receive_loss_db{2.0f};
};

struct DetectionPolicy {
  float cfar_pfa{1e-6f};
  float min_snr_db{-10.0f};
};

struct RcsPhysicsConfig {
  bool enable_physical_rcs{false};
  float frequency_hz{0.0f};
  float physics_mix_ratio{0.0f};
  float cylinder_weight{0.5f};
  float min_equivalent_radius_m{0.05f};
  float max_equivalent_radius_m{5.0f};
  float min_rcs_m2{0.01f};
  float max_rcs_m2{1000.0f};
  float bistatic_psi_offset_deg{5.0f};
};

struct DetectionConfig {
  bool enable_physics_detection{false};
  TransmitterConfig transmitter{};
  AntennaConfig antenna{};
  ReceiverConfig receiver{};
  DetectionPolicy detection_policy{};
  RcsPhysicsConfig rcs_physics{};
  float min_detection_margin_db{-2.0f};
  int pulse_count{10};
};

}  // namespace engineering

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_DETECTION_CONFIG_H_
