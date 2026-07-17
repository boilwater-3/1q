/**
 * @file ArHardwareConfig.h
 * @brief 机载雷达硬件域主配置类型集合。
 *
 * 硬件域配置（探测链路物理参数、天线方向图、RCS 物理建模等）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

namespace profiles {

/**
 * @brief 目标雷达散射截面起伏统计模型。
 *
 * 不同 Swerling 模型对应不同的目标 RCS 起伏统计特性，
 * 影响多脉冲检测概率的计算方式。
 */
enum class ONEQ_API SwerlingModel {
  kSwerling0 = 0, /**< 不起伏——目标 RCS 在各次观测中恒定。 */
  kSwerling1 = 1, /**< 扫描间慢起伏，单次扫描内 RCS 恒定。 */
  kSwerling2 = 2, /**< 脉冲间快起伏，每个脉冲 RCS 独立采样。 */
  kSwerling3 = 3, /**< 扫描间慢起伏，RCS 服从 2 自由度 chi-squared 分布。 */
  kSwerling4 = 4  /**< 脉冲间快起伏，RCS 服从 2 自由度 chi-squared 分布。 */
};

/**
 * @brief 预设硬件能力档位。
 *
 * 选择档位后 Builder 会自动填写发射功率、工作频率、天线增益、
 * 接收机噪声系数等探测链路物理参数，免去逐项手工配置。
 */
enum class ONEQ_API ArHardwareProfile {
  kGenericAirborneXBand = 0, /**< 典型机载 X 波段（~3 GHz, 1 MW），适用于通用仿真。 */
  kLongRangeHighPower = 1,   /**< 远程高功率（5 MW, 9.3 GHz），适用于远距探测场景。 */
  kLightweightLpi = 2        /**< 轻型低截获概率（350 kW, 10 GHz），适用于隐蔽探测场景。 */
};

/**
 * @brief 探测意图档位。
 *
 * 控制脉冲积累数、虚警概率、最小信噪比和探测裕量，
 * 决定探测链路在"多发现"与"少误报"之间的平衡点。
 */
enum class ONEQ_API DetectionIntentProfile {
  kBalanced = 0,              /**< 均衡——默认探测裕量。 */
  kDetectionPriority = 1,     /**< 探测优先——降低信噪比门限、增大积累数，追求更高的检测概率。 */
  kTrackStabilityPriority = 2 /**< 航迹稳定优先——收紧门限、减少积累数，降低虚警以保持航迹连续。 */
};

/**
 * @brief 物理 RCS 估计与经验值的融合策略。
 *
 * 控制是否将基于目标几何的物理 RCS 估计值与经验值混合，
 * 以及物理模型的权重占比。
 */
enum class ONEQ_API RcsFusionProfile {
  kDisabled = 0,     /**< 不使用物理 RCS 估计，完全依赖经验值。 */
  kConservative = 1, /**< 低权重物理混合（25%），对经验值做小幅修正。 */
  kEnhanced = 2      /**< 高权重物理混合（60%），显著依赖几何散射模型。 */
};

/**
 * @brief 天线方向图预设档位。
 *
 * 控制旁瓣抑制电平、方向图近似模型和扫描损失上限，
 * 影响 off-boresight 增益衰减和抗干扰方向图特性。
 */
enum class ONEQ_API AntennaPatternProfile {
  kStandard = 0,    /**< 标准方向图——默认旁瓣与扫描损失参数。 */
  kLowSidelobe = 1, /**< 低旁瓣——旁瓣 -30 dB / 后瓣 -42 dB，适合抗旁瓣干扰。 */
  kWideCoverage = 2 /**< 宽覆盖——抛物线主瓣近似，放宽扫描损失上限至 8 dB。 */
};

/**
 * @brief 跟踪滤波策略档位。
 *
 * 控制 Kalman 滤波器量测噪声、更新后端选择和航迹衰减系数，
 * 决定跟踪在"快速响应"与"抗干扰稳定"之间的偏好。
 */
enum class ONEQ_API TrackingPolicyProfile {
  kBalanced = 0,         /**< 均衡——默认 Kalman 参数与衰减系数。 */
  kFastAssociation = 1,  /**< 快速关联——低量测噪声、Joseph 形式更新，适合目标密集场景。 */
  kRobustAntiJamming = 2 /**< 抗干扰鲁棒——高量测噪声、UD 分解更新，提升关联代价以抑制虚假量测。 */
};

/**
 * @brief 航迹生命周期管理策略档位。
 *
 * 控制从 tentative 到 confirmed 所需的检测命中数、
 * 允许的连续丢失次数以及 lost 态最大保留周期。
 */
enum class ONEQ_API LifecyclePolicyProfile {
  kBalanced = 0,       /**< 均衡——默认确认与丢失门限。 */
  kFastConfirm = 1,    /**< 快速确认——1 次命中即确认，1 次丢失即标记 lost，3 周期后删除。 */
  kHighPersistence = 2 /**< 高持久——需 3 次命中确认，容忍 3 次丢失，保留 8 周期。 */
};

}  // namespace profiles

namespace detection {

/**
 * @brief 方向图模型类型。
 */
enum class ONEQ_API AntennaPatternModelType {
  kGaussianMainLobe = 0,  /**< 高斯主瓣近似。 */
  kParabolicMainLobe = 1, /**< 抛物线主瓣近似。 */
  kCosinePower = 2,       /**< 余弦幂方向图近似。 */
  kSincPattern = 3        /**< sinc² 方向图（均匀孔径理论解，需物理孔径尺寸）。 */
};

/**
 * @brief 天线方向图参数。
 */
struct ONEQ_API AntennaPatternConfig {
  AntennaPatternModelType model_type{
      AntennaPatternModelType::kGaussianMainLobe};   /**< 主瓣模型类型。 */
  float max_sidelobe_level_db{-20.0f};               /**< 最大旁瓣电平。 */
  float backlobe_level_db{-35.0f};                   /**< 后瓣电平。 */
  float scan_loss_coeff_db_per_deg2{0.0f};           /**< 扫描损失系数。 */
  float max_scan_loss_db{6.0f};                      /**< 扫描损失上限。 */
  config::AzimuthElevationDeg boresight_offset_deg{}; /**< 方向图相对安装轴偏置。 */
};

/**
 * @brief 天线工程参数。
 */
struct ONEQ_API AntennaConfig {
  float main_beam_gain_db{35.0f};         /**< 主瓣峰值增益。 */
  float nominal_az_beamwidth_deg{4.0f};   /**< 名义方位波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float nominal_el_beamwidth_deg{4.0f};   /**< 名义俯仰波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float antenna_length_m{0.0f};           /**< 物理方位孔径尺寸；正值参与波束推导和 sinc² 模式，0 表示未配置。 */
  float antenna_width_m{0.0f};            /**< 物理俯仰孔径尺寸；正值参与波束推导和 sinc² 模式，0 表示未配置。 */
  AntennaPatternConfig pattern{};         /**< 方向图参数。 */
  bool enable_directional_pattern{false}; /**< 是否启用离轴方向图评估。 */
};

/**
 * @brief 探测门限与判决配置。
 */
struct ONEQ_API DetectionPolicyConfig {
  float cfar_pfa{1e-6f};    /**< CFAR 虚警概率目标值。 */
  float min_snr_db{-10.0f}; /**< 最小信噪比门限。 */
};

/**
 * @brief 接收机工程参数。
 */
struct ONEQ_API ReceiverConfig {
  float noise_figure_db{4.0f}; /**< 接收机噪声系数。 */
  float receive_loss_db{2.0f}; /**< 接收链路损耗。 */
};

/**
 * @brief 发射机工程参数。
 */
struct ONEQ_API TransmitterConfig {
  float peak_power_w{1e6f};     /**< 峰值发射功率。 */
  float frequency_hz{3e9f};     /**< 工作频率。 */
  float bandwidth_hz{4.5e6f};   /**< 发射带宽。 */
  float pulse_width_s{13e-6f};  /**< 脉宽。 */
  float prf_hz{300.0f};         /**< 脉冲重复频率。 */
  float transmit_loss_db{3.5f}; /**< 发射链路损耗。 */
};

/**
 * @brief RCS 物理建模参数。
 */
struct ONEQ_API RcsPhysicsConfig {
  bool enable_physical_rcs{false};      /**< 是否启用物理 RCS 估计。 */
  float frequency_hz{0.0f};             /**< 物理 RCS 评估频率；0 跟随当前有效发射频率，正值固定且不随频率捷变。 */
  float physics_mix_ratio{0.0f};        /**< 物理估计与经验值的混合比例。 */
  float cylinder_weight{0.5f};          /**< 圆柱散射模型权重。 */
  float min_equivalent_radius_m{0.05f}; /**< 等效半径下界。 */
  float max_equivalent_radius_m{5.0f};  /**< 等效半径上界。 */
  float min_rcs_m2{0.01f};              /**< RCS 裁剪下界。 */
  float max_rcs_m2{1000.0f};            /**< RCS 裁剪上界。 */
  float bistatic_psi_offset_deg{5.0f};  /**< 双站角偏移补偿。 */
};

/**
 * @brief 探测聚合配置。
 */
struct ONEQ_API DetectionConfig {
  bool enable_physics_detection{false};     /**< 是否启用物理雷达方程检测链。 */
  TransmitterConfig transmitter{};          /**< 发射机参数。 */
  AntennaConfig antenna{};                  /**< 天线参数。 */
  ReceiverConfig receiver{};                /**< 接收机参数。 */
  DetectionPolicyConfig detection_policy{}; /**< 探测判决参数。 */
  RcsPhysicsConfig rcs_physics{};           /**< RCS 物理建模参数。 */
  float min_detection_margin_db{-2.0f};     /**< 最小探测裕量。 */
  int pulse_count{10};                      /**< 脉冲积累数。 */
};

}  // namespace detection

using detection::AntennaPatternModelType;
using detection::DetectionConfig;

/** @brief 雷达硬件域配置——DetectionConfig 别名。 */
using ArHardwareConfig = detection::DetectionConfig;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_HARDWARE_CONFIG_H_
