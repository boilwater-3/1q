/**
 * @file RirHardwareConfig.h
 * @brief 远程识别雷达硬件域主配置类型。
 *
 * 硬件域仅包含物理硬件能力参数（发射机、天线、接收机），服务于识别链路的
 * 效能级 SNR 计算与距离像分辨率推导（带宽）。
 *
 * @note 本文件结构为 `ArHardwareConfig.h`（include/1q/airborne_radar/config/，
 * 审计基线 96de367c）中识别链路实际消费子集的副本，字段名与数值语义保持一致；
 * `RirRcsPhysicsConfig` 与 AR `RcsPhysicsConfig` 字段对齐。阶段 3 评估 common 化。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_HARDWARE_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_HARDWARE_CONFIG_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electromagnetics/RfLinkBudget.h"
#include "1q/electromagnetics/RfScene.h"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirAzimuthElevationDeg 雷达局部坐标系的方位-俯仰二维角度（单位：度）。
 *
 * 角度约定（雷达局部 ENU 右手坐标系，与
 * `remote_identification_radar::session::RirSceneTarget::position_x/y/z` 同帧）：
 * - `az_deg`：从 +x（东）向 +y（北）旋转的方位角，合法域 [-180, 180]；
 *   驻留调度器输出绝对指向前经 `NormalizeAzimuthDeg` 折算。
 * - `el_deg`：相对 x-y 水平面的俯仰角，合法域 [-90, 90]；正值指向 +z（天）。
 * - `(az_deg, el_deg)` 对应的单位指向向量为
 *   `(cos(el)·cos(az), cos(el)·sin(az), sin(el))`；
 *   因此 (0°, 0°) 指向 +x（东向水平），az 正向转向 +y（北），el 正向转向 +z（天）。
 *
 * 本类型既用于方向图安装偏置，也用于驻留波束中心。驻留波束中心由驻留调度
 * 显式给定，RIR 只消费并信任给定值（调度器给指向、RIR 信指向），不按目标
 * 位置重算波束指向。
 */
struct ONEQ_API RirAzimuthElevationDeg {
  float az_deg{0.0f}; /**< 方位角（deg）：从 +x 向 +y，[-180, 180]。 */
  float el_deg{0.0f}; /**< 俯仰角（deg）：相对水平面，[-90, 90]，正值向上。 */

  RirAzimuthElevationDeg() = default;
  RirAzimuthElevationDeg(float az_deg_in, float el_deg_in)
      : az_deg(az_deg_in), el_deg(el_deg_in) {}
};

/**
 * @brief RirAzimuthElevationLimitsDeg 方位-俯仰扫描限位（单位：度）。
 * @note 与 AR `AzimuthElevationLimitsDeg` 同口径；RIR orientation 域中 az 为
 *       相对阵面/转台基准、el 为绝对俯仰；common 扫描内核消费相对 az 与绝对 el。
 */
struct ONEQ_API RirAzimuthElevationLimitsDeg {
  float az_min_deg{-60.0f}; /**< 方位最小扫描角（单位：度）。 */
  float az_max_deg{60.0f};  /**< 方位最大扫描角（单位：度）。 */
  float el_min_deg{-30.0f}; /**< 俯仰最小扫描角（单位：度）。 */
  float el_max_deg{30.0f};  /**< 俯仰最大扫描角（单位：度）。 */
};

namespace hardware {

/**
 * @brief 方向图模型类型（副本：airborne_radar::config::detection::AntennaPatternModelType）。
 */
enum class ONEQ_API RirAntennaPatternModelType {
  kGaussianMainLobe = 0,  /**< 高斯主瓣近似。 */
  kParabolicMainLobe = 1, /**< 抛物线主瓣近似。 */
  kCosinePower = 2,       /**< 余弦幂方向图近似。 */
  kSincPattern = 3        /**< sinc² 方向图（均匀孔径理论解，需物理孔径尺寸）。 */
};

/**
 * @brief 天线方向图参数（副本：airborne_radar::config::detection::AntennaPatternConfig）。
 */
struct ONEQ_API RirAntennaPatternConfig {
  RirAntennaPatternModelType model_type{
      RirAntennaPatternModelType::kGaussianMainLobe}; /**< 主瓣模型类型。 */
  float max_sidelobe_level_db{-20.0f};                /**< 最大旁瓣电平。 */
  float backlobe_level_db{-35.0f};                    /**< 后瓣电平。 */
  float scan_loss_coeff_db_per_deg2{0.0f};            /**< 扫描损失系数。 */
  float max_scan_loss_db{6.0f};                       /**< 扫描损失上限。 */
  config::RirAzimuthElevationDeg boresight_offset_deg{}; /**< 方向图相对安装轴偏置（角度约定同 RirAzimuthElevationDeg）。 */
};

/**
 * @brief 天线工程参数（副本：airborne_radar::config::detection::AntennaConfig）。
 */
struct ONEQ_API RirAntennaConfig {
  float main_beam_gain_db{35.0f}; /**< 主瓣峰值增益。 */
  float nominal_az_beamwidth_deg{
      4.0f}; /**< 名义方位波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float nominal_el_beamwidth_deg{
      4.0f}; /**< 名义俯仰波束宽度；正值直接生效，0 表示从有效物理孔径推导。 */
  float antenna_length_m{
      0.0f}; /**< 物理方位孔径尺寸；正值参与波束推导和 sinc² 模式，0 表示未配置。 */
  float antenna_width_m{
      0.0f}; /**< 物理俯仰孔径尺寸；正值参与波束推导和 sinc² 模式，0 表示未配置。 */
  RirAntennaPatternConfig pattern{};         /**< 方向图参数。 */
  bool enable_directional_pattern{false}; /**< 是否启用离轴方向图评估。 */
};

/**
 * @brief 接收机工程参数（副本：airborne_radar::config::detection::ReceiverConfig）。
 */
struct ONEQ_API RirReceiverConfig {
  std::uint64_t equipment_id{2U}; /**< RF scene 中的接收设备身份；同平台内必须唯一。 */
  float noise_figure_db{4.0f};    /**< 接收机噪声系数。 */
  float receive_loss_db{2.0f};    /**< 接收链路损耗。 */
  float cross_polarization_isolation_db{30.0f}; /**< 正交极化隔离（dB）。 */
  float minimum_far_field_range_m{1.0f};        /**< 远场公式最小适用距离（m）。 */
  bool has_co_site_isolation{false};            /**< 是否配置同平台耦合隔离。 */
  float co_site_isolation_db{0.0f};             /**< 同平台耦合路径隔离（dB）。 */
  float maximum_linear_input_power_w{1.0e-3f};  /**< 线性接收上限（W）。 */
  float preselector_bandwidth_hz{20.0e6f};      /**< 宽带前端预选器带宽（Hz）。 */
  float interference_observation_jn_gate_db{6.0f}; /**< 干扰观测发布 J/N 门限（dB）。 */
  oneq::electromagnetics::RfScenePolarization scene_polarization{
      oneq::electromagnetics::RfScenePolarization::kHorizontal}; /**< RF v2 接收极化。 */
  std::vector<oneq::electromagnetics::RfCoSiteIsolationPath> co_site_paths{
      {1U, 2U, 120.0}}; /**< 发射设备到接收设备的显式有向隔离路径。 */
};

/**
 * @brief 发射机工程参数（副本：airborne_radar::config::detection::TransmitterConfig）。
 * @note 识别雷达距离像分辨率由 `bandwidth_hz` 推导（c/(2B)），带宽是识别能力
 *       的关键硬件参数。
 */
struct ONEQ_API RirTransmitterConfig {
  std::uint64_t equipment_id{1U};      /**< RF scene 中的发射设备身份；同平台内必须唯一。 */
  float peak_power_w{1e6f};            /**< 峰值发射功率。 */
  float frequency_hz{3e9f};            /**< 工作频率。 */
  float bandwidth_hz{4.5e6f};          /**< 发射带宽。 */
  float pulse_width_s{13e-6f};         /**< 脉宽。 */
  float prf_hz{300.0f};                /**< 脉冲重复频率。 */
  float transmit_loss_db{3.5f};        /**< 发射链路损耗。 */
  float maximum_peak_power_w{1.2e6f};  /**< 烧穿等控制不可越过的峰值功率上限。 */
  float maximum_duty_cycle{0.10f};     /**< 允许的最大发射占空比，范围 (0, 1]。 */
  float maximum_pulse_energy_j{20.0f}; /**< 单脉冲能量上限（J）。 */
  std::vector<double> frequency_plan_hz{3.0e9}; /**< 允许周期执行选择的离散载频表。 */
};

/**
 * @brief RCS 物理建模参数（副本：airborne_radar::config::detection::RcsPhysicsConfig）。
 */
struct ONEQ_API RirRcsPhysicsConfig {
  bool enable_physical_rcs{false};      /**< 是否启用物理 RCS 估计。 */
  float physics_mix_ratio{0.0f};        /**< 物理估计与经验值的混合比例。 */
  float cylinder_weight{0.5f};          /**< 圆柱散射模型权重。 */
  float min_equivalent_radius_m{0.05f}; /**< 等效半径下界。 */
  float max_equivalent_radius_m{5.0f};  /**< 等效半径上界。 */
  float min_rcs_m2{0.01f};              /**< RCS 裁剪下界。 */
  float max_rcs_m2{1000.0f};            /**< RCS 裁剪上界。 */
  float bistatic_psi_offset_deg{5.0f};  /**< 双站角偏移补偿。 */
};

/**
 * @brief 信号处理增益偏置（阶段 2-M M3，《能力边界界定》§3.2 方案 A）。
 *
 * 四个 dB 偏置叠加在分项 SINR 账本上；**缺省 0 dB 时与保守账本逐位一致**
 * （脉压/积累增益自动派生、分母不加处理增益的口径不变）。符号约定跟随各物理量
 * 规格书惯例：改善因子正 dB 为优（target/clutter/jamming），噪声代价正 dB 为劣
 * （noise，与 `noise_figure_db` 同向）。值域 [0, 40] dB（配置校验拒绝越界）。
 * @note 脉压（B·τ）与积累（N）增益永远自动应用，禁止把派生量手填进偏置（防双算）；
 *       额外链路损耗继续走 `transmit_loss_db`/`receive_loss_db`，不与增益混用。
 */
struct ONEQ_API RirSignalProcessingConfig {
  float target_processing_gain_db{0.0f}; /**< 目标信号额外处理增益；正 = 提升 SINR（账本分子）。 */
  float noise_processing_gain_db{0.0f}; /**< 噪声代价；正 = 抬高噪声底（账本分母）。 */
  float clutter_suppression_gain_db{0.0f}; /**< 杂波抑制（MTI 改善因子）；正 = 抑制杂波（分母）。 */
  float jamming_suppression_gain_db{0.0f}; /**< 干扰抑制（旁瓣对消改善因子）；正 = 抑制干扰（分母）。 */
};

}  // namespace hardware

/**
 * @brief RirHardwareConfig 远程识别雷达硬件域配置。
 *
 * 仅包含物理硬件能力。识别策略门限（acceptance_score 等）在
 * `RirPolicyConfig::recognition` 中。
 */
struct ONEQ_API RirHardwareConfig {
  hardware::RirTransmitterConfig transmitter{};      /**< 发射机参数。 */
  hardware::RirAntennaConfig antenna{};              /**< 天线参数。 */
  hardware::RirReceiverConfig receiver{};            /**< 接收机参数。 */
  hardware::RirRcsPhysicsConfig rcs_physics{};       /**< RCS 物理建模参数。 */
  hardware::RirSignalProcessingConfig signal_processing{}; /**< 信号处理增益偏置（默认全 0 dB）。 */
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_HARDWARE_CONFIG_H_
