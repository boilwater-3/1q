/**
 * @file EosInternalExecutionConfig.h
 * @brief 定义 EOS 唯一内部执行配置真值及其物理域子配置。
 *
 * 按物理处理域将 pipeline 配置拆分为 6 个子配置：
 * - OpticsConfig：光学采集（波长/口径/焦距）
 * - ScanConfig：扫描指向（FOV/扫描速率/工作模式）
 * - DetectorConfig：探测器（灵敏度/面积/俯仰限）
 * - StrayLightConfig：杂散光抑制（遮光罩几何/抑制比）
 * - EnvironmentConfig：环境衰减（辐射传输/气溶胶/湍流）
 * - DetectionConfig：探测判决（SNR 门限/灵敏度/参考辐照度）
 *
 * 其中 OpticsConfig、ScanConfig、DetectionConfig 和 StrayLightConfig 与公开类型完全一致，使用 using 别名；
 * 其余 2 个为独立内部类型，字段为策略解析后的数值。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_INTERNAL_EXECUTION_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_INTERNAL_EXECUTION_CONFIG_H_

#include "1q/electro_optical_sensor/config/EosHardwareConfig.h"
#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/config/EosPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace config {
namespace execution {

/** @brief preset 派生的内部辐射传输算法。 */
enum class RadiativeTransferModel {
  kDerivedBeerLambert = 0,
  kHumidityWeighted,
  kAdaptivePathRadiance
};

// ── 与公开类型完全一致的 using 别名 ──

/** @brief 光学采集配置别名（波长/口径/焦距），直接复用公开类型。 */
using OpticsConfig = config::EosHardwareConfig;

/** @brief 扫描指向配置别名（FOV/扫描速率/工作模式），直接复用公开类型。 */
using ScanConfig = config::EosMissionConfig;

// ── 新建内部类型（策略解析后、无 profile 元数据） ──

/**
 * @brief 探测器硬件参数配置。
 */
struct DetectorConfig {
  float detector_detectivity_cm_sqrt_hz_per_w{1.0e10f}; /**< 探测器比探测率（D*） */
  float detector_area_cm2{0.25f};                       /**< 探测器面积（单位：cm2） */
  float min_detection_depression_deg{1.0f};             /**< 最小探测俯仰角（单位：deg） */
  float max_detection_depression_deg{89.0f};            /**< 最大探测俯仰角（单位：deg） */
};

/** @brief 杂散光抑制配置 — 直接别名到公开类型。 */
using StrayLightConfig = config::EosStrayLightPolicyConfig;

/**
 * @brief 环境衰减配置（场景派生后的数值）。
 *
 * @note 本结构完全由公开 preset 和标准大气观测派生，不暴露给调用方。
 */
struct EnvironmentConfig {
  RadiativeTransferModel radiative_transfer_model{
      RadiativeTransferModel::kDerivedBeerLambert}; /**< 辐射传输模型 */
  float aerosol_density_factor{1.0f};                       /**< 气溶胶密度因子 */
  float turbulence_factor{1.0f};                            /**< 湍流因子 */
  oneq::environment::AtmosphericObservation atmospheric_physics{}; /**< 大气物理观测快照 */
};

/** @brief 探测判决配置 — 直接别名到公开类型。 */
using DetectionConfig = config::EosDetectionPolicyConfig;

/**
 * @brief 唯一内部执行配置真值。
 *
 * 按物理处理域将字段组织为 6 个子配置。
 * 各 pipeline 函数应只接收其所需的子配置引用，而非整个 EosInternalExecutionConfig。
 */
struct EosInternalExecutionConfig {
  bool sensor_enabled{true}; /**< 全局设备开关 */
  OpticsConfig optics{};             /**< 光学采集配置 */
  ScanConfig scan{};                 /**< 扫描指向配置 */
  DetectorConfig detector{};         /**< 探测器硬件参数 */
  StrayLightConfig stray_light{};    /**< 杂散光抑制配置 */
  EnvironmentConfig environment{};   /**< 环境衰减配置 */
  DetectionConfig detection{};       /**< 探测判决配置 */
};

}  // namespace execution
}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_INTERNAL_EXECUTION_CONFIG_H_
