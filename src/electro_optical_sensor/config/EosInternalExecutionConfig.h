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
 * 其中 OpticsConfig 和 ScanConfig 与公开类型完全一致，使用 using 别名；
 * 其余 4 个为独立内部类型，字段为策略解析后的数值（无 profile 元数据）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_INTERNAL_EXECUTION_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_INTERNAL_EXECUTION_CONFIG_H_

#include "1q/electro_optical_sensor/config/EosHardwareConfig.h"
#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace config {
namespace execution {

// ── 与公开类型完全一致的 using 别名 ──
using OpticsConfig = config::EosHardwareConfig;
using ScanConfig = config::EosMissionConfig;

// ── 新建内部类型（策略解析后、无 profile 元数据） ──

/**
 * @brief 探测器硬件参数配置。
 */
struct DetectorConfig {
  float detector_detectivity_cm_sqrt_hz_per_w{1.0e10f};
  float detector_area_cm2{0.25f};
  float min_detection_depression_deg{1.0f};
  float max_detection_depression_deg{89.0f};
};

/**
 * @brief 杂散光抑制配置（profile 解析后的数值）。
 */
struct StrayLightConfig {
  bool enable_straylight_filter{false};
  float hood_inner_half_angle_deg{12.0f};
  float hood_outer_half_angle_deg{75.0f};
  float hood_min_suppression_ratio{0.20f};
  float hood_max_suppression_ratio{0.85f};
};

/**
 * @brief 环境衰减配置（场景派生后的数值）。
 */
struct EnvironmentConfig {
  environment::EosEnvironmentModelType environment_model_type{
      environment::EosEnvironmentModelType::kSimplified};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
};

/**
 * @brief 探测判决配置（profile 解析后的数值）。
 */
struct DetectionConfig {
  float minimum_snr_db{6.0f};
  float detection_sensitivity_w{1.0e-12f};
  float visible_reference_irradiance_w_m2{800.0f};
};

/**
 * @brief 唯一内部执行配置真值。
 *
 * 按物理处理域将字段组织为 6 个子配置。
 * 各 pipeline 函数应只接收其所需的子配置引用，而非整个 EosInternalExecutionConfig。
 */
struct EosInternalExecutionConfig {
  OpticsConfig optics{};
  ScanConfig scan{};
  DetectorConfig detector{};
  StrayLightConfig stray_light{};
  EnvironmentConfig environment{};
  DetectionConfig detection{};
};

}  // namespace execution
}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_INTERNAL_EXECUTION_CONFIG_H_
