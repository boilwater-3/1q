/**
 * @file EosPipelineTypes.h
 * @brief EOS 管线扩展契约类型。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_PIPELINE_EOS_PIPELINE_TYPES_H_
#define ELECTRO_OPTICAL_SENSOR_PIPELINE_EOS_PIPELINE_TYPES_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace pipeline {

/**
 * @brief EosPipelineWorkMode 描述核心探测评估模式。
 */
enum class ONEQ_API EosPipelineWorkMode {
  kInfraredOnly = 0,
  kVisibleOnly,
  kFused
};

/**
 * @brief EosPipelineEnvironmentModelType 描述环境模型策略。
 */
enum class ONEQ_API EosPipelineEnvironmentModelType {
  kSimplified = 0,
  kAdvanced
};

/**
 * @brief EosPipelineConfig 描述核心处理层配置。
 */
struct ONEQ_API EosPipelineConfig {
  float wavelength_lower_um{3.0f};
  float wavelength_upper_um{5.0f};
  float optical_aperture_m{0.2f};
  float focal_length_m{0.8f};
  EosPipelineWorkMode work_mode{EosPipelineWorkMode::kFused};
  float horizontal_fov_deg{6.0f};
  float vertical_fov_deg{4.0f};
  float scan_rate_deg_per_sec{20.0f};
  float frame_rate_hz{30.0f};
  float minimum_snr_db{6.0f};
  float detection_sensitivity_w{1.0e-12f};
  float scan_start_az_deg{-60.0f};
  float scan_end_az_deg{60.0f};
  float scan_center_el_deg{0.0f};
  float boresight_depression_deg{45.0f};
  float min_detection_depression_deg{1.0f};
  float max_detection_depression_deg{89.0f};
  float visible_reference_irradiance_w_m2{800.0f};
  bool enable_straylight_filter{false};
  float hood_inner_half_angle_deg{12.0f};
  float hood_outer_half_angle_deg{75.0f};
  float hood_min_suppression_ratio{0.20f};
  float hood_max_suppression_ratio{0.85f};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  EosPipelineEnvironmentModelType environment_model_type{
      EosPipelineEnvironmentModelType::kSimplified};
};

}  // namespace pipeline
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_PIPELINE_EOS_PIPELINE_TYPES_H_
