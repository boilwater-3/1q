/**
 * @file EosPipelineTypes.h
 * @brief EOS 管线扩展契约类型与输出帧结构。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_EXTENSION_EOS_PIPELINE_TYPES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_EXTENSION_EOS_PIPELINE_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace output {

/**
 * @brief EosDetectionRecord 表示单目标探测输出条目。
 */
struct ONEQ_API EosDetectionRecord {
  std::uint64_t target_id{0U};      /**< 目标标识 */
  float range_m{0.0f};              /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};          /**< 方位角（单位：deg） */
  float elevation_deg{0.0f};        /**< 仰角（单位：deg） */
  float infrared_snr_linear{0.0f};  /**< 红外通道线性 SNR */
  float visible_snr_linear{0.0f};   /**< 可见光通道线性 SNR */
  float fused_snr_linear{0.0f};     /**< 融合线性 SNR */
  float fused_snr_db{0.0f};         /**< 融合 dB SNR */
  bool detected{false};             /**< 是否通过探测门限判决 */
};

/** @brief EosDetectionRecordList 表示单周期探测结果列表。 */
using EosDetectionRecordList = std::vector<EosDetectionRecord>;

}  // namespace output

namespace extension {

/**
 * @brief EosPipelineWorkMode 描述核心探测评估模式。
 * @note 等价于 config::EosWorkMode，保留命名以维持扩展层独立性。
 */
using EosPipelineWorkMode = config::EosWorkMode;

/**
 * @brief EosPipelineEnvironmentModelType 描述环境模型策略。
 * @note 等价于 environment::EosEnvironmentModelType，保留命名以维持扩展层独立性。
 */
using EosPipelineEnvironmentModelType = environment::EosEnvironmentModelType;

/**
 * @brief EosPipelineConfig 描述核心处理层配置（公开契约，拍平类型）。
 *
 * 供 IEosPipeline::UpdateConfig() 使用。
 * 内部执行域请使用 config::execution::EosInternalExecutionConfig。
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
  float detector_detectivity_cm_sqrt_hz_per_w{1.0e10f};
  float detector_area_cm2{0.25f};
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

/**
 * @brief EosPipelineRuntimeState 描述 EOS 管线运行态快照。
 */
struct ONEQ_API EosPipelineRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  float current_scan_azimuth_deg{0.0f};
  float scan_start_az_deg{0.0f};
  float scan_end_az_deg{0.0f};
  float scan_rate_deg_per_sec{0.0f};
};

/**
 * @brief EosPipelineAbortReason 描述核心管线周期终止原因。
 */
enum class ONEQ_API EosPipelineAbortReason {
  kNone = 0,
  kValidationRejected,
  kOutputContractViolation,
  kRuntimeStateRestoreRejected
};

/**
 * @brief EosPipelineExecuteResult 描述核心管线单周期执行结果。
 */
struct ONEQ_API EosPipelineExecuteResult {
  output::EosDetectionRecordList detections{};
  float scan_azimuth_deg{0.0f};
  bool executed_this_cycle{false};
  EosPipelineAbortReason abort_reason{EosPipelineAbortReason::kNone};
};

}  // namespace extension
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_EXTENSION_EOS_PIPELINE_TYPES_H_
