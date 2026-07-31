/**
 * @file EosProfileConstants.h
 * @brief 光电传感器语义档位常量表。
 *
 * 将原 EosSessionConfigBuilder 的 Profile 枚举翻译结果提取为预定义结构体常量，
 * 用户直接赋值到目标字段即可（"Profile 覆盖"语义不再存在——赋值即最终决定）。
 *
 * 命名约定：k<档位名><子域类型>。仅收录对 struct 默认值产生有效覆盖的档位；
 * 与默认值相同的 no-op 档位（EosHardwareProfile::kStandardMidWaveIR，struct
 * 默认即该档位）不提供常量。
 *
 * @note Mission 档位跨域覆写 policy.detection.minimum_snr_db：迁移后该覆写
 *   不再自动发生，用户需显式赋值，或接受档位常量携带的 snr 取值。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_PROFILE_CONSTANTS_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_PROFILE_CONSTANTS_H_

#include "1q/electro_optical_sensor/config/EosHardwareConfig.h"
#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/config/EosPolicyConfig.h"

namespace electro_optical_sensor {
namespace config {
namespace profiles {

// =============================================================================
// 任务剖面档位（赋给 config.mission + config.policy.detection.minimum_snr_db）
// =============================================================================

/** @brief 大范围搜索：Fused，12°×8°，30°/s，15Hz，snr=6dB（与 struct 默认一致，无需赋值）。 */
inline const config::EosMissionConfig kWideAreaSearchMission = [] {
  config::EosMissionConfig m{};
  m.work_mode = config::EosWorkMode::kFused;
  m.horizontal_fov_deg = 12.0f;
  m.vertical_fov_deg = 8.0f;
  m.scan_rate_deg_per_sec = 30.0f;
  m.frame_rate_hz = 15.0f;
  return m;
}();

/** @brief 远程监视：InfraredOnly，3°×2°，10°/s，10Hz，snr=3dB。 */
inline const config::EosMissionConfig kLongRangeSurveillanceMission = [] {
  config::EosMissionConfig m{};
  m.work_mode = config::EosWorkMode::kInfraredOnly;
  m.horizontal_fov_deg = 3.0f;
  m.vertical_fov_deg = 2.0f;
  m.scan_rate_deg_per_sec = 10.0f;
  m.frame_rate_hz = 10.0f;
  return m;
}();

/** @brief 远程监视档位携带的探测门限：snr=3dB。 */
inline const config::EosDetectionPolicyConfig kLongRangeSurveillanceDetection = [] {
  config::EosDetectionPolicyConfig c{};
  c.minimum_snr_db = 3.0f;
  return c;
}();

/** @brief 高精度跟踪：Fused，1.5°×1°，5°/s，60Hz，snr=2dB。 */
inline const config::EosMissionConfig kHighResolutionTrackMission = [] {
  config::EosMissionConfig m{};
  m.work_mode = config::EosWorkMode::kFused;
  m.horizontal_fov_deg = 1.5f;
  m.vertical_fov_deg = 1.0f;
  m.scan_rate_deg_per_sec = 5.0f;
  m.frame_rate_hz = 60.0f;
  return m;
}();

/** @brief 高精度跟踪档位携带的探测门限：snr=2dB。 */
inline const config::EosDetectionPolicyConfig kHighResolutionTrackDetection = [] {
  config::EosDetectionPolicyConfig c{};
  c.minimum_snr_db = 2.0f;
  return c;
}();

// =============================================================================
// 硬件规格档位（赋给 config.hardware，类型 config::EosHardwareConfig）
// =============================================================================

/** @brief 远程大口径：3-5μm、0.4m 口径、D*=2e10。 */
inline const config::EosHardwareConfig kLongRangeLargeApertureHardware = [] {
  config::EosHardwareConfig h{};
  h.wavelength_lower_um = 3.0f;
  h.wavelength_upper_um = 5.0f;
  h.optical_aperture_m = 0.4f;
  h.detector_detectivity_cm_sqrt_hz_per_w = 2.0e10f;
  return h;
}();

/** @brief 广域紧凑：8-12μm、0.1m 口径、D*=5e9。 */
inline const config::EosHardwareConfig kWideAreaCompactHardware = [] {
  config::EosHardwareConfig h{};
  h.wavelength_lower_um = 8.0f;
  h.wavelength_upper_um = 12.0f;
  h.optical_aperture_m = 0.1f;
  h.detector_detectivity_cm_sqrt_hz_per_w = 5.0e9f;
  return h;
}();

}  // namespace profiles
}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_PROFILE_CONSTANTS_H_
