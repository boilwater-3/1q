/**
 * @file EsrProfileConstants.h
 * @brief 电子侦察雷达语义档位常量表。
 *
 * 将原 EsrSessionConfigBuilder 的 Profile 枚举翻译结果提取为预定义结构体常量，
 * 用户直接赋值到目标字段即可（"Profile 覆盖"语义不再存在——赋值即最终决定）。
 *
 * 命名约定：k<档位名><子域类型>。仅收录对 struct 默认值产生有效覆盖的档位；
 * 与默认值相同的 no-op 档位（EsrSensitivityProfile::kStandard）不提供常量，
 * struct 默认值即为该档位。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_PROFILE_CONSTANTS_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_PROFILE_CONSTANTS_H_

#include "1q/electronic_surveillance_radar/config/EsrMissionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrPolicyConfig.h"

namespace electronic_surveillance_radar {
namespace config {
namespace profiles {

// =============================================================================
// 任务剖面档位（赋给 config.mission，类型 config::EsrMissionConfig）
// =============================================================================

/** @brief 电子战斗序列采集：ESM 模式，2Hz 快扫，±60°×±10°。 */
const config::EsrMissionConfig kElectronicOrderOfBattleMission = [] {
  config::EsrMissionConfig m{};
  m.work_mode = config::EsrWorkMode::kEsm;
  m.scan.scan_rate_hz = 2.0f;
  m.scan.use_explicit_scan_bounds = true;
  m.scan.scan_start_az_deg = -60.0f;
  m.scan.scan_end_az_deg = 60.0f;
  m.scan.scan_start_el_deg = -10.0f;
  m.scan.scan_end_el_deg = 10.0f;
  return m;
}();

/** @brief 精确辐射源分析：HGESM 模式，0.5Hz 慢扫，±30°×±5°。 */
const config::EsrMissionConfig kPrecisionEmitterAnalysisMission = [] {
  config::EsrMissionConfig m{};
  m.work_mode = config::EsrWorkMode::kHgesm;
  m.scan.scan_rate_hz = 0.5f;
  m.scan.use_explicit_scan_bounds = true;
  m.scan.scan_start_az_deg = -30.0f;
  m.scan.scan_end_az_deg = 30.0f;
  m.scan.scan_start_el_deg = -5.0f;
  m.scan.scan_end_el_deg = 5.0f;
  return m;
}();

/** @brief 威胁告警：RWR 模式，5Hz 快扫，±60°×±10°。 */
const config::EsrMissionConfig kThreatWarningMission = [] {
  config::EsrMissionConfig m{};
  m.work_mode = config::EsrWorkMode::kRwr;
  m.scan.scan_rate_hz = 5.0f;
  m.scan.use_explicit_scan_bounds = true;
  m.scan.scan_start_az_deg = -60.0f;
  m.scan.scan_end_az_deg = 60.0f;
  m.scan.scan_start_el_deg = -10.0f;
  m.scan.scan_end_el_deg = 10.0f;
  return m;
}();

// =============================================================================
// 探测灵敏度档位（赋给 config.policy.detection，类型 config::EsrDetectionPolicyConfig）
// =============================================================================

/** @brief 高灵敏（远距弱信号）：min_snr=3dB，脉冲积累 16，pfa=5e-6。 */
const config::EsrDetectionPolicyConfig kHighSensitivityDetection = [] {
  config::EsrDetectionPolicyConfig c{};
  c.minimum_snr_db = 3.0f;
  c.pulse_count = 16U;
  c.pfa = 5.0e-6f;
  c.threshold_scale = 1.0f;
  c.enable_statistical_detection = true;
  return c;
}();

/** @brief 抗干扰（复杂电磁环境）：min_snr=10dB，脉冲积累 4，pfa=1e-7。 */
const config::EsrDetectionPolicyConfig kRobustDetection = [] {
  config::EsrDetectionPolicyConfig c{};
  c.minimum_snr_db = 10.0f;
  c.pulse_count = 4U;
  c.pfa = 1.0e-7f;
  c.threshold_scale = 1.0f;
  c.enable_statistical_detection = true;
  return c;
}();

}  // namespace profiles
}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_PROFILE_CONSTANTS_H_
