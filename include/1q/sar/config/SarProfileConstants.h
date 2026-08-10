/**
 * @file SarProfileConstants.h
 * @brief 合成孔径雷达语义档位常量表。
 *
 * 将原 SarSessionConfigBuilder 的 Profile 枚举翻译结果提取为预定义结构体常量，
 * 用户直接赋值到目标字段即可（"Profile 覆盖"语义不再存在——赋值即最终决定）。
 *
 * 命名约定：k<档位名><子域类型>。仅收录对 struct 默认值产生有效覆盖的档位；
 * 与默认值相同的 no-op 档位（SarMissionProfile::kStripmapSurvey，struct 默认
 * 即该档位）不提供常量。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_PROFILE_CONSTANTS_H_
#define ONEQ_SAR_CONFIG_SAR_PROFILE_CONSTANTS_H_

#include "1q/sar/config/SarMissionConfig.h"
#include "1q/sar/config/SarPolicyConfig.h"

namespace sar {
namespace config {
namespace profiles {

// =============================================================================
// 任务剖面档位（赋给 config.mission，类型 config::SarMissionConfig）
// =============================================================================

/** @brief 高分辨成像：斜距 10km、0.5m 分辨率、2048 脉冲。 */
const config::SarMissionConfig kHighResolutionImagingMission = [] {
  config::SarMissionConfig m{};
  m.nominal_slant_range_m = 10000.0;
  m.platform_speed_mps = 150.0;
  m.azimuth_pulse_count = 2048U;
  m.range_sample_count = 4096U;
  m.desired_ground_range_resolution_m = 0.5;
  m.desired_azimuth_resolution_m = 0.5;
  return m;
}();

/** @brief 远程监视：斜距 50km、3.0m 分辨率、512 脉冲。 */
const config::SarMissionConfig kLongRangeSurveillanceMission = [] {
  config::SarMissionConfig m{};
  m.nominal_slant_range_m = 50000.0;
  m.platform_speed_mps = 200.0;
  m.azimuth_pulse_count = 512U;
  m.range_sample_count = 1024U;
  m.desired_ground_range_resolution_m = 3.0;
  m.desired_azimuth_resolution_m = 3.0;
  return m;
}();

// =============================================================================
// 处理流水线档位（赋给 config.policy，类型 config::SarPolicyConfig）
// =============================================================================

/** @brief 仅生回波：只开 raw echo generation，保留原始回波图像。 */
const config::SarPolicyConfig kRawEchoOnlyProcessing = [] {
  config::SarPolicyConfig p{};
  p.enable_raw_echo_generation = true;
  p.enable_l1_rda_imaging = false;
  p.enable_l2_motion_compensation = false;
  p.enable_l3_bp_imaging = false;
  p.retain_focused_image = false;
  return p;
}();

/** @brief 距离压缩+L1 RDA：开 raw echo + range compression + L1。 */
const config::SarPolicyConfig kRangeCompressedL1Processing = [] {
  config::SarPolicyConfig p{};
  p.enable_raw_echo_generation = true;
  p.enable_l1_rda_imaging = true;
  p.enable_l2_motion_compensation = false;
  p.enable_l3_bp_imaging = false;
  p.retain_focused_image = true;
  return p;
}();

/** @brief L3 BP 路径：开 raw echo + range compression + L3 BP（航点需自行配置）。 */
const config::SarPolicyConfig kL3BackprojectionProcessing = [] {
  config::SarPolicyConfig p{};
  p.enable_raw_echo_generation = true;
  p.enable_l1_rda_imaging = false;
  p.enable_l2_motion_compensation = false;
  p.enable_l3_bp_imaging = true;
  p.retain_focused_image = true;
  return p;
}();

}  // namespace profiles
}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_PROFILE_CONSTANTS_H_
