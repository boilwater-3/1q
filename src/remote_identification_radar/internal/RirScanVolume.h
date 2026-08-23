/**
 * @file RirScanVolume.h
 * @brief 可扫描体积判定（会话驻留门与检测候选裁剪共用单源）。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SCAN_VOLUME_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SCAN_VOLUME_H_

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "common/radar/ScanScheduleRuntime.h"

namespace remote_identification_radar {
namespace internal {

/**
 * @brief 目标视线角是否在 scan_center + 可扫描体积内（az 相对、el 绝对）。
 * @param[in] look        目标视线角（雷达局部 ENU，deg）。
 * @param[in] volume      阵面相对可扫描体积（az 相对 scan_center，el 绝对，deg）。
 * @param[in] scan_center 转台当前朝向（ENU az/el，deg）。
 * @return 体积内为 true。
 * @note 同一口径两处消费：会话层指定识别目标驻留门（RirSession）与检测候选
 *       角域裁剪（RirController，2026-08-22 甲方批注「设定方位俯仰进行扫描」）。
 */
inline bool TargetWithinSteerableVolume(
    const config::RirAzimuthElevationDeg& look,
    const config::RirAzimuthElevationLimitsDeg& volume,
    const config::RirAzimuthElevationDeg& scan_center) {
  const float delta_az_deg =
      oneq::common::radar::NormalizeAzimuthDeltaDeg(look.az_deg - scan_center.az_deg);
  return delta_az_deg >= volume.az_min_deg && delta_az_deg <= volume.az_max_deg &&
         look.el_deg >= volume.el_min_deg && look.el_deg <= volume.el_max_deg;
}

}  // namespace internal
}  // namespace remote_identification_radar

#endif  // ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_INTERNAL_RIR_SCAN_VOLUME_H_
