// Copyright 2026. All Rights Reserved.

/**
 * @file MeasurementErrorModel.h
 * @brief 定义基于有效信噪比与波束宽度的测量误差模型。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_MEASUREMENT_ERROR_MODEL_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_MEASUREMENT_ERROR_MODEL_H_

#include <cmath>

#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/signal/detection/RadarEquations.h"

namespace airborne_radar {
namespace signal {
namespace detection {
/**
 * @brief MeasurementErrorState 表示当前探测的测量误差结果。
 */
struct MeasurementErrorState {
/**
 * @brief 距离测量标准差（米）。
 */
  float range_error_std_m{0.0f};
/**
 * @brief 方位/俯仰合成的等效角度测量标准差（弧度）。
 */
  float angle_error_std_rad{0.0f};
};
/**
 * @brief MeasurementErrorModel 负责计算当前探测的测量误差。
 */
class MeasurementErrorModel {
 public:
/**
 * @brief 计算测量误差。
 * @param effective_snr_db 等效积累后的信噪比（dB）。
 * @param effective_beamwidth_deg 生效波束宽度（度）。
 * @param bandwidth_hz 发射机信号带宽（Hz）。
 * @return 测量误差结果。
 */
  static MeasurementErrorState Compute(
      float effective_snr_db,
      const EffectiveBeamwidthDeg& effective_beamwidth_deg,
      float bandwidth_hz) {
    MeasurementErrorState state;
    state.range_error_std_m = RadarEquations::ComputeRangeErrorStdDev(
        effective_snr_db, bandwidth_hz);
    state.angle_error_std_rad = ComputeEquivalentAngleErrorStdDev(
        effective_snr_db, effective_beamwidth_deg);
    return state;
  }

 private:
/**
 * @brief 根据有效方位/俯仰波束宽度计算等效角度测量标准差。
 * @param snr_db 等效积累后的信噪比（dB）。
 * @param beamwidth_deg 已解析的有效方位/俯仰波束宽度。
 * @return 横向各向同性量测模型使用的等效角度标准差（弧度）。
 */
  static float ComputeEquivalentAngleErrorStdDev(
      float snr_db,
      const EffectiveBeamwidthDeg& beamwidth_deg) {
    const float kDeg2Rad = 3.14159265358979f / 180.0f;
    const float az_beamwidth_rad = beamwidth_deg.az_beamwidth_deg * kDeg2Rad;
    const float el_beamwidth_rad = beamwidth_deg.el_beamwidth_deg * kDeg2Rad;
    const float az_std_rad =
        RadarEquations::ComputeAngleErrorStdDev(snr_db, az_beamwidth_rad);
    const float el_std_rad =
        RadarEquations::ComputeAngleErrorStdDev(snr_db, el_beamwidth_rad);
    return std::sqrt(0.5f * (az_std_rad * az_std_rad +
                             el_std_rad * el_std_rad));
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_MEASUREMENT_ERROR_MODEL_H_
