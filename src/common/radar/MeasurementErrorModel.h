/**
 * @file MeasurementErrorModel.h
 * @brief 定义基于有效信噪比与波束宽度的通用测量误差模型（common 单源）。
 */

#ifndef COMMON_RADAR_MEASUREMENT_ERROR_MODEL_H_
#define COMMON_RADAR_MEASUREMENT_ERROR_MODEL_H_

#include <cmath>

#include "common/radar/RadarEquations.h"

namespace oneq {
namespace common {
namespace radar {

/**
 * @brief MeasurementErrorState 表示当前探测的测量误差结果。
 */
struct MeasurementErrorState {
  float range_error_std_m{0.0f};   /**< 距离测量标准差（米）。 */
  float angle_error_std_rad{0.0f}; /**< 方位/俯仰合成的等效角度测量标准差（弧度）。 */
};

/**
 * @brief 根据有效方位/俯仰波束宽度计算等效角度测量标准差。
 * @param snr_db 等效积累后的信噪比（dB）。
 * @param az_beamwidth_rad 有效方位波束宽度（弧度）。
 * @param el_beamwidth_rad 有效俯仰波束宽度（弧度）。
 * @return 横向各向同性量测模型使用的等效角度标准差（弧度）。
 */
inline float ComputeEquivalentAngleErrorStdDev(float snr_db, float az_beamwidth_rad,
                                               float el_beamwidth_rad) {
  const float az_std_rad = RadarEquations::ComputeAngleErrorStdDev(snr_db, az_beamwidth_rad);
  const float el_std_rad = RadarEquations::ComputeAngleErrorStdDev(snr_db, el_beamwidth_rad);
  return std::sqrt(0.5f * (az_std_rad * az_std_rad + el_std_rad * el_std_rad));
}

/**
 * @brief 计算测量误差。
 * @param effective_snr_db 等效积累后的信噪比（dB）。
 * @param bandwidth_hz 发射机信号带宽（Hz）。
 * @param az_beamwidth_rad 生效方位波束宽度（弧度）。
 * @param el_beamwidth_rad 生效俯仰波束宽度（弧度）。
 * @return 测量误差结果。
 */
inline MeasurementErrorState ComputeMeasurementError(float effective_snr_db, float bandwidth_hz,
                                                     float az_beamwidth_rad,
                                                     float el_beamwidth_rad) {
  MeasurementErrorState state;
  state.range_error_std_m = RadarEquations::ComputeRangeErrorStdDev(effective_snr_db, bandwidth_hz);
  state.angle_error_std_rad =
      ComputeEquivalentAngleErrorStdDev(effective_snr_db, az_beamwidth_rad, el_beamwidth_rad);
  return state;
}

}  // namespace radar
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RADAR_MEASUREMENT_ERROR_MODEL_H_
