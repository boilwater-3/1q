/**
 * @file RirMeasurementErrorModel.h
 * @brief 定义 RIR 基于有效信噪比与波束宽度的测量误差模型（私有实现头）。
 *
 * 副本来源：`src/airborne_radar/signal/detection/MeasurementErrorModel.h`
 * （审计基线 96de367c，阶段 2-M M6），波束宽度类型走 `RirEffectiveBeamwidthDeg`、
 * 物理函数走 `RirRadarEquations`，数值语义逐行一致。
 * @note 本文件仅供 RIR 模块内部使用，不作为公开 API；检测量测误差供
 *       阶段 2-T 轻量关联的波门定标消费（不出 public 面）。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_MEASUREMENT_ERROR_MODEL_H_
#define REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_MEASUREMENT_ERROR_MODEL_H_

#include <cmath>

#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/internal/RirRadarEquations.h"

namespace remote_identification_radar {
namespace dwell {

/**
 * @brief RirMeasurementErrorState 表示当前探测的测量误差结果。
 */
struct RirMeasurementErrorState {
  float range_error_std_m{0.0f};   /**< 距离测量标准差（米）。 */
  float angle_error_std_rad{0.0f}; /**< 方位/俯仰合成的等效角度测量标准差（弧度）。 */
};

/**
 * @brief RirMeasurementErrorModel 负责计算当前探测的测量误差。
 */
class RirMeasurementErrorModel {
 public:
  /**
   * @brief 计算测量误差。
   * @param effective_snr_db 等效积累后的信噪比（dB）。
   * @param effective_beamwidth_deg 生效波束宽度（度）。
   * @param bandwidth_hz 发射机信号带宽（Hz）。
   * @return 测量误差结果。
   */
  static RirMeasurementErrorState Compute(float effective_snr_db,
                                          const RirEffectiveBeamwidthDeg& effective_beamwidth_deg,
                                          float bandwidth_hz) {
    RirMeasurementErrorState state;
    state.range_error_std_m =
        internal::RirRadarEquations::ComputeRangeErrorStdDev(effective_snr_db, bandwidth_hz);
    state.angle_error_std_rad =
        ComputeEquivalentAngleErrorStdDev(effective_snr_db, effective_beamwidth_deg);
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
      float snr_db, const RirEffectiveBeamwidthDeg& beamwidth_deg) {
    const float kDeg2Rad = 3.14159265358979f / 180.0f;
    const float az_beamwidth_rad = beamwidth_deg.az_beamwidth_deg * kDeg2Rad;
    const float el_beamwidth_rad = beamwidth_deg.el_beamwidth_deg * kDeg2Rad;
    const float az_std_rad =
        internal::RirRadarEquations::ComputeAngleErrorStdDev(snr_db, az_beamwidth_rad);
    const float el_std_rad =
        internal::RirRadarEquations::ComputeAngleErrorStdDev(snr_db, el_beamwidth_rad);
    return std::sqrt(0.5f * (az_std_rad * az_std_rad + el_std_rad * el_std_rad));
  }
};

}  // namespace dwell
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_DWELL_RIR_MEASUREMENT_ERROR_MODEL_H_
