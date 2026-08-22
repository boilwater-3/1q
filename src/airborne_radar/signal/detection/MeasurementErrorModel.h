/**
 * @file MeasurementErrorModel.h
 * @brief AR 测量误差模型薄适配层（common 单源）。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_MEASUREMENT_ERROR_MODEL_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_MEASUREMENT_ERROR_MODEL_H_

#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "common/numerics/Constants.h"
#include "common/radar/MeasurementErrorModel.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief MeasurementErrorState 表示当前探测的测量误差结果。
 */
struct MeasurementErrorState {
  float range_error_std_m{0.0f};   /**< 距离测量标准差（米）。 */
  float angle_error_std_rad{0.0f}; /**< 方位/俯仰合成的等效角度测量标准差（弧度）。 */
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
  static MeasurementErrorState Compute(float effective_snr_db,
                                       const EffectiveBeamwidthDeg& effective_beamwidth_deg,
                                       float bandwidth_hz) {
    const oneq::common::radar::MeasurementErrorState common_state =
        oneq::common::radar::ComputeMeasurementError(
            effective_snr_db, bandwidth_hz,
            oneq::common::numerics::DegToRad(effective_beamwidth_deg.az_beamwidth_deg),
            oneq::common::numerics::DegToRad(effective_beamwidth_deg.el_beamwidth_deg));
    MeasurementErrorState state;
    state.range_error_std_m = common_state.range_error_std_m;
    state.angle_error_std_rad = common_state.angle_error_std_rad;
    return state;
  }

 private:
  /**
   * @brief 根据有效方位/俯仰波束宽度计算等效角度测量标准差。
   * @param snr_db 等效积累后的信噪比（dB）。
   * @param beamwidth_deg 已解析的有效方位/俯仰波束宽度。
   * @return 横向各向同性量测模型使用的等效角度标准差（弧度）。
   */
  static float ComputeEquivalentAngleErrorStdDev(float snr_db,
                                                 const EffectiveBeamwidthDeg& beamwidth_deg) {
    return oneq::common::radar::ComputeEquivalentAngleErrorStdDev(
        snr_db, oneq::common::numerics::DegToRad(beamwidth_deg.az_beamwidth_deg),
        oneq::common::numerics::DegToRad(beamwidth_deg.el_beamwidth_deg));
  }
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_MEASUREMENT_ERROR_MODEL_H_
