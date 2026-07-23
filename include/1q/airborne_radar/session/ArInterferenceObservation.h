/**
 * @file ArInterferenceObservation.h
 * @brief 定义 AR 接收机生成的去真值化 RF 干扰观测。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_INTERFERENCE_OBSERVATION_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_INTERFERENCE_OBSERVATION_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 表示通过接收机 J/N 门限后形成的本地 RF 干扰观测。
 * @note 该类型不携带 truth platform、equipment 或 emission 身份。
 */
struct ONEQ_API ArInterferenceObservation {
  std::uint64_t observation_id{0U};            /**< 当前周期结果内的稳定本地编号。 */
  double estimated_bearing_azimuth_deg{0.0};   /**< ECEF 切平面方位估计。 */
  double estimated_bearing_elevation_deg{0.0}; /**< ECEF 仰角估计。 */
  double estimated_off_boresight_deg{0.0};     /**< 相对冻结接收波束轴的夹角估计。 */
  double estimated_center_frequency_hz{0.0};   /**< 中心频率估计。 */
  double estimated_bandwidth_hz{0.0};          /**< 占用带宽估计。 */
  oneq::electromagnetics::RfSceneWaveformKind estimated_waveform_kind{
      oneq::electromagnetics::RfSceneWaveformKind::kContinuous}; /**< 波形类别估计。 */
  double jammer_to_noise_db{0.0};                                /**< 接收端 J/N（dB）。 */
  double bearing_standard_deviation_deg{0.0};                    /**< 方位/俯仰一标准差。 */
  double frequency_standard_deviation_hz{0.0};                   /**< 频率一标准差。 */
  double bandwidth_standard_deviation_hz{0.0};                   /**< 带宽一标准差。 */
};

/** @brief AR 干扰观测列表。 */
using ArInterferenceObservationList = std::vector<ArInterferenceObservation>;

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_INTERFERENCE_OBSERVATION_H_
