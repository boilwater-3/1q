/**
 * @file EmitterObservation.h
 * @brief 定义接收机视角的辐射源观测记录类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_EMITTER_OBSERVATION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_EMITTER_OBSERVATION_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrObservationQuality 表示观测质量等级。
 */
enum class ONEQ_API EsrObservationQuality {
  kLow = 0, /**< 低质量观测 */
  kMedium,  /**< 中等质量观测 */
  kHigh     /**< 高质量观测 */
};

/**
 * @brief EmitterObservation 描述单条接收机观测。
 */
struct ONEQ_API EmitterObservation {
  std::uint64_t observation_id{0U};                     /**< 观测记录唯一标识 */
  double timestamp_s{0.0};                              /**< 观测时间戳（单位：s） */
  double aoa_az_deg{0.0};                               /**< 测得方位角（单位：deg） */
  double aoa_el_deg{0.0};                               /**< 测得俯仰角（单位：deg） */
  double rf_hz{0.0};                                    /**< 测得载频（单位：Hz） */
  double pulse_width_s{0.0};                            /**< 测得脉宽（单位：s） */
  double amplitude_db{0.0};                             /**< 接收幅度（单位：dB） */
  double snr_db{0.0};                                   /**< 观测信噪比（单位：dB） */
  EsrObservationQuality quality{EsrObservationQuality::kLow}; /**< 观测质量等级 */
  bool is_jammed{false};                                /**< 该观测是否受干扰显著影响 */
};

/** @brief EmitterObservationList 表示观测记录列表。 */
using EmitterObservationList = std::vector<EmitterObservation>;

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_EMITTER_OBSERVATION_H_
