/**
 * @file EsrExternalOutputAdapter.h
 * @brief ESR 外部输出适配统一入口。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_OUTPUT_ADAPTER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_OUTPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/session/EmitterObservation.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief 外部可消费的 ESR 观测输出。
 * @note ESR 输出没有距离量，无法反解唯一 ECEF 位置；本结构输出 ECEF 单位方位线。
 */
struct ONEQ_API EsrExternalObservation {
  std::uint64_t observation_id{0U};               /**< 观测记录唯一标识 */
  double timestamp_s{0.0};                        /**< 观测时间戳（单位：s） */
  oneq::coordinate::Vector3d bearing_unit_ecef{}; /**< ECEF 单位方位线 */
  double aoa_az_deg{0.0};                         /**< 原始接收机方位角（单位：deg） */
  double aoa_el_deg{0.0};                         /**< 原始接收机俯仰角（单位：deg） */
  double rf_hz{0.0};                              /**< 测得载频（单位：Hz） */
  double pulse_width_s{0.0};                      /**< 测得脉宽（单位：s） */
  double amplitude_db{0.0};                       /**< 接收幅度（单位：dB） */
  double snr_db{0.0};                             /**< 观测信噪比（单位：dB） */
  session::EsrObservationQuality quality{session::EsrObservationQuality::kLow}; /**< 观测质量 */
  bool is_jammed{false}; /**< 是否受干扰显著影响 */
};

/** @brief EsrExternalObservationList 表示外部观测输出集合。 */
using EsrExternalObservationList = std::vector<EsrExternalObservation>;

/**
 * @brief 外部可消费的 ESR 辐射源假设输出。
 */
struct ONEQ_API EsrExternalEmitterHypothesis {
  std::uint64_t hypothesis_id{0U};                             /**< 假设记录唯一标识 */
  oneq::coordinate::Vector3d bearing_unit_ecef{};              /**< ECEF 单位方位线 */
  float bearing_az_deg{0.0f};                                  /**< 原始方位线方位角（单位：deg） */
  float bearing_el_deg{0.0f};                                  /**< 原始方位线俯仰角（单位：deg） */
  float bearing_std_deg{0.0f};                                 /**< 方位测量标准差（单位：deg） */
  float confidence{0.0f};                                      /**< 假设置信度 */
  std::uint32_t last_seen_cycle{0U};                           /**< 最近命中周期号 */
  session::EsrEmitterMode mode{session::EsrEmitterMode::kUnknown}; /**< 工作模式假设 */
  session::EsrThreatLevel threat_level{session::EsrThreatLevel::kLow}; /**< 威胁等级 */
};

/** @brief EsrExternalEmitterHypothesisList 表示外部假设输出集合。 */
using EsrExternalEmitterHypothesisList = std::vector<EsrExternalEmitterHypothesis>;

ONEQ_API bool TryMakeExternalObservationFromRecord(const session::EmitterObservation& observation,
                                                   const oneq::coordinate::LocalFrameReference& reference,
                                                   const oneq::foundation::PoseState& platform_pose,
                                                   EsrExternalObservation* output);

ONEQ_API bool TryMakeExternalHypothesisFromRecord(const session::EmitterHypothesis& hypothesis,
                                                  const oneq::coordinate::LocalFrameReference& reference,
                                                  const oneq::foundation::PoseState& platform_pose,
                                                  EsrExternalEmitterHypothesis* output);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_OUTPUT_ADAPTER_H_
