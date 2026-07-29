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
 * @brief EsrWaveformClass 表示接收机视角下的波形类别。
 *
 * 由场景波形类别（`oneq::electromagnetics::RfSceneWaveformKind`）映射而来，
 * 用于聚类按波形类别分流与工作模式推断。`kPulse` 的 PRI/PW 字段有物理意义，
 * 其余类别为 energy-only（PRI/PW 保持默认 0）。
 */
enum class ONEQ_API EsrWaveformClass : std::uint8_t {
  kPulse = 0,      /**< 脉冲串；PRI/PW 有物理意义 */
  kContinuous = 1, /**< 连续波；energy-only */
  kSweep = 2,      /**< 线性扫频；energy-only */
  kNoise = 3,      /**< 带限噪声；energy-only */
};

/**
 * @brief EsrDeceptionClass 表示观测的欺骗干扰分类。
 *
 * 基于单周期内脉冲列观测的角度一致性进行判定。
 * 默认 kNone（非欺骗），仅适用于 EsrWaveformClass::kPulse 波形。
 */
enum class ONEQ_API EsrDeceptionClass : std::uint8_t {
  kNone = 0,            /**< 非欺骗（常规观测或未知类型）。 */
  kLikelyFalseTarget,   /**< 疑似假目标：同波束宽度内有多组脉冲列。 */
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
  double bandwidth_hz{0.0};                             /**< 估计占用带宽（单位：Hz） */
  double pri_s{0.0};                                    /**< 估计脉冲重复间隔（单位：s） */
  double pulse_width_s{0.0};                            /**< 测得脉宽（单位：s） */
  double rf_std_hz{0.0};                                /**< 载频估计标准差（单位：Hz） */
  double bandwidth_std_hz{0.0};                         /**< 带宽估计标准差（单位：Hz） */
  double pri_std_s{0.0};                                /**< PRI 估计标准差（单位：s） */
  double pulse_width_std_s{0.0};                        /**< 脉宽估计标准差（单位：s） */
  double amplitude_db{0.0};                             /**< 接收幅度（单位：dB） */
  double snr_db{0.0};                                   /**< 观测信噪比（单位：dB） */
  EsrObservationQuality quality{EsrObservationQuality::kLow}; /**< 观测质量等级 */
  EsrWaveformClass waveform_class{EsrWaveformClass::kPulse}; /**< 波形类别，默认脉冲 */
  EsrDeceptionClass deception_class{EsrDeceptionClass::kNone}; /**< 欺骗干扰分类 */
};

/** @brief EmitterObservationList 表示观测记录列表。 */
using EmitterObservationList = std::vector<EmitterObservation>;

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_EMITTER_OBSERVATION_H_
