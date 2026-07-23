/**
 * @file EsrOutputTypes.h
 * @brief 定义电子侦察公共输出通道与周期终止原因类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_TYPES_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/session/EmitterObservation.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ObservationOutputFrame 表示观测输出通道。
 */
struct ONEQ_API ObservationOutputFrame {
  std::size_t raw_observation_count{0U};
  std::size_t cluster_count{0U};
  double receiver_center_frequency_hz{0.0}; /**< 本周期接收调谐中心频率（单位：Hz）。 */
  double receiver_bandwidth_hz{0.0};        /**< 本周期接收调谐带宽（单位：Hz）。 */
  bool receiver_saturated{false};           /**< 本周期是否超过最大线性输入功率。 */
  session::EmitterObservationList observations{};
};

/**
 * @brief EmitterOutputFrame 表示侦察输出通道。
 */
struct ONEQ_API EmitterOutputFrame {
  session::EmitterHypothesisList hypotheses{};
};

/**
 * @brief EsrPipelineAbortReason 表示单周期核心管线流产原因。
 */
enum class EsrPipelineAbortReason {
  kNone = 0,                    /**< 正常执行完成，未中断 */
  kValidationRejected,          /**< 因输入级严重校验问题（Error）而主动放弃计算 */
  kRuntimeStateRestoreRejected, /**< 因运行时状态恢复失败引发阻断 */
  kOutputContractViolation,     /**< pipeline 输出违反内部契约 */
  kSensorPoweredOff = 4,        /**< 设备关机，pipeline 未执行 */
  kRfReceiverRejected = 5       /**< RF v2 接收机链路前置条件不成立 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_OUTPUT_TYPES_H_
