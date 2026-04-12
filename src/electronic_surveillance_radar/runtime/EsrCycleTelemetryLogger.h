/**
 * @file EsrCycleTelemetryLogger.h
 * @brief 封装电子侦察单周期遥测日志输出职责。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_RUNTIME_ESR_CYCLE_TELEMETRY_LOGGER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_RUNTIME_ESR_CYCLE_TELEMETRY_LOGGER_H_

#include <cstddef>
#include <cstdint>

#include "common/runtime/RuntimeCycleExecutor.h"

namespace electronic_surveillance_radar {
namespace runtime {

/**
 * @brief EsrCycleTelemetryPayload 聚合单周期日志所需的全部字段。
 */
struct EsrCycleTelemetryPayload {
  const oneq::internal::runtime::RuntimeCycleStamp& stamp;
  std::size_t input_emitter_count;
  std::size_t raw_observation_count;
  std::size_t observation_count;
  std::size_t cluster_count;
  std::size_t hypothesis_count;
  std::size_t truth_association_count;
  std::size_t matched_truth_count;
  bool sensor_enabled;

  EsrCycleTelemetryPayload(const oneq::internal::runtime::RuntimeCycleStamp& stamp_,
                           std::size_t input_emitter_count_, std::size_t raw_observation_count_,
                           std::size_t observation_count_, std::size_t cluster_count_,
                           std::size_t hypothesis_count_, std::size_t truth_association_count_,
                           std::size_t matched_truth_count_, bool sensor_enabled_)
      : stamp(stamp_),
        input_emitter_count(input_emitter_count_),
        raw_observation_count(raw_observation_count_),
        observation_count(observation_count_),
        cluster_count(cluster_count_),
        hypothesis_count(hypothesis_count_),
        truth_association_count(truth_association_count_),
        matched_truth_count(matched_truth_count_),
        sensor_enabled(sensor_enabled_) {}
};

/**
 * @brief 无状态周期遥测日志输出工具类。
 */
class EsrCycleTelemetryLogger {
 public:
  /**
   * @brief 将单周期摘要输出为结构化 debug 日志。
   * @param payload 本周期日志载荷。
   */
  static void LogCycleSummary(const EsrCycleTelemetryPayload& payload);
};

}  // namespace runtime


}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_RUNTIME_ESR_CYCLE_TELEMETRY_LOGGER_H_
