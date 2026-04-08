/**
 * @file CycleTelemetryLogger.h
 * @brief 封装单周期遥测日志输出职责。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_CYCLE_TELEMETRY_LOGGER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_CYCLE_TELEMETRY_LOGGER_H_

#include <cstddef>
#include <cstdint>

#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace extension {

/**
 * @brief CycleTelemetryPayload 聚合单周期日志所需的全部字段。
 */
struct CycleTelemetryPayload {
  const oneq::internal::runtime::RuntimeCycleStamp& stamp;
  std::size_t input_target_count;
  std::size_t decision_track_count;
  std::size_t applied_directive_count;
  bool environment_jamming_detected;
  std::uint64_t profile_version;
  const model::PerceptionQualityInfo& perception_quality_info;
  const extension::AssociationQualityMetrics& association_metrics;

  CycleTelemetryPayload(const oneq::internal::runtime::RuntimeCycleStamp& stamp_,
                        std::size_t input_target_count_,
                        std::size_t decision_track_count_,
                        std::size_t applied_directive_count_,
                        bool environment_jamming_detected_,
                        std::uint64_t profile_version_,
                        const model::PerceptionQualityInfo& perception_quality_info_,
                        const extension::AssociationQualityMetrics& association_metrics_)
      : stamp(stamp_),
        input_target_count(input_target_count_),
        decision_track_count(decision_track_count_),
        applied_directive_count(applied_directive_count_),
        environment_jamming_detected(environment_jamming_detected_),
        profile_version(profile_version_),
        perception_quality_info(perception_quality_info_),
        association_metrics(association_metrics_) {}
};

/**
 * @brief 无状态周期遥测日志输出工具类。
 */
class CycleTelemetryLogger {
 public:
  /**
   * @brief 将单周期摘要输出为结构化 debug 日志。
   * @param payload 本周期日志载荷。
   */
  static void LogCycleSummary(const CycleTelemetryPayload& payload);
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_CYCLE_TELEMETRY_LOGGER_H_
