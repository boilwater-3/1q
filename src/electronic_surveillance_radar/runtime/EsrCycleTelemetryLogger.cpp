#include "electronic_surveillance_radar/runtime/EsrCycleTelemetryLogger.h"

#include "common/logging/ProjectLog.h"

namespace electronic_surveillance_radar {
namespace runtime {

void EsrCycleTelemetryLogger::LogCycleSummary(const EsrCycleTelemetryPayload& payload) {
  PROJECT_LOG_DEBUG(
      "[EsrController] cycle telemetry: cycle_index={} batch_id={} "
      "sensor_enabled={} input_emitters={} raw_observations={} "
      "observations={} clusters={} hypotheses={} "
      "truth_associations={} matched_truth={}",
      payload.stamp.cycle_index, payload.stamp.batch_id, payload.sensor_enabled ? "true" : "false",
      payload.input_emitter_count, payload.raw_observation_count, payload.observation_count,
      payload.cluster_count, payload.hypothesis_count, payload.truth_association_count,
      payload.matched_truth_count);
}

}  // namespace runtime


}  // namespace electronic_surveillance_radar
