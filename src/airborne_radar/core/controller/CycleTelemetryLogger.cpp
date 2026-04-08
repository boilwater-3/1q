#include "airborne_radar/core/controller/CycleTelemetryLogger.h"

#include "1q/airborne_radar/common/utils/JammingSemantics.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace core {
namespace controller {

namespace {

const char* JammingSemanticName(common::utils::JammingSemantic semantic) {
  switch (semantic) {
    case common::utils::JammingSemantic::kNoiseSuppression:
      return "noise";
    case common::utils::JammingSemantic::kDeception:
      return "deception";
    case common::utils::JammingSemantic::kRepeater:
      return "repeater";
    case common::utils::JammingSemantic::kMixed:
      return "mixed";
    case common::utils::JammingSemantic::kNone:
    default:
      return "none";
  }
}

}  // namespace

void CycleTelemetryLogger::LogCycleSummary(const CycleTelemetryPayload& payload) {
  PROJECT_LOG_DEBUG(
      "[RadarController] cycle summary: cycle_index={} batch_id={} "
      "input_targets={} decision_features={} directives={} "
      "jamming_detected={} profile_version={} detect_rate={:.3f} "
      "detect_stress={:.3f} assoc_priors={} assoc_detections={} "
      "assoc_matches={} assoc_new_tracks={} assoc_missed_tracks={} "
      "assoc_match_rate={:.3f} assoc_new_track_rate={:.3f} "
      "assoc_missed_rate={:.3f} assoc_mean_cost={:.3f} "
      "assoc_p95_cost={:.3f} assoc_jam_semantic={} "
      "assoc_jam_severity={:.3f} assoc_stress={:.3f}",
      payload.stamp.cycle_index, payload.stamp.batch_id, payload.input_target_count,
      payload.decision_track_count, payload.applied_directive_count,
      payload.environment_jamming_detected ? "true" : "false", payload.profile_version,
      payload.perception_quality_info.detection_rate,
      payload.perception_quality_info.detection_stress,
      payload.association_metrics.prior_track_count, payload.association_metrics.detection_count,
      payload.association_metrics.matched_count, payload.association_metrics.new_track_count,
      payload.association_metrics.missed_track_count, payload.association_metrics.match_rate,
      payload.association_metrics.new_track_rate, payload.association_metrics.missed_track_rate,
      payload.association_metrics.mean_match_cost, payload.association_metrics.p95_match_cost,
      JammingSemanticName(payload.association_metrics.dominant_jamming_semantic),
      payload.association_metrics.jamming_severity, payload.association_metrics.association_stress);
}

}  // namespace controller
}  // namespace core
}  // namespace airborne_radar
