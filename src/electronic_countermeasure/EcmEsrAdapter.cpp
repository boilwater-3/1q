#include "1q/electronic_countermeasure/EcmEsrAdapter.h"

#include <cmath>
#include <set>

namespace electronic_countermeasure {
namespace session {
namespace {

float ThreatLevelScore(electronic_surveillance_radar::session::EsrThreatLevel level) {
  switch (level) {
    case electronic_surveillance_radar::session::EsrThreatLevel::kHigh:
      return 1.0f;
    case electronic_surveillance_radar::session::EsrThreatLevel::kMedium:
      return 0.66f;
    case electronic_surveillance_radar::session::EsrThreatLevel::kLow:
      return 0.33f;
  }
  return 0.0f;
}

bool IsValidHypothesis(
    const electronic_surveillance_radar::session::EmitterHypothesis& hypothesis) {
  return std::isfinite(hypothesis.estimated_center_frequency_hz) &&
         hypothesis.estimated_center_frequency_hz > 0.0 &&
         std::isfinite(hypothesis.estimated_bandwidth_hz) &&
         hypothesis.estimated_bandwidth_hz > 0.0 &&
         std::isfinite(hypothesis.estimated_pri_s) && hypothesis.estimated_pri_s >= 0.0 &&
         std::isfinite(hypothesis.estimated_pulse_width_s) &&
         hypothesis.estimated_pulse_width_s >= 0.0 &&
         std::isfinite(hypothesis.center_frequency_std_hz) &&
         hypothesis.center_frequency_std_hz >= 0.0 &&
         std::isfinite(hypothesis.bandwidth_std_hz) && hypothesis.bandwidth_std_hz >= 0.0 &&
         std::isfinite(hypothesis.bearing_az_deg) &&
         std::isfinite(hypothesis.bearing_el_deg) &&
         std::isfinite(hypothesis.bearing_std_deg) && hypothesis.bearing_std_deg >= 0.0f &&
         std::isfinite(hypothesis.confidence) && hypothesis.confidence >= 0.0f &&
         hypothesis.confidence <= 1.0f;
}

}  // namespace

bool TryBuildEcmSensorObservationFrame(
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses,
    std::uint64_t source_esr_batch_id, EcmSensorObservationFrame* output) {
  if (output == nullptr || source_esr_batch_id == 0U) {
    // A fresh frame must carry a real ESR published batch_id (provenance); zero
    // means "no real ESR success batch" and is rejected so callers cannot supply
    // an unbound frame. See design.md §2 prototype limitation.
    return false;
  }
  EcmSensorObservationFrame candidate;
  candidate.source_esr_batch_id = source_esr_batch_id;
  std::set<std::uint64_t> hypothesis_ids;
  for (const electronic_surveillance_radar::session::EmitterHypothesis& hypothesis : hypotheses) {
    if (!IsValidHypothesis(hypothesis) ||
        !hypothesis_ids.insert(hypothesis.hypothesis_id).second) {
      return false;
    }
    EcmSensorObservation observation;
    observation.source_hypothesis_id = hypothesis.hypothesis_id;
    observation.estimated_center_frequency_hz = hypothesis.estimated_center_frequency_hz;
    observation.estimated_bandwidth_hz = hypothesis.estimated_bandwidth_hz;
    observation.estimated_pri_s = hypothesis.estimated_pri_s;
    observation.estimated_pulse_width_s = hypothesis.estimated_pulse_width_s;
    observation.center_frequency_std_hz = hypothesis.center_frequency_std_hz;
    observation.bandwidth_std_hz = hypothesis.bandwidth_std_hz;
    observation.bearing_az_deg = hypothesis.bearing_az_deg;
    observation.bearing_el_deg = hypothesis.bearing_el_deg;
    observation.bearing_std_deg = hypothesis.bearing_std_deg;
    observation.confidence = hypothesis.confidence;
    observation.threat_score =
        ThreatLevelScore(hypothesis.threat_level) * hypothesis.confidence;
    candidate.observations.push_back(observation);
  }
  *output = candidate;
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
