#include "1q/electronic_surveillance_radar/session/EsrCycleOutputBuilder.h"

#include "1q/coordinate/position_transform.h"

namespace electronic_surveillance_radar {
namespace session {

bool EsrCycleOutputBuilder::Build(const EsrExternalPoseInput& platform, const EsrOutputFrame& frame,
                                  EsrExternalOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  EsrCoordinateReference reference;
  if (!oneq::coordinate::TryEcefToLla(platform.platform_position_ecef_m, &reference.origin_lla)) {
    return false;
  }
  reference.frame_attitude_deg = platform.platform_attitude_deg;

  EsrPoseState platform_pose;
  if (!TryMakeEsrPoseFromExternalKinematics(platform, reference, &platform_pose)) {
    return false;
  }
  return Build(reference, platform_pose, frame, output);
}

bool EsrCycleOutputBuilder::Build(const EsrCoordinateReference& reference,
                                  const EsrPoseState& platform_pose, const EsrOutputFrame& frame,
                                  EsrExternalOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  output->cycle_index = frame.cycle_index;
  output->batch_id = frame.batch_id;
  output->truth_evaluation_output = frame.truth_evaluation_output;
  output->observations.clear();
  output->observations.reserve(frame.observation_output.observations.size());
  for (std::size_t i = 0; i < frame.observation_output.observations.size(); ++i) {
    EsrExternalObservation observation;
    if (!TryMakeExternalObservationFromRecord(frame.observation_output.observations[i], reference,
                                              platform_pose, &observation)) {
      return false;
    }
    output->observations.push_back(observation);
  }

  output->hypotheses.clear();
  output->hypotheses.reserve(frame.emitter_output.hypotheses.size());
  for (std::size_t i = 0; i < frame.emitter_output.hypotheses.size(); ++i) {
    EsrExternalEmitterHypothesis hypothesis;
    if (!TryMakeExternalHypothesisFromRecord(frame.emitter_output.hypotheses[i], reference,
                                             platform_pose, &hypothesis)) {
      return false;
    }
    output->hypotheses.push_back(hypothesis);
  }
  return true;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
