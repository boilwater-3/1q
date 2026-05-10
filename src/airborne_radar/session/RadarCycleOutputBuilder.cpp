#include "1q/airborne_radar/session/RadarCycleOutputBuilder.h"

namespace airborne_radar {
namespace session {

bool RadarCycleOutputBuilder::Build(const RadarExternalPoseInput& platform,
                                    const TrackOutputFrame& frame,
                                    RadarExternalTrackOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  RadarLocalFrameReference reference;
  oneq::foundation::PoseState platform_pose;
  if (!TryMakeRadarPoseFromExternalKinematics(platform, &reference, &platform_pose)) {
    return false;
  }
  return Build(reference, platform_pose.velocity_mps, frame, output);
}

bool RadarCycleOutputBuilder::Build(const RadarLocalFrameReference& reference,
                                    oneq::foundation::Vector3f radar_local_velocity_mps,
                                    const TrackOutputFrame& frame,
                                    RadarExternalTrackOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  output->cycle_index = frame.cycle_index;
  output->batch_id = frame.batch_id;
  output->tracks.clear();
  output->tracks.reserve(frame.tracks.size());

  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    RadarExternalTrackKinematics track;
    if (!TryMakeExternalTrackFromSnapshot(frame.tracks[i], reference, radar_local_velocity_mps,
                                          &track)) {
      return false;
    }
    output->tracks.push_back(track);
  }
  return true;
}

}  // namespace session
}  // namespace airborne_radar
