#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"

#include "1q/airborne_radar/session/ArRadarFrameTransform.h"

namespace airborne_radar {
namespace session {

bool ArCycleOutputAdapter::Build(const ArPlatformInput& platform,
                                 const TrackOutputFrame& frame,
                                 ArExternalTrackOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  oneq::coordinate::LocalFrameReference reference;
  oneq::coordinate::Vector3d radar_local_velocity;
  const oneq::coordinate::EulerAnglesDeg zero_mount{};
  if (!TryMakeArPoseFromPlatform(platform, zero_mount, &reference, &radar_local_velocity)) {
    return false;
  }
  return Build(reference, radar_local_velocity, frame, output);
}

bool ArCycleOutputAdapter::Build(const oneq::coordinate::LocalFrameReference& reference,
                                 oneq::coordinate::Vector3d radar_local_velocity_mps,
                                 const TrackOutputFrame& frame,
                                 ArExternalTrackOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  output->cycle_index = frame.cycle_index;
  output->batch_id = frame.batch_id;
  output->tracks.clear();
  output->tracks.reserve(frame.tracks.size());

  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    ArExternalTrackKinematics track;
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
