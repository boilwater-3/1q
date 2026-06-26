#include "1q/electro_optical_sensor/session/EosCycleOutputAdapter.h"

#include "1q/coordinate/position_transform.h"

namespace electro_optical_sensor {
namespace session {

bool EosCycleOutputAdapter::Build(const EosExternalPoseInput& platform, const EosOutputFrame& frame,
                                  EosExternalOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  oneq::coordinate::LocalFrameReference reference;
  if (!oneq::coordinate::TryEcefToLla(platform.platform_position_ecef_m, &reference.origin_lla)) {
    return false;
  }
  reference.frame_attitude_deg = platform.platform_attitude_deg;

  oneq::foundation::PoseState platform_pose;
  if (!TryMakeEosPoseFromExternalKinematics(platform, reference, &platform_pose)) {
    return false;
  }
  return Build(reference, platform_pose, frame, output);
}

bool EosCycleOutputAdapter::Build(const oneq::coordinate::LocalFrameReference& reference,
                                  const oneq::foundation::PoseState& platform_pose,
                                  const EosOutputFrame& frame, EosExternalOutputFrame* output) {
  if (output == nullptr) {
    return false;
  }

  output->cycle_index = frame.cycle_index;
  output->scan_azimuth_deg = frame.scan_azimuth_deg;
  output->detections.clear();
  output->detections.reserve(frame.detections.size());
  for (std::size_t i = 0; i < frame.detections.size(); ++i) {
    EosExternalDetectionRecord record;
    if (!TryMakeExternalDetectionFromRecord(frame.detections[i], reference, platform_pose,
                                            &record)) {
      return false;
    }
    output->detections.push_back(record);
  }
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
