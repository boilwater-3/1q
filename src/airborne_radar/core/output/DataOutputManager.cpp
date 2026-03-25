#include "airborne_radar/core/output/DataOutputManager.h"

#include <cstddef>

#include "common/output/OutputFrameUtils.h"

namespace airborne_radar {
namespace core {
namespace output {

common::TrackOutputFrame DataOutputManager::BuildTrackOutputFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id,
    const common::DecisionTrackSnapshotList& track_snapshots) const {
  common::TrackOutputFrame frame;
  oneq::internal::output::SetCycleAndBatch(frame, cycle_index, batch_id);
  frame.tracks = track_snapshots;
  frame.published_track_count = track_snapshots.size();
  frame.confirmed_track_count = oneq::internal::output::CountMatching(
      track_snapshots, [](const common::DecisionTrackSnapshot& snapshot) {
        return snapshot.state.status == common::DecisionTrackStatus::kConfirmed;
      });
  frame.contains_lost_tracks =
      oneq::internal::output::CountMatching(
          track_snapshots, [](const common::DecisionTrackSnapshot& snapshot) {
            return snapshot.state.status == common::DecisionTrackStatus::kLost;
          }) > 0U;
  return frame;
}

common::DecisionInputFrame DataOutputManager::BuildDecisionInputFrame(
    const common::TrackOutputFrame& track_output_frame,
    const common::EccmSourceInfo& eccm_source_info,
    const common::AssociationQualityInfo& association_quality_info,
    const common::PerceptionQualityInfo& perception_quality_info) const {
  common::DecisionInputFrame frame;
  frame.cycle_index = track_output_frame.cycle_index;
  frame.batch_id = track_output_frame.batch_id;
  frame.environment_jamming_detected = eccm_source_info.has_jamming_signal;
  frame.eccm_source_info = eccm_source_info;
  frame.association_quality_info = association_quality_info;
  frame.perception_quality_info = perception_quality_info;
  frame.tracks = track_output_frame.tracks;
  return frame;
}

}  // namespace output
}  // namespace core
}  // namespace airborne_radar
