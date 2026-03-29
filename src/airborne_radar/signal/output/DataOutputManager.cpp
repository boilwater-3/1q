#include "airborne_radar/signal/output/DataOutputManager.h"

#include <cstddef>

#include "common/output/OutputFrameUtils.h"

namespace airborne_radar {
namespace signal {
namespace output {

common::output::TrackOutputFrame DataOutputManager::BuildTrackOutputFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id,
    const common::model::DecisionTrackSnapshotList& track_snapshots) const {
  common::output::TrackOutputFrame frame;
  oneq::internal::output::SetCycleAndBatch(frame, cycle_index, batch_id);
  frame.tracks = track_snapshots;
  frame.published_track_count = track_snapshots.size();
  frame.confirmed_track_count = oneq::internal::output::CountMatching(
      track_snapshots, [](const common::model::DecisionTrackSnapshot& snapshot) {
        return snapshot.state.status == common::model::DecisionTrackStatus::kConfirmed;
      });
  frame.contains_lost_tracks =
      oneq::internal::output::CountMatching(
          track_snapshots, [](const common::model::DecisionTrackSnapshot& snapshot) {
            return snapshot.state.status == common::model::DecisionTrackStatus::kLost;
          }) > 0U;
  return frame;
}

common::model::DecisionInputFrame DataOutputManager::BuildDecisionInputFrame(
    const common::output::TrackOutputFrame& track_output_frame,
    const common::model::EccmSourceInfo& eccm_source_info,
    const common::model::AssociationQualityInfo& association_quality_info,
    const common::model::PerceptionQualityInfo& perception_quality_info) const {
  common::model::DecisionInputFrame frame;
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
}  // namespace signal
}  // namespace airborne_radar
