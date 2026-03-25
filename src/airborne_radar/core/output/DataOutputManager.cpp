#include "airborne_radar/core/output/DataOutputManager.h"

#include <cstddef>

namespace airborne_radar {
namespace core {
namespace output {

namespace {

/**
 * @brief 统计输出帧内的已确认轨迹数量。
 * @param track_snapshots 当前周期轨迹快照列表。
 * @return 已确认轨迹数量。
 */
std::size_t CountConfirmedTracks(const common::DecisionTrackSnapshotList& track_snapshots) {
  std::size_t confirmed_track_count = 0U;
  for (std::size_t i = 0; i < track_snapshots.size(); ++i) {
    if (track_snapshots[i].state.status == common::DecisionTrackStatus::kConfirmed) {
      ++confirmed_track_count;
    }
  }
  return confirmed_track_count;
}

/**
 * @brief 判断输出帧内是否包含 lost 轨迹。
 * @param track_snapshots 当前周期轨迹快照列表。
 * @return 若包含 lost 轨迹则返回 `true`。
 */
bool ContainsLostTracks(const common::DecisionTrackSnapshotList& track_snapshots) {
  for (std::size_t i = 0; i < track_snapshots.size(); ++i) {
    if (track_snapshots[i].state.status == common::DecisionTrackStatus::kLost) {
      return true;
    }
  }
  return false;
}

}  // namespace

common::TrackOutputFrame DataOutputManager::BuildTrackOutputFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id,
    const common::DecisionTrackSnapshotList& track_snapshots) const {
  common::TrackOutputFrame frame;
  frame.cycle_index = cycle_index;
  frame.batch_id = batch_id;
  frame.tracks = track_snapshots;
  frame.published_track_count = track_snapshots.size();
  frame.confirmed_track_count = CountConfirmedTracks(track_snapshots);
  frame.contains_lost_tracks = ContainsLostTracks(track_snapshots);
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
