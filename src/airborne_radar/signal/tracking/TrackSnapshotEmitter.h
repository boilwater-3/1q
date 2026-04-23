/**
 * @file TrackSnapshotEmitter.h
 * @brief 定义轨迹快照导出器，负责将活跃轨迹集转换为各类外部消费快照。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_SNAPSHOT_EMITTER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_SNAPSHOT_EMITTER_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"
#include "airborne_radar/signal/tracking/TrackState.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief 将生命周期管理器的活跃轨迹集转化为各类外部消费快照。
 *
 * 调用方须在每次 Update() 完成后调用 Refresh() 刷新内部快照，
 * 后续的 Build* 方法均基于刷新后的数据生成只读视图。
 */
class TrackSnapshotEmitter {
 public:
  /**
   * @brief 刷新内部快照缓存。
   * @param tracks_by_key 生命周期管理器的活跃轨迹表（key 为关联键）。
   */
  void Refresh(const std::unordered_map<std::uint64_t, TrackState*>& tracks_by_key);

  /**
   * @brief 导出供外围事件链路消费的轻量目标特征快照。
   * @return 可直接用于外部状态广播的目标特征列表。
   */
  model::TargetFeatureList BuildFeatureSnapshot() const;

  /**
   * @brief 导出供决策层消费的活跃轨迹快照。
   * @return 包含 tentative/confirmed/lost 状态且未回收的决策快照列表。
   */
  model::TrackStateSnapshotList BuildDecisionSnapshot() const;

  /**
   * @brief 导出完整的决策输入帧。
   * @param cycle_index 当前处理周期索引。
   * @param batch_id 本次批处理唯一 ID。
   * @param environment_jamming_detected 环境是否检测到大面积干扰。
   * @return 填充好的决策输入数据帧。
   */
  model::DecisionInputFrame BuildDecisionFrame(std::uint32_t cycle_index,
                                                       std::uint64_t batch_id,
                                                       bool environment_jamming_detected) const;

  /**
   * @brief 导出供关联阶段消费的轨迹种子。
   * @return 由当前未回收轨迹（tentative/confirmed/lost）构成的关联种子列表。
   */
  std::vector<AssociationTrackSeed> BuildAssociationSeeds() const;

 private:
  /** @brief 活跃轨迹条目（关联键 + 只读轨迹指针）。 */
  struct ActiveTrackEntry {
    std::uint64_t key{0};
    const TrackState* track{nullptr};
  };

  std::vector<ActiveTrackEntry> active_tracks_;
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_SNAPSHOT_EMITTER_H_
