/**
 * @file DataOutputManager.h
 * @brief 定义默认数据输出管理服务实现。
 */

#ifndef AIRBORNE_RADAR_CORE_OUTPUT_DATA_OUTPUT_MANAGER_H_
#define AIRBORNE_RADAR_CORE_OUTPUT_DATA_OUTPUT_MANAGER_H_

#include "airborne_radar/core/output/IDataOutputManager.h"

namespace airborne_radar {
namespace core {
namespace output {

/// @brief DataOutputManager 实现中性输出帧与决策输入帧装配逻辑。
class DataOutputManager : public IDataOutputManager {
public:
  /// @brief 使用轨迹快照装配中性输出帧。
  /// @param cycle_index 当前周期号。
  /// @param batch_id 当前批号。
  /// @param track_snapshots 当前周期导出的轨迹快照列表。
  /// @return 填充完成的中性输出帧。
  common::TrackOutputFrame BuildTrackOutputFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
      const common::DecisionTrackSnapshotList& track_snapshots) const override;

  /// @brief 使用中性输出帧和周期摘要装配决策输入帧。
  /// @param track_output_frame 当前周期中性输出帧。
  /// @param eccm_source_info 供 ECCM 消费的干扰事实摘要。
  /// @param association_quality_info 供决策层消费的关联质量摘要。
  /// @param perception_quality_info 供决策层消费的探测质量摘要。
  /// @return 填充完成的决策输入帧。
  common::DecisionInputFrame BuildDecisionInputFrame(
      const common::TrackOutputFrame& track_output_frame,
      const common::EccmSourceInfo& eccm_source_info,
      const common::AssociationQualityInfo& association_quality_info,
      const common::PerceptionQualityInfo& perception_quality_info) const override;
};

} // namespace output
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_OUTPUT_DATA_OUTPUT_MANAGER_H_
