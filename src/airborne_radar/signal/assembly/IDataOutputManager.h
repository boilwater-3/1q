/**
 * @file IDataOutputManager.h
 * @brief 定义数据输出管理服务抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSEMBLY_I_DATA_OUTPUT_MANAGER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSEMBLY_I_DATA_OUTPUT_MANAGER_H_

#include <cstdint>

#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"

namespace airborne_radar {
namespace signal {
namespace assembly {

/**
 * @brief 负责中性输出帧与决策输入帧的装配。
 */
class IDataOutputManager {
 public:
  virtual ~IDataOutputManager() = default;

  /**
   * @brief 使用轨迹快照装配中性输出帧。
   * @param cycle_index 当前周期号。
   * @param batch_id 当前批号。
   * @param track_snapshots 当前周期导出的轨迹快照列表。
   * @return 填充完成的中性输出帧。
   */
  virtual output::TrackOutputFrame BuildTrackOutputFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
      const model::DecisionTrackSnapshotList& track_snapshots) const = 0;

  /**
   * @brief 使用中性输出帧和周期摘要装配决策输入帧。
   * @param track_output_frame 当前周期中性输出帧。
   * @param eccm_source_info 供 ECCM 消费的干扰事实摘要。
   * @param association_quality_info 供决策层消费的关联质量摘要。
   * @param perception_quality_info 供决策层消费的探测质量摘要。
   * @return 填充完成的决策输入帧。
   */
  virtual model::DecisionInputFrame BuildDecisionInputFrame(
      const output::TrackOutputFrame& track_output_frame,
      const model::EccmSourceInfo& eccm_source_info,
      const model::AssociationQualityInfo& association_quality_info,
      const model::PerceptionQualityInfo& perception_quality_info) const = 0;
};

}  // namespace assembly
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_ASSEMBLY_I_DATA_OUTPUT_MANAGER_H_
