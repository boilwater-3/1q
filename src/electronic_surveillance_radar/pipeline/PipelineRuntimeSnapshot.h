/**
 * @file PipelineRuntimeSnapshot.h
 * @brief 定义 ESR 流水线运行态快照的私有数据结构及其类型安全访问器。
 *
 * InterceptPipelineRuntimeState::snapshot 在公共 API 中为 shared_ptr<const void>，
 * 以保持接口不透明。本文件集中管理所有 static_cast，外部代码仅通过
 * CapturePipelineSnapshot / RestorePipelineSnapshot 访问。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_PIPELINE_RUNTIME_SNAPSHOT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_PIPELINE_RUNTIME_SNAPSHOT_H_

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief ESR 流水线运行态私有快照，封装需要跨周期持久化的可变状态。
 */
struct PipelineRuntimeSnapshot {
  std::mt19937 rng;                                 /**< 随机引擎状态 */
  std::uint64_t next_observation_id{1U};             /**< 观测 ID 分配器 */
  std::uint64_t next_hypothesis_id{1U};              /**< 假设 ID 分配器 */
  std::vector<HypothesisAssociator::TrackState> tracks; /**< 关联器轨迹状态 */
};

/**
 * @brief 从 InterceptPipelineRuntimeState 中提取类型安全的快照指针。
 * @param state 运行时状态。
 * @return 快照只读指针；若 snapshot 为空则返回 nullptr。
 */
inline const PipelineRuntimeSnapshot* RestorePipelineSnapshot(
    const extension::InterceptPipelineRuntimeState& state) {
  return static_cast<const PipelineRuntimeSnapshot*>(state.snapshot.get());
}

/**
 * @brief 将类型安全的快照写入 InterceptPipelineRuntimeState。
 * @param state 运行时状态（in-out）。
 * @param snapshot 待写入的快照。
 */
inline void CapturePipelineSnapshot(
    extension::InterceptPipelineRuntimeState& state,
    std::shared_ptr<const PipelineRuntimeSnapshot> snapshot) {
  state.snapshot = std::move(snapshot);
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_PIPELINE_RUNTIME_SNAPSHOT_H_
