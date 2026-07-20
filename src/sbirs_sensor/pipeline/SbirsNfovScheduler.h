/**
 * @file SbirsNfovScheduler.h
 * @brief NFOV 多通道资源调度器：候选排序、通道分配与释放。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_SCHEDULER_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_SCHEDULER_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace pipeline {

/**
 * @brief 单个 WFOV 候选的调度中间结构。
 * @note 由 pipeline 在 WFOV 发现阶段填充，交由调度器排序与选取（design 2.6）。
 */
struct SbirsCandidate {
  const session::SbirsSceneTarget* target{nullptr};
  float azimuth_deg{0.0f};
  float elevation_deg{0.0f};
  float delayed_truth_azimuth_deg{0.0f};   ///< cue 延迟后的仿真真值方位角，仅用于捕获判定
  float delayed_truth_elevation_deg{0.0f}; ///< cue 延迟后的仿真真值俯仰角，仅用于捕获判定
  float measured_azimuth_deg{0.0f};
  float measured_elevation_deg{0.0f};
  float command_azimuth_deg{0.0f};   ///< 仅由 WFOV 测量历史生成的 NFOV 命令方位角
  float command_elevation_deg{0.0f}; ///< 仅由 WFOV 测量历史生成的 NFOV 命令俯仰角
  double range_m{0.0};        ///< 真值距离（调度优先级用）
  double measured_range_m{0.0};  ///< 带误差距离（NFOV cue 与 attribution 诊断用）
  float angular_rate_deg_per_sec{0.0f}; ///< Sensor-like 成功观测的动态滞后输入
  double snr{0.0};
};

/**
 * @brief NFOV 资源调度器快照，用于 pipeline validated checkpoint 与确定性 continuation。
 * @note 仅包含目标→通道的分配关系；滤波/IMM 状态由 pipeline 各自管理。
 */
struct SbirsNfovSchedulerSnapshot {
  std::map<std::uint64_t, int> target_to_channel{}; ///< 已锁定目标 ID 到 NFOV 通道编号的映射 */
};

/**
 * @brief NFOV 多通道资源调度器。
 * @details 在 `max_concurrent_locks` 上限内，按 design 2.6 优先级规则
 *          （SNR 降序 → 距离升序 → target_id 升序）从 WFOV 候选中选取目标进入首次捕获，
 *          并为每个被锁定的目标分配一个 NFOV 通道编号。通道编号采用最小可用编号分配、
 *          释放后回收，保证相同输入下分配结果确定（replay 可复现）。
 */
class SbirsNfovScheduler {
 public:
  /**
   * @brief 构造调度器。
   * @param[in] max_concurrent_locks 最大并发 NFOV 锁定通道数（>=1）
   */
  explicit SbirsNfovScheduler(int max_concurrent_locks);

  /** @return 最大并发 NFOV 锁定通道数。 */
  int max_locks() const { return max_locks_; }

  /** @return 当前已锁定（占用 NFOV 通道）的目标数。 */
  std::size_t LockedCount() const { return target_to_channel_.size(); }

  /**
   * @brief 按优先级排序候选，并在通道余量内选出可进入首次捕获的目标。
   * @param[in,out] candidates WFOV 候选列表（就地排序，稳定规则保证 replay 可复现）
   * @return 本周期可进入首次捕获的候选指针列表（按优先级排序，受通道上限截断）
   * @note 已锁定目标的候选不会重复入选；通道满时返回空。
   */
  std::vector<const SbirsCandidate*> SelectForAcquisition(std::vector<SbirsCandidate>& candidates);

  /** @return 目标是否已占用 NFOV 通道。 */
  bool IsLocked(std::uint64_t target_id) const;

  /**
   * @brief 为目标分配一个 NFOV 通道编号。
   * @param[in] target_id 待锁定的目标 ID
   * @return 分配到的通道编号（>=0）；通道已满或目标已锁定时返回 -1
   */
  int Acquire(std::uint64_t target_id);

  /**
   * @brief 释放目标占用的 NFOV 通道（目标消失、NIS 丢锁或传感器进入 standby）。
   * @param[in] target_id 待释放的目标 ID
   */
  void Release(std::uint64_t target_id);

  /**
   * @return 目标占用的 NFOV 通道编号；未占用时返回 -1。
   */
  int ChannelOf(std::uint64_t target_id) const;

  /** @brief 清空所有通道分配（standby / 重置）。 */
  void Clear();

  /** @return 当前调度状态快照，用于 pipeline checkpoint 与状态恢复测试。 */
  SbirsNfovSchedulerSnapshot Capture() const;

  /**
   * @brief 从快照恢复调度状态。
   * @param[in] snapshot 待恢复的状态快照
   */
  void Restore(const SbirsNfovSchedulerSnapshot& snapshot);

 private:
  int max_locks_{1};
  std::map<std::uint64_t, int> target_to_channel_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_NFOV_SCHEDULER_H_
