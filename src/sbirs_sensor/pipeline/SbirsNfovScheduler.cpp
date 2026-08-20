/**
 * @file SbirsNfovScheduler.cpp
 * @brief NFOV 多通道资源调度器实现。
 */

#include "sbirs_sensor/pipeline/SbirsNfovScheduler.h"

#include <algorithm>

namespace sbirs_sensor {
namespace pipeline {

SbirsNfovScheduler::SbirsNfovScheduler(int max_concurrent_locks)
    : max_locks_(max_concurrent_locks < 1 ? 1 : max_concurrent_locks) {}

bool SbirsNfovScheduler::IsLocked(std::uint64_t target_id) const {
  return target_to_channel_.find(target_id) != target_to_channel_.end();
}

int SbirsNfovScheduler::ChannelOf(std::uint64_t target_id) const {
  const auto it = target_to_channel_.find(target_id);
  return it == target_to_channel_.end() ? -1 : it->second;
}

int SbirsNfovScheduler::Acquire(std::uint64_t target_id) {
  if (IsLocked(target_id)) {
    return ChannelOf(target_id);
  }
  if (static_cast<int>(target_to_channel_.size()) >= max_locks_) {
    return -1;
  }
  // 最小可用编号分配：扫描 0..max_locks_-1，取首个未占用的编号。
  // 通道数有上限且通常较小，线性扫描足够且确定性明确。
  for (int channel = 0; channel < max_locks_; ++channel) {
    bool occupied = false;
    for (const auto& entry : target_to_channel_) {
      if (entry.second == channel) {
        occupied = true;
        break;
      }
    }
    if (!occupied) {
      target_to_channel_[target_id] = channel;
      return channel;
    }
  }
  return -1;  // 不可达：余量检查保证有空闲通道
}

void SbirsNfovScheduler::Release(std::uint64_t target_id) {
  target_to_channel_.erase(target_id);
}

void SbirsNfovScheduler::Clear() { target_to_channel_.clear(); }

std::vector<const SbirsCandidate*> SbirsNfovScheduler::SelectForAcquisition(
    std::vector<SbirsCandidate>& candidates) {
  std::vector<const SbirsCandidate*> selected;
  const std::size_t available =
      max_locks_ > static_cast<int>(target_to_channel_.size())
          ? static_cast<std::size_t>(max_locks_ - static_cast<int>(target_to_channel_.size()))
          : 0U;
  if (available == 0U) {
    return selected;
  }
  // design 2.6 优先级：SNR 降序 → 距离升序 → target_id 升序（稳定，保证 replay 可复现）。
  std::sort(candidates.begin(), candidates.end(), [](const SbirsCandidate& lhs, const SbirsCandidate& rhs) {
    if (lhs.snr != rhs.snr) {
      return lhs.snr > rhs.snr;
    }
    if (lhs.range_m != rhs.range_m) {
      return lhs.range_m < rhs.range_m;
    }
    return lhs.target->target_id < rhs.target->target_id;
  });
  for (const SbirsCandidate& candidate : candidates) {
    if (selected.size() >= available) {
      break;
    }
    if (!IsLocked(candidate.target->target_id)) {
      selected.push_back(&candidate);
    }
  }
  return selected;
}

SbirsNfovSchedulerSnapshot SbirsNfovScheduler::Capture() const {
  SbirsNfovSchedulerSnapshot snapshot;
  snapshot.target_to_channel = target_to_channel_;
  return snapshot;
}

void SbirsNfovScheduler::Restore(const SbirsNfovSchedulerSnapshot& snapshot) {
  target_to_channel_ = snapshot.target_to_channel;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
