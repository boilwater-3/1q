/**
 * @file RirTrackPool.cpp
 * @brief RIR 航迹对象池实现（阶段 2-T N3）。
 */

#include "remote_identification_radar/tracking/RirTrackPool.h"

#include "common/logging/ProjectLog.h"

namespace remote_identification_radar {
namespace tracking {

RirTrackPool::RirTrackPool(std::size_t prewarm_count, std::size_t max_cached_objects)
    : pool_(), free_list_(), max_cached_objects_(max_cached_objects) {
  free_list_.reserve(prewarm_count);
  for (std::size_t i = 0; i < prewarm_count; ++i) {
    RirTrackState* track = pool_.construct();
    if (track == nullptr) {
      // 中译：预充（prewarm）构造航迹对象失败（已备/目标/在用/缓存数）。
      // 标识：内存池分配失败——预充中断但构造器继续，空闲列表少于预期，
      //       后续 Acquire 可能需实时构造。
      PROJECT_LOG_ERROR(
          "[RirTrackPool] prewarm construct failed: prepared={} target_prewarm={} in_use={} "
          "cached={}",
          free_list_.size(), prewarm_count, in_use_count_, free_list_.size());
      break;
    }
    free_list_.push_back(track);
  }
}

RirTrackState* RirTrackPool::Acquire() {
  RirTrackState* track = nullptr;
  if (!free_list_.empty()) {
    track = free_list_.back();
    free_list_.pop_back();
  } else {
    track = pool_.construct();
    if (track == nullptr) {
      // 中译：取用（Acquire）时构造航迹对象失败（在用/缓存/容量）。
      // 标识：内存池分配失败——返回空指针，调用方应处理获取失败。
      PROJECT_LOG_ERROR("[RirTrackPool] acquire construct failed: in_use={} cached={} capacity={}",
                        in_use_count_, free_list_.size(), Capacity());
    }
  }

  if (track != nullptr) {
    const std::pair<std::unordered_set<RirTrackState*>::iterator, bool> inserted =
        in_use_tracks_.insert(track);
    if (!inserted.second) {
      // 中译：Acquire 检测到重复的在用指针，返回空。
      // 标识：对象池一致性保护——同一航迹对象被重复取用说明池状态损坏。
      PROJECT_LOG_ERROR("[RirTrackPool] Acquire detected duplicate in-use pointer: {}",
                        static_cast<void*>(track));
      return nullptr;
    }
    ++in_use_count_;
  }
  return track;
}

void RirTrackPool::Release(RirTrackState* track) {
  if (track == nullptr) {
    return;
  }

  if (in_use_tracks_.erase(track) == 0U) {
    // 中译：Release 拒绝了未知或重复释放的指针。
    // 标识：对象池一致性保护——释放未在用的对象说明池状态损坏。
    PROJECT_LOG_ERROR("[RirTrackPool] Release rejected unknown or double-released pointer: {}",
                      static_cast<void*>(track));
    return;
  }

  // erase 成功 ⟹ 指针原在集合中，正常不变量下计数必 ≥ 1。计数为 0 说明
  // 集合/计数已失配（并发误用等）：按集合大小自愈计数（本次 erase 已计入），
  // 让对象继续走正常归还路径——而不是把指针重插集合，那会使 set 与 count
  // 永久相差 1 且对象永远无法归还（滞留泄漏）。
  if (in_use_count_ == 0) {
    // 中译：Release 时在用计数与注册集合失配，计数已按集合大小修复，本次仍正常归还。
    // 标识：对象池一致性自愈——失配通常来自并发误用；修复后 count == set.size()
    //       不变量恢复，避免指针滞留与容量统计永久失真。
    PROJECT_LOG_ERROR(
        "[RirTrackPool] Release found in-use count/registry mismatch; count repaired to "
        "registry size, object still released normally");
    in_use_count_ = in_use_tracks_.size();
  } else {
    --in_use_count_;
  }

  if (free_list_.size() < max_cached_objects_) {
    free_list_.push_back(track);
    return;
  }

  // 超过空闲缓存上限时执行显式析构，避免空闲列表无限膨胀。
  pool_.destroy(track);
}

std::size_t RirTrackPool::Capacity() const { return in_use_count_ + free_list_.size(); }

}  // namespace tracking
}  // namespace remote_identification_radar
