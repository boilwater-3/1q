/**
 * @file ObjectPool.h
 * @brief Boost object_pool + 自由表 + 在用集合的通用航迹/对象池（header-only 模板）。
 */

#ifndef COMMON_TRACKING_OBJECT_POOL_H_
#define COMMON_TRACKING_OBJECT_POOL_H_

#include <boost/pool/object_pool.hpp>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/logging/ProjectLog.h"

namespace oneq {
namespace common {
namespace tracking {

/**
 * @brief 轻量对象池：预热、Acquire/Release、双重释放拒绝。
 * @tparam ObjectT 池化对象类型（需默认可构造）。
 */
template <typename ObjectT>
class ObjectPool {
 public:
  /**
   * @brief 构造对象池。
   * @param prewarm_count 预热对象数量。
   * @param max_cached_objects 空闲缓存上限（超过后归还即销毁）。
   * @param log_tag 日志前缀（区分 AR/RIR 调用方）。
   */
  explicit ObjectPool(std::size_t prewarm_count = 128, std::size_t max_cached_objects = 4096,
                      std::string log_tag = "ObjectPool")
      : pool_(),
        free_list_(),
        in_use_tracks_(),
        max_cached_objects_(max_cached_objects),
        in_use_count_(0),
        log_tag_(std::move(log_tag)) {
    free_list_.reserve(prewarm_count);
    for (std::size_t i = 0; i < prewarm_count; ++i) {
      ObjectT* object = pool_.construct();
      if (object == nullptr) {
        // 中译：预充构造对象失败（已备/目标/在用/缓存数）。
        // 标识：内存池分配失败——预充中断但构造器继续。
        PROJECT_LOG_ERROR(
            "[{}] prewarm construct failed: prepared={} target_prewarm={} in_use={} cached={}",
            log_tag_, free_list_.size(), prewarm_count, in_use_count_, free_list_.size());
        break;
      }
      free_list_.push_back(object);
    }
  }

  ObjectT* Acquire() {
    ObjectT* object = nullptr;
    if (!free_list_.empty()) {
      object = free_list_.back();
      free_list_.pop_back();
    } else {
      object = pool_.construct();
      if (object == nullptr) {
        // 中译：取用时构造对象失败。
        // 标识：内存池分配失败——返回空指针。
        PROJECT_LOG_ERROR("[{}] acquire construct failed: in_use={} cached={} capacity={}",
                          log_tag_, in_use_count_, free_list_.size(), Capacity());
      }
    }

    if (object != nullptr) {
      const std::pair<typename std::unordered_set<ObjectT*>::iterator, bool> inserted =
          in_use_tracks_.insert(object);
      if (!inserted.second) {
        // 中译：Acquire 检测到重复的在用指针，返回空。
        // 标识：对象池一致性保护。
        PROJECT_LOG_ERROR("[{}] Acquire detected duplicate in-use pointer: {}", log_tag_,
                          static_cast<void*>(object));
        return nullptr;
      }
      ++in_use_count_;
    }
    return object;
  }

  void Release(ObjectT* object) {
    if (object == nullptr) {
      return;
    }

    if (in_use_tracks_.erase(object) == 0U) {
      // 中译：Release 拒绝未知或重复释放的指针。
      // 标识：对象池一致性保护。
      PROJECT_LOG_ERROR("[{}] Release rejected unknown or double-released pointer: {}", log_tag_,
                        static_cast<void*>(object));
      return;
    }

    if (in_use_count_ == 0) {
      // 中译：Release 时在用计数与注册集合失配，计数已按集合大小修复。
      // 标识：对象池一致性自愈。
      PROJECT_LOG_ERROR(
          "[{}] Release found in-use count/registry mismatch; count repaired to registry "
          "size, object still released normally",
          log_tag_);
      in_use_count_ = in_use_tracks_.size();
    } else {
      --in_use_count_;
    }

    if (free_list_.size() < max_cached_objects_) {
      free_list_.push_back(object);
      return;
    }
    pool_.destroy(object);
  }

  std::size_t Capacity() const { return in_use_count_ + free_list_.size(); }
  std::size_t InUseCount() const { return in_use_count_; }
  std::size_t FreeCount() const { return free_list_.size(); }

 private:
  boost::object_pool<ObjectT> pool_;
  std::vector<ObjectT*> free_list_;
  std::unordered_set<ObjectT*> in_use_tracks_;
  std::size_t max_cached_objects_{4096};
  std::size_t in_use_count_{0};
  std::string log_tag_;
};

}  // namespace tracking
}  // namespace common
}  // namespace oneq

#endif  // COMMON_TRACKING_OBJECT_POOL_H_
