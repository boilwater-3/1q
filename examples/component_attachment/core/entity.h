/**
 * @file entity.h
 * @brief 自定义实体-组件示例：实体（组件的挂载容器）。
 *
 * 实体按挂载序持有组件列表（unique_ptr 所有权）；Step 按挂载序步进全部
 * 组件——挂载序即周期执行序（本示例平台实体：Flight → ESR → [ECM] → AR
 * → EOS → Fusion，保证 ESR 假设先于 ECM、干扰经 rf_world 先于 AR 消费）。
 *
 * 组件间通信：
 *  - 周期内同步数据聚合：Find<T>()（dynamic_cast 类型化访问同实体组件）；
 *  - 跨周期通知/记录：World 的信号（Boost.Signals2，见 signals.h）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_CORE_ENTITY_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_CORE_ENTITY_H_

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "component.h"

namespace component_attachment {

using EntityId = std::uint64_t;

/**
 * @brief 实体：组件的挂载容器。
 *
 * 实体本身无逻辑（逻辑在组件）；由 World 创建并持有（创建序即步进序），
 * 名称供消费方按名查找。
 */
class Entity {
 public:
  /** @param[in] id 实体标识（World 递增分配，从 1 开始） */
  /** @param[in] name 实体名称（消费方约定唯一） */
  Entity(EntityId id, std::string name) : id_(id), name_(std::move(name)) {}

  ~Entity() {
    // 实体销毁 = 组件卸载：逐一通知 OnDetach（生命周期钩子对称）。
    for (auto& component : components_) {
      if (component != nullptr) {
        component->OnDetach();
      }
    }
  }

  Entity(const Entity&) = delete;
  Entity& operator=(const Entity&) = delete;

  /** @brief 实体标识。 */
  EntityId id() const { return id_; }

  /** @brief 实体名称。 */
  const std::string& name() const { return name_; }

  /**
   * @brief 挂载组件：按序追加并触发 OnAttach。
   * @return 组件裸指针（所有权归实体）。
   */
  Component* Attach(std::unique_ptr<Component> component) {
    Component* raw = component.get();
    components_.push_back(std::move(component));
    raw->OnAttach(*this);
    return raw;
  }

  /**
   * @brief 按 Name() 卸载第一个匹配组件（触发 OnDetach）。
   * @return 是否卸载成功（无匹配返回 false）。
   */
  bool DetachByName(const char* name) {
    for (auto it = components_.begin(); it != components_.end(); ++it) {
      if ((*it) != nullptr && std::strcmp((*it)->Name(), name) == 0) {
        (*it)->OnDetach();
        components_.erase(it);
        return true;
      }
    }
    return false;
  }

  /** @brief 类型化查找组件（dynamic_cast；无匹配返回 nullptr）。 */
  template <typename T>
  T* Find() {
    for (auto& component : components_) {
      if (T* typed = dynamic_cast<T*>(component.get())) {
        return typed;
      }
    }
    return nullptr;
  }

  /** @brief 类型化查找组件（const 版本）。 */
  template <typename T>
  const T* Find() const {
    for (const auto& component : components_) {
      if (const T* typed = dynamic_cast<const T*>(component.get())) {
        return typed;
      }
    }
    return nullptr;
  }

  /** @brief 按挂载序步进全部组件。 */
  void Step(World& world, double dt_sec) {
    for (auto& component : components_) {
      if (component != nullptr) {
        component->Step(world, dt_sec);
      }
    }
  }

  /** @brief 当前组件数量（日志/断言用）。 */
  std::size_t component_count() const { return components_.size(); }

 private:
  EntityId id_{0U};
  std::string name_;
  std::vector<std::unique_ptr<Component>> components_;
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_CORE_ENTITY_H_
