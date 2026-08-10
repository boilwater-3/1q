/**
 * @file world.h
 * @brief 自定义实体-组件示例：世界（实体注册表 + 步进 + 事件信号）。
 *
 * World 是实体注册表：实体创建/查找、按序步进、共享上下文
 * （scene_state）、事件信号集合（signals）。
 * 实体按创建序步进；周期调用 World::Step(dt) 即推进整场仿真。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_CORE_WORLD_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_CORE_WORLD_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "entity.h"
#include "signals.h"

namespace component_attachment {

/**
 * @brief 场景共享状态基类：消费方继承扩展。
 *
 * 消费方每周期更新（周期号/时间/世界真值），组件 Step 经 world.scene_state()
 * 读取——共享上下文，避免把世界模型塞进单个组件。
 */
struct SceneState {
  std::uint64_t cycle{0U}; /**< 当前世界周期号（从 1 开始） */
  double t_sec{0.0};       /**< 当前周期绝对时间（s） */
};

/**
 * @brief 世界：实体注册表与仿真步进入口。
 */
class World {
 public:
  /** @param[in] scene_state 共享场景状态（消费方持有生命周期） */
  explicit World(SceneState& scene_state) : scene_state_(scene_state) {}

  ~World() = default;

  World(const World&) = delete;
  World& operator=(const World&) = delete;

  /** @brief 创建实体；创建序即每周期步进序。返回的引用在实体销毁前稳定。 */
  Entity& CreateEntity(const std::string& name) {
    auto entity = std::make_unique<Entity>(next_entity_id_++, name);
    Entity* raw = entity.get();
    entities_.push_back(std::move(entity));
    return *raw;
  }

  /** @brief 按名称查找实体；未找到返回 nullptr。 */
  Entity* FindEntity(const std::string& name) {
    for (auto& entity : entities_) {
      if (entity->name() == name) {
        return entity.get();
      }
    }
    return nullptr;
  }

  /** @brief 共享场景状态（消费方每周期更新，组件 Step 读取）。 */
  SceneState& scene_state() { return scene_state_; }

  /** @brief 事件信号集合（Boost.Signals2 多播，见 signals.h）。 */
  Signals& signals() { return signals_; }

  /** @brief 按实体创建序步进全部实体（组件按挂载序步进）。 */
  void Step(double dt_sec) {
    for (auto& entity : entities_) {
      entity->Step(*this, dt_sec);
    }
  }

  /** @brief 当前实体数量（日志用）。 */
  std::size_t entity_count() const { return entities_.size(); }

 private:
  SceneState& scene_state_;
  std::vector<std::unique_ptr<Entity>> entities_;
  Signals signals_;
  EntityId next_entity_id_{1U};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_CORE_WORLD_H_
