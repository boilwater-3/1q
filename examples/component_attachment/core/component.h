/**
 * @file component.h
 * @brief 自定义实体-组件示例：组件基类。
 *
 * 本示例的组件携带逻辑：每个仿真模块对应一个 Component 子类，挂载到实体
 * 后按挂载序周期步进。生命周期钩子 OnAttach/OnDetach 提供挂载/卸载通知。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_CORE_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_CORE_COMPONENT_H_

namespace component_attachment {

class Entity;
class World;

/**
 * @brief 组件基类：模块逻辑的挂载单元。
 *
 * 子类实现 Name（类型标识）与 Step（周期推进）；OnAttach 时保存宿主
 * 实体引用（经 host_.Find<T>() 类型化访问同实体组件，供周期内同步数据
 * 聚合）；跨周期通知/记录经 World 的信号发布事件（见 signals.h）。
 */
class Component {
 public:
  virtual ~Component() = default;

  Component(const Component&) = delete;
  Component& operator=(const Component&) = delete;

  /** @brief 组件类型名（日志/卸载/事件记录用）。 */
  virtual const char* Name() const = 0;

  /** @brief 挂载钩子：组件被挂到实体时调用（默认空实现）。 */
  virtual void OnAttach(Entity& host) { (void)host; }

  /** @brief 卸载钩子：组件从实体卸载或实体销毁时调用（默认空实现）。 */
  virtual void OnDetach() {}

  /** @brief 周期步进：World 每周期按实体创建序、组件挂载序调用。 */
  virtual void Step(World& world, double dt_sec) = 0;

 protected:
  Component() = default;
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_CORE_COMPONENT_H_
