/**
 * @file ecs_core_test.cpp
 * @brief 自定义实体-组件核心（examples/component_attachment/core/）单元测试。
 *
 * 覆盖：实体挂载/卸载生命周期钩子调用序、类型化组件访问、挂载序与
 * 实体创建序步进、共享场景状态引用、以及组件经 Boost.Signals2 信号
 * 发布 → 订阅者接收的事件接线（验证接线正确性，不测库本身）。
 */

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/component.h"
#include "core/entity.h"
#include "core/events.h"
#include "core/signals.h"
#include "core/world.h"

namespace ca = component_attachment;

namespace {

/// 记录生命周期钩子与步进调用的桩组件。
class TraceComponent : public ca::Component {
 public:
  TraceComponent(const char* name, std::vector<std::string>* log)
      : name_(name), log_(log) {}

  const char* Name() const override { return name_; }

  void OnAttach(ca::Entity& host) override {
    host_ = &host;
    if (log_ != nullptr) log_->push_back(std::string(name_) + ":attach");
  }

  void OnDetach() override {
    if (log_ != nullptr) log_->push_back(std::string(name_) + ":detach");
  }

  void Step(ca::World& world, double dt_sec) override {
    (void)world;
    (void)dt_sec;
    if (log_ != nullptr) log_->push_back(std::string(name_) + ":step");
  }

  ca::Entity* host() const { return host_; }

 private:
  const char* name_;
  std::vector<std::string>* log_;
  ca::Entity* host_{nullptr};
};

/// 每周期发布平台状态事件的桩组件（验证信号接线）。
class PublisherComponent : public ca::Component {
 public:
  const char* Name() const override { return "Publisher"; }

  void Step(ca::World& world, double dt_sec) override {
    (void)dt_sec;
    ca::PlatformStateEvent event;
    event.cycle = world.scene_state().cycle;
    event.speed_mps = 42.0;
    world.signals().on_platform_state(event);
  }
};

/// 带标签字段的组件（验证类型化访问区分组件类型）。
class TaggedComponent : public ca::Component {
 public:
  explicit TaggedComponent(int tag) : tag_(tag) {}

  const char* Name() const override { return "Tagged"; }

  void Step(ca::World& world, double dt_sec) override {
    (void)world;
    (void)dt_sec;
  }

  int tag() const { return tag_; }

 private:
  int tag_;
};

/// 挂载时保存宿主、Step 时经宿主访问兄弟组件的组件（验证组件间
/// 类型化访问通路——真实传感器/融合组件依赖的核心模式）。
class SiblingAccessComponent : public ca::Component {
 public:
  SiblingAccessComponent(std::vector<int>* found_tags) : found_tags_(found_tags) {}

  const char* Name() const override { return "SiblingAccess"; }

  void OnAttach(ca::Entity& host) override { host_ = &host; }

  void Step(ca::World& world, double dt_sec) override {
    (void)world;
    (void)dt_sec;
    if (host_ != nullptr) {
      if (const auto* sibling = host_->Find<TaggedComponent>()) {
        found_tags_->push_back(sibling->tag());
      }
    }
  }

 private:
  std::vector<int>* found_tags_;
  ca::Entity* host_{nullptr};
};

}  // namespace

TEST(EntityTest, AttachTriggersOnAttachAndExposesHost) {
  std::vector<std::string> log;
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& entity = world.CreateEntity("e1");

  entity.Attach(std::make_unique<TraceComponent>("A", &log));

  EXPECT_EQ(log, std::vector<std::string>({"A:attach"}));
  EXPECT_EQ(entity.component_count(), 1U);
  ASSERT_NE(entity.Find<TraceComponent>(), nullptr);
  EXPECT_EQ(entity.Find<TraceComponent>()->host(), &entity);
}

TEST(EntityTest, DetachByNameCallsOnDetachAndRemoves) {
  std::vector<std::string> log;
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& entity = world.CreateEntity("e1");
  entity.Attach(std::make_unique<TraceComponent>("A", &log));

  EXPECT_TRUE(entity.DetachByName("A"));
  EXPECT_FALSE(entity.DetachByName("A"));  // 已卸载：再次卸载失败

  EXPECT_EQ(log, std::vector<std::string>({"A:attach", "A:detach"}));
  EXPECT_EQ(entity.component_count(), 0U);
  EXPECT_EQ(entity.Find<TraceComponent>(), nullptr);
}

TEST(EntityTest, TypedFindDistinguishesComponentTypes) {
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& entity = world.CreateEntity("e1");
  entity.Attach(std::make_unique<TaggedComponent>(7));
  entity.Attach(std::make_unique<TraceComponent>("A", nullptr));

  ASSERT_NE(entity.Find<TaggedComponent>(), nullptr);
  EXPECT_EQ(entity.Find<TaggedComponent>()->tag(), 7);
  EXPECT_NE(entity.Find<TraceComponent>(), nullptr);
  EXPECT_EQ(entity.Find<PublisherComponent>(), nullptr);  // 未挂载类型

  const ca::Entity& const_ref = entity;
  ASSERT_NE(const_ref.Find<TaggedComponent>(), nullptr);
  EXPECT_EQ(const_ref.Find<TaggedComponent>()->tag(), 7);
}

TEST(EntityTest, StepFollowsMountOrder) {
  std::vector<std::string> log;
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& entity = world.CreateEntity("e1");
  entity.Attach(std::make_unique<TraceComponent>("A", &log));
  entity.Attach(std::make_unique<TraceComponent>("B", &log));
  entity.Attach(std::make_unique<TraceComponent>("C", &log));

  world.Step(1.0);

  EXPECT_EQ(log, std::vector<std::string>(
                     {"A:attach", "B:attach", "C:attach", "A:step", "B:step", "C:step"}));
}

TEST(EntityTest, DetachedComponentStopsStepping) {
  std::vector<std::string> log;
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& entity = world.CreateEntity("e1");
  entity.Attach(std::make_unique<TraceComponent>("A", &log));
  entity.DetachByName("A");

  world.Step(1.0);

  // 卸载后不再步进（日志止于 attach/detach）。
  EXPECT_EQ(log, std::vector<std::string>({"A:attach", "A:detach"}));
}

TEST(EntityTest, DetachByNameMatchesNameExactlyAmongMultiple) {
  std::vector<std::string> log;
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& entity = world.CreateEntity("e1");
  entity.Attach(std::make_unique<TraceComponent>("A", &log));
  entity.Attach(std::make_unique<TraceComponent>("B", &log));
  entity.Attach(std::make_unique<TaggedComponent>(1));

  EXPECT_TRUE(entity.DetachByName("A"));
  EXPECT_FALSE(entity.DetachByName("A"));  // 已卸载：再次卸载失败

  // 按名精确匹配：只卸载 A，B（同类型）与 Tagged 仍在（挂载序保留）。
  EXPECT_EQ(entity.component_count(), 2U);
  ASSERT_NE(entity.Find<TraceComponent>(), nullptr);  // B 仍在（A 已卸）
  ASSERT_NE(entity.Find<TaggedComponent>(), nullptr);
  EXPECT_EQ(log, std::vector<std::string>({"A:attach", "B:attach", "A:detach"}));
}

TEST(EntityTest, StepCanAccessSiblingComponentsViaHost) {
  std::vector<int> found_tags;
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& entity = world.CreateEntity("e1");
  entity.Attach(std::make_unique<TaggedComponent>(7));
  entity.Attach(std::make_unique<SiblingAccessComponent>(&found_tags));

  world.Step(1.0);

  // Step 期间经 OnAttach 保存的 host 引用访问同实体兄弟组件（核心通路）。
  EXPECT_EQ(found_tags, std::vector<int>({7}));
}

TEST(EntityTest, DestroyEntityCallsOnDetach) {
  std::vector<std::string> log;
  ca::SceneState scene;
  {
    ca::World world(scene);
    ca::Entity& entity = world.CreateEntity("e1");
    entity.Attach(std::make_unique<TraceComponent>("A", &log));
    entity.Attach(std::make_unique<TraceComponent>("B", &log));
  }  // World 销毁 → 实体销毁 → 组件 OnDetach（按挂载序）

  EXPECT_EQ(log, std::vector<std::string>({"A:attach", "B:attach", "A:detach", "B:detach"}));
}

TEST(WorldTest, StepFollowsEntityCreationOrder) {
  std::vector<std::string> log;
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& e1 = world.CreateEntity("e1");
  ca::Entity& e2 = world.CreateEntity("e2");
  e1.Attach(std::make_unique<TraceComponent>("A", &log));
  e2.Attach(std::make_unique<TraceComponent>("B", &log));
  e1.Attach(std::make_unique<TraceComponent>("C", &log));

  world.Step(1.0);

  // 实体按创建序步进；组件按挂载序步进（e1 先于 e2，A 先于 C）。
  EXPECT_EQ(log, std::vector<std::string>({"A:attach", "B:attach", "C:attach", "A:step",
                                           "C:step", "B:step"}));
}

TEST(WorldTest, FindEntityByName) {
  ca::SceneState scene;
  ca::World world(scene);
  ca::Entity& e1 = world.CreateEntity("alpha");
  ca::Entity& e2 = world.CreateEntity("beta");

  EXPECT_EQ(world.FindEntity("alpha"), &e1);
  EXPECT_EQ(world.FindEntity("beta"), &e2);
  EXPECT_EQ(world.FindEntity("missing"), nullptr);
  EXPECT_EQ(world.entity_count(), 2U);
}

TEST(WorldTest, SceneStateIsSharedReference) {
  ca::SceneState scene;
  scene.cycle = 7U;
  ca::World world(scene);

  EXPECT_EQ(world.scene_state().cycle, 7U);
  world.scene_state().cycle = 8U;  // 经 World 写 → 外部可见
  EXPECT_EQ(scene.cycle, 8U);
}

TEST(SignalTest, PublishReachesAllSubscribers) {
  std::vector<std::uint64_t> received_cycles;
  std::vector<double> received_speeds;
  ca::SceneState scene;
  scene.cycle = 3U;
  ca::World world(scene);
  world.signals().on_platform_state.connect([&](const ca::PlatformStateEvent& e) {
    received_cycles.push_back(e.cycle);
  });
  world.signals().on_platform_state.connect([&](const ca::PlatformStateEvent& e) {
    received_speeds.push_back(e.speed_mps);
  });
  ca::Entity& entity = world.CreateEntity("pub");
  entity.Attach(std::make_unique<PublisherComponent>());

  world.Step(1.0);

  EXPECT_EQ(received_cycles, std::vector<std::uint64_t>({3U}));
  EXPECT_EQ(received_speeds, std::vector<double>({42.0}));

  world.Step(1.0);  // 第二个周期：再次多播
  EXPECT_EQ(received_cycles.size(), 2U);
  EXPECT_EQ(received_speeds.size(), 2U);
}

TEST(SignalTest, UnsubscribedEventTypeNotDelivered) {
  int waypoint_calls = 0;
  ca::SceneState scene;
  ca::World world(scene);
  // 只订阅航点到达信号；组件发布的是平台状态信号。
  world.signals().on_waypoint_reached.connect(
      [&](const ca::WaypointReachedEvent&) { ++waypoint_calls; });
  ca::Entity& entity = world.CreateEntity("pub");
  entity.Attach(std::make_unique<PublisherComponent>());

  world.Step(1.0);

  EXPECT_EQ(waypoint_calls, 0);  // 类型隔离：不同事件不串扰
}
