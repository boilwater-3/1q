// Copyright 2026. All Rights Reserved.
//
// Description: 验证 EventBus 的边界行为与订阅生命周期。

#include <gtest/gtest.h>

#include "1q/airborne_radar/core/event/IEventBus.h"
#include "airborne_radar/core/event/EventBus.h"

namespace airborne_radar { namespace tests {
namespace {

struct BusTestEvent {
  BusTestEvent() = default;
  BusTestEvent(int v) : value(v) {}
  int value{0};
};

} // namespace

TEST(EventBusTest, PublishWithoutSubscribersIsNoOp) {
  core::event::EventBus bus;

  const BusTestEvent event{42};
  bus.Publish(event);

  SUCCEED();
}

TEST(EventBusTest, DuplicateSubscriptionsAllReceiveEvent) {
  core::event::EventBus bus;

  int first_sum = 0;
  int second_sum = 0;

  bus.Subscribe<BusTestEvent>(
      [&first_sum](const BusTestEvent &event) { first_sum += event.value; });
  bus.Subscribe<BusTestEvent>(
      [&second_sum](const BusTestEvent &event) { second_sum += event.value; });

  const BusTestEvent event{7};
  bus.Publish(event);

  EXPECT_EQ(first_sum, 7);
  EXPECT_EQ(second_sum, 7);
}

TEST(EventBusTest, UnsubscribeStopsDelivery) {
  core::event::EventBus bus;

  int hit_count = 0;
  const auto token = bus.Subscribe<BusTestEvent>(
      [&hit_count](const BusTestEvent &) { ++hit_count; });

  bus.Publish(BusTestEvent{1});
  bus.Unsubscribe(token);
  bus.Publish(BusTestEvent{1});

  EXPECT_EQ(hit_count, 1);
}

TEST(EventBusTest, ClearRemovesAllSubscriptions) {
  core::event::EventBus bus;

  int hit_count = 0;
  bus.Subscribe<BusTestEvent>(
      [&hit_count](const BusTestEvent &) { ++hit_count; });

  bus.Clear();
  bus.Publish(BusTestEvent{1});

  EXPECT_EQ(hit_count, 0);
}

TEST(EventBusTest, UnsubscribeUnknownTokenIsNoOp) {
  core::event::EventBus bus;

  int hit_count = 0;
  bus.Subscribe<BusTestEvent>(
      [&hit_count](const BusTestEvent &) { ++hit_count; });

  bus.Unsubscribe(core::event::EventToken{typeid(BusTestEvent), 99999});
  bus.Publish(BusTestEvent{1});

  EXPECT_EQ(hit_count, 1);
}

} } // namespace airborne_radar::tests
