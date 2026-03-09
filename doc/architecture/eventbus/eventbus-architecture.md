# EventBus 模块架构设计（基于当前实现）

## 1. 设计目标

EventBus 模块为雷达核心流程提供跨层消息传递能力，支持两种语义：

- 即时分发（`EventBus`）：发布即触发订阅回调。
- 周期队列（`CycleEventBus`）：本周期发布，下一周期消费。

两者统一通过 `IEventBus` 暴露给上层，允许 `RadarController` 透明切换实现。

## 2. 抽象接口设计

文件：`include/1q/airborne_radar/core/event/IEventBus.h`

### 2.1 通用能力

- `Subscribe<Event>(handler)`
- `Publish<Event>(event)`
- `Enqueue<Event>(event)`
- `Unsubscribe(token)`
- `Clear()`

### 2.2 周期扩展钩子

- `BeginCycle()`
- `DispatchCurrentCycle()`
- `EndCycle()`

默认空实现保证兼容即时总线。

### 2.3 载荷生命周期

- `EventPayload = std::shared_ptr<const void>`
- 发布/入队时通过 `std::make_shared<Event>(event)` 持有载荷，避免异步队列下悬垂指针问题。

## 3. 即时总线 `EventBus`

文件：

- `src/airborne_radar/core/event/EventBus.h`
- `src/airborne_radar/core/event/EventBus.cpp`

实现特点：

- 基于 `eventpp::EventDispatcher`。
- `PublishImpl(...)` 直接 `dispatch(...)`。
- 维护 `listeners_`，支持按 token 取消订阅。
- 适合需要“发布后立即响应”的场景。

## 4. 周期总线 `CycleEventBus`

文件：

- `src/airborne_radar/core/event/CycleEventBus.h`
- `src/airborne_radar/core/event/CycleEventBus.cpp`

实现特点：

- 基于 `eventpp::EventQueue`。
- 双队列双缓冲：`queues_[2]`。
- 索引管理：
  - `current_queue_index_`
  - `next_queue_index_`
- `BeginCycle()` 交换 current/next。
- `DispatchCurrentCycle()` 处理 current 队列。
- `PublishImpl()` 和 `EnqueueImpl()` 均入 next 队列。

语义结果：

- 同周期发布的事件不会在本周期回流处理。
- 下一次周期开始后才可被消费。

## 5. 与 `RadarController` 的集成

文件：`src/airborne_radar/core/controller/RadarController.cpp`

当前集成方式：

1. 周期开始：
   - `BeginCycle()`
   - `DispatchCurrentCycle()`
2. 完成信号/决策/命令下发。
3. 发布周期输出事件（`Enqueue`）：
   - `TracksUpdatedEvent`
   - `JammingAlertEvent`
   - `CommandsSubmittedEvent`
   - `RadarCycleCompletedEvent`
4. 周期结束：`EndCycle()`

说明：

- 对 `EventBus`（即时分发）而言，`Enqueue` 默认退化为 `Publish`，事件会立即触达。
- 对 `CycleEventBus` 而言，上述四类事件会在下一周期被消费。

## 6. 已定义事件类型

文件：`include/1q/airborne_radar/core/event/RadarEvents.h`

- `TracksUpdatedEvent`
- `JammingAlertEvent`
- `CommandsSubmittedEvent`
- `RadarCycleCompletedEvent`

## 7. 测试覆盖（当前代码）

- `tests/event_bus_test.cpp`
  - 即时总线订阅、重复订阅、取消订阅、清空订阅边界行为。
- `tests/core_controller_test.cpp`
  - `CycleEventBusDeliversEventsOnNextCycle` 验证下一周期可见语义。

## 8. 当前约束

- 当前 `CycleEventBus` 未引入线程同步机制，默认按单线程周期主循环使用。
- `EndCycle()` 暂为扩展点，可在后续接入统计/监控。

## 9. 图示

- 架构图：`doc/architecture/eventbus/eventbus-architecture.puml`
- 周期流程图：`doc/architecture/eventbus/eventbus-cycle-flow.puml`
