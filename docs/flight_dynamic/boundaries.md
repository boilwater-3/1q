---
Status: active
Last-reviewed: 2026-08-03
Authority: flight_dynamic 模块级边界、非目标与设计变更规则
Answers: flight_dynamic 有哪些模块级禁令、FlightManager public seam 边界、变更规则
---

# Flight Dynamic 模块边界

本文承载 flight_dynamic 的模块级边界、public seam 定义、非目标和变更规则。算法级边界（自动驾驶
profile、航路点语义、机动执行等）见 [algorithms.md](algorithms.md)。

## FlightManager public seam（不得随意删除）

`FlightManager` 的 `GetAdapter()`、`GetAutopilot()`、`GetEngineManager()` 和 `GetWaypointManager()` 是
既有 public 兼容面，不是可随意删除的内部便利函数。当前消费者证明它们承担不同职责：

1. `GetAdapter()`：FD adapter 测试和已安装的起降示例直接访问 JSBSim property tree、传播状态与地面反力。
2. `GetAutopilot()`：多个飞行示例和行为测试读取 aircraft control profile，并设置速度/高度保持目标。
3. `GetEngineManager()` 与 `GetWaypointManager()`：分别被发动机/航路点行为测试和示例消费。

保留四个 getter，不新增包装它们的通用控制端口，也不以传感器 `Session/Cycle` 形状重构 Flight Dynamic。
若未来收窄任一 getter，必须先逐个迁移已安装示例和 consumer test，并以独立 Stage A 冻结替代 API、
JSBSim 属性边界和兼容期限。

[evidence: tests/unit/flight_dynamic/fd_adapter_test]

## 模块定位边界

`flight_dynamic` 不是自研飞行动力学求解器：

1. 底层六自由度积分、气动、发动机、起落架、FCS 和 aircraft XML 行为由 JSBSim 提供。
2. 本模块只负责装载、映射、控制意图下发和状态映射。
3. 代码调试必须区分 JSBSim 库行为、aircraft XML/data contract、1Q 集成层、控制算法和测试阈值——
   这五者不可混为一谈。
4. aircraft-specific tuning 优先进入 XML `guidance/*` 属性或配置，而不是散落到通用机动逻辑。

## 编译开关

模块由 `ONEQ_ENABLE_FLIGHT_DYNAMIC` 控制，默认 **OFF**。关闭时不编译模块目标和测试；JSBSim 仍是
必选依赖（`src/common/environment/JsbsimAtmosphereAdapter` 需要）。启用后才提供目标、测试和示例。

## 非目标

1. 不把 JSBSim 库和 aircraft XML 的行为包装成 1Q 自己的物理模型承诺。
2. 不通过放宽阈值、扩大 skip 或改弱测试条件来掩盖模型/集成问题。
3. 不在通用 autopilot/maneuver 代码中硬编码 aircraft-specific 特例。
4. 不把 RunIC/trim/gear/engine 初始化问题归入 waypoint 或 maneuver 算法问题。

## 设计变更规则

1. JSBSim 属性、RunIC、trim、initial velocity frame 或状态映射变化必须同步本文档集。
2. Autopilot profile、XML `guidance/*` override 或机动控制律变化必须同步 algorithms.md。
3. Waypoint 到达语义变化必须明确说明 capture radius、法平面穿越和 cross-track corridor 的影响。
4. 已知限制应进入 dedicated test label（`known_limit;flight_dynamic`），不得改变稳定测试
   （`unit;flight_dynamic`）含义。
5. known-limit 不得替代稳定能力承诺。
