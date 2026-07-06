# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

---

## OQ-1 飞行动力学局部 NE 投影 cos-lat 约定分叉

`WaypointManager` 与 `Maneuver` 各自实现了 lat/lon → 局部北/东米投影，但两者使用的 cos-纬度约定不同，对大 cross-track 偏移会产生不同几何结果。

- `WaypointManager` 用平均纬度 cos：`src/flight_dynamic/guidance/WaypointManager.cpp:24`（`mean_latitude_rad = 0.5 * (lat + origin_lat)`）、`:28`（`east_m = ... * cos(mean_latitude_rad)`）。
- `Maneuver` 用参考中心纬度 cos：`src/flight_dynamic/guidance/Maneuver.cpp:160`、`:203`（`cos_lat = std::cos(center.latitude_rad)`），并在文件内 6 处内联重复该投影。

为何未决：无法从代码判断哪个约定是有意为之。两者在小偏移下数值接近，分叉只在远场才显现；现有测试未覆盖"两套约定应一致"或"应不同"的断言。合并到单一投影需要先决定以哪个 cos-lat 约定为准。

推进需要：
- 领域知识确认：orbit / figure-8 / racetrack 几何中，cos(mean lat) 与 cos(center/reference lat) 哪个是正确意图；
- 决定后，要么统一为单一约定（带参数的 helper），要么显式记录"两者有意不同"并保留；
- 重测 orbit / figure-8 / racetrack 几何，确认无回归。

注：P2.5a（commit `ff0c9a2c`）只收拢了 `NormalizeRad`/`RadToDeg360`（角度归一化），明确未触碰此 NE 投影分叉。

## OQ-3 飞行动力学失速速度 ρ 来源漂移

失速速度公式 `V_stall = sqrt(2W / (ρ·S·CLmax))` 的 ρ 在三个调用点来源不一致，是已知 bug 但尚无失败测试证据。

- Autopilot：硬编码 `kRhoSeaLevel = 0.002377`：`src/flight_dynamic/autopilot/Autopilot.cpp:339`。
- EngineManager `GetRotationSpeedKts`：读 property tree `atmosphere/rho-slugs_ft3`：`src/flight_dynamic/propulsion/EngineManager.cpp:184`。
- EngineManager `GetDefaultApproachSpeedMps`：读了 property tree ρ 做校验（`:290`），但 V_stall 计算又用硬编码 `kRhoSeaLevel`（`:311`）——同一函数内 ρ 来源自相矛盾。

为何未决：narrow 重构契约要求零行为变化，且无测试因 ρ 漂移而失败。修它等于改变至少一个调用点的数值输出，必须有测试兜底。

推进需要：
- 确认正确的 ρ 来源应是哪个（property tree 的实时大气密度，还是固定海平面常数）；
- 补一个针对 ρ 来源的失败/边界测试（如高海拔场景下 V_stall 应随 ρ 变化）；
- 在测试兜底下统一 ρ 来源。

注：P2.4（commit `65cc7fc4`）把 CLmax + V_stall 公式收拢为单一 `AircraftPerformanceDerivation` helper，但 ρ 作为入参透传，严格保留了三处现状——漂移本身未修。

## OQ-5 flight_dynamic 模块游离于统一 cycle/result 范式

`flight_dynamic::FlightManager` 暴露 `GetAdapter()`/`GetAutopilot()`/`GetEngineManager()` 等内部子系统直接引用（`include/1q/flight_dynamic/FlightManager.h:168-172`），并使用 `FlightManagerState` 枚举状态机，而其它四个传感器模块采用 Session→Controller→Pipeline 分层与 cycle/result/abort-reason 协议。

- 公开子系统所有权：`include/1q/flight_dynamic/FlightManager.h:168-172`。
- 枚举状态机：同文件 `FlightManagerState`（`:48`）与 `state_`（`:187`），区别于其它模块的 `*CycleResult` + `*PipelineAbortReason`。

为何未决：这是 `flight_dynamic` 的既有 public 边界设计，不是缺陷。是否引入统一的 session/cycle 外壳属于跨模块 API 形态决策，需要先确认飞行动力学作为"平台状态生产者"的特殊定位是否应当保留更宽的 public 接口。

推进需要：先在 `docs/flight_dynamic/design.md` 显式文档化该模块的特殊边界（与传感器模块的区别），再评估是否引入更统一的 cycle 外壳。

注：源自 `src/` 架构与安全审查的 M8，原状态 verified-deferred。

## OQ-7 自研 JSON 解析器的 long-term 替换决策

`src/common/config/JsonReader.cpp` 的主要加固已完成（最大嵌套深度、尾随内容拒绝、`\uXXXX` 完整性校验、surrogate escape 拒绝、数字语法负例拒绝）。剩余的是是否长期替换为成熟 JSON 库的策略决策。

- 缺失键返回静态全局 `kNullValue` 引用（`src/common/config/JsonReader.cpp:11`/`:253-262`），改为 `optional`/指针会改变 `JsonValue` public API。
- 完整 surrogate pair 合成仍可补充，但当前加固已阻断主要解析风险。

为何未决：替换/重写 `JsonValue` public API 是独立的 public-surface 变更，须单独契约化，不应与安全加固混批。当前自研解析器的剩余缺口不影响已加固的路径。

推进需要：评估引入成熟 JSON 库 vs 继续 harden 自研解析器，若替换则需同步 `JsonValue` consumer 与 public API 契约。

注：源自 `src/` 架构与安全审查的 H5 残余与 L1。

## OQ-8 common 层局部代码质量收尾

若干低风险样式/编译成本项已验证但未在本轮修复，列出以便独立批次处理，避免与语义修复混批。

- L3：`src/common/geometry/GeometryTransform.h:9` 全量 `#include <Eigen/Core>`，可评估前向声明降低编译成本（纯编译优化，无语义影响）。
- L4：约 10 处 `ptr.reset(new T)` 可逐步替换为 `std::make_unique`（纯样式）。
- L6：`src/common/atmosphere/AtmospherePhysics.cpp:63-76` 的 `refractivity_index_n_*` 同函数并列 `tc_celsius` 与 `tk_kelvin` 两个温标参数，调用方易传错；改签名涉及 REOS 对齐与兼容迁移。

为何未决：三者均为样式/兼容性收尾，不改变运行时行为，混入语义修复批次会模糊变更意图。

推进需要：在独立的小步重构批次中处理，L6 需要配套 REOS 对齐签名迁移。

注：源自 `src/` 架构与安全审查的 L3/L4/L6，原状态 verified-deferred。
