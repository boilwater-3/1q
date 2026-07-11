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

## OQ-8 折射率成对温度输入的 public 迁移

原 OQ-8 的低风险收尾已复核：L3 不能移除 `GeometryTransform.h` 的 `Eigen/Core`，因为该头直接以 `Eigen::Vector3f` / `Eigen::Matrix3f` 作为函数返回值和参数；L4（`src/common` 的 `reset(new)`）已经不存在。两项均无需代码修复。

剩余的 L6 不再是低风险样式项：`refractivity_index_n_r4/r8` 和公开的 `RefractivityIndex` 同时接收摄氏与开氏温度。两个裸浮点参数可被调换，但改变为成对温度类型或单一温标会改变 REOS 对齐的 public 签名。

为何未决：仓库内只有转发实现和一致温标的单测，无法证明仓库外调用方不依赖当前签名。静默派生其中一个温度会改变不一致输入的数值语义，也不能可靠修复“参数被调换”。

推进需要：独立 Stage A 冻结 public migration（新 typed input/过渡入口、REOS 对齐、外部 consumer 期限），并补充温标不一致的拒绝或诊断契约。\
[evidence: `include/1q/environment/PropagationPhysics.h:RefractivityIndex` — public 六标量签名;\
 `src/common/atmosphere/AtmospherePhysics.cpp:refractivity_index_n_r8` — 同时消费 Celsius 与 Kelvin;\
 `tests/unit/airborne_radar/ar_atmosphere_physics_test.cpp` — 当前只覆盖一致温标]
