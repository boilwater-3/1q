---
Status: active
Last-reviewed: 2026-08-03
Authority: flight_dynamic 算法登记与实现边界
Answers: flight_dynamic 用了哪些算法、各自实现到什么地步、边界在哪
---

# Flight Dynamic 算法登记

本文是 flight_dynamic 算法清单与边界的权威。算法本身的逐步逻辑读代码（`src/flight_dynamic/`）；
本文只回答"用没用/到哪步/边界在哪"。模块级边界（FlightManager public seam、编译开关）见
[boundaries.md](boundaries.md)。

## 算法登记表

| 算法 | 意图（一句话） | 实现状态 | 证据 |
|---|---|---|---|
| JSBSim 适配 | 加载 aircraft、设置属性、RunIC、trim、Run | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_adapter_test] |
| 初始条件映射 | 注入 LLA/ECEF 位置和 body/ECEF 速度 | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_adapter_test] |
| 状态映射 | JSBSim propagate/accelerations 到 VehicleState | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_adapter_test] |
| 自动驾驶 profile | 推导速度/俯仰/滚转/油门边界，允许 XML 覆盖 | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_aircraft_performance_derivation_test] |
| 航路点管理 | active waypoint、到达/越过判定 | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_aircraft_maneuver_test] |
| 机动执行 | 起飞/降落/盘旋/航路/S-turn/racetrack/figure-8 | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_aircraft_maneuver_test] |
| 推进管理 | 发动机类型、启动、油门/推进辅助 | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_adapter_test] |
| 诊断 | 最低高度/速度、最大姿态、坠毁和结果分类 | 生产可用 | [evidence: tests/unit/flight_dynamic/fd_orbit_quality_test] |

## JSBSim 初始化与适配

- **意图**：`JsbsimAdapter` 是 1Q 与 JSBSim 的边界，负责 aircraft 装载、初始条件注入、RunIC、trim、Run。
- **实现边界**：
  1. `InitialVelocityFrame::kBody` 与 `kEcef` 是不同契约，不得在测试中混用。
  2. 机型地面高度优先尊重 reset XML；当配置 altitude 为 0 时，状态映射不应覆盖该 aircraft-specific
     地面值。
  3. JSBSim FCS 内部状态可能不完全体现在 property tree，trim 失败恢复必须处理 FCS component 内部状态。
  4. 地面起始时执行 `SettleInitialGroundState()`，让接触力先收敛。
  5. 可选 `DoTrim()` 失败时重置 FCS 内部状态并重新 RunIC。
- **证据**：[evidence: tests/unit/flight_dynamic/fd_adapter_test]

## 状态映射

- **意图**：`VehicleStateMapper` 把 JSBSim 状态映射为 `VehicleState`，是 public 输出状态的唯一稳定入口。
- **实现边界**：
  1. 不把 JSBSim 属性名直接泄露给调用方。
  2. 不在状态映射层修正控制问题；控制应回到 autopilot、maneuver 或 XML/profile。
  3. **纬度语义必须是 WGS84 大地纬度（geodetic）**：JSBSim 1.3.1 的
     `FGLocation::GetLatitude` / `FGInitialCondition::SetLatitudeDegIC` 均为地心纬度
     （geocentric）语义，1Q 必须走 `GetGeodLatitudeRad` / `SetGeodLatitudeDegIC`。
     两约定在赤道重合、非赤道相差可达 ~0.19°（30°N ≈ 18.5 km）——曾导致 IC 位置与
     `VehicleState.latitude_rad` 整体北移，行为层示例航点跟踪随之偏离（2026-08-05 修复，
     回归测试见 fd_adapter_test 的 InitialLatitudeIsGeodeticNotGeocentric）。
- **证据**：[evidence: tests/unit/flight_dynamic/fd_adapter_test]

## 自动驾驶 profile 与控制律

- **意图**：`Autopilot` 把高层目标转成 JSBSim 控制属性；profile 由 aircraft 物理量和 XML override 共同决定。
- **实现边界**：
  1. profile 推导是 fallback；aircraft-specific tuning 应进入 XML `guidance/*`。
  2. 自动驾驶写 JSBSim 属性，不直接修改 `VehicleState`。
  3. 速度/高度/姿态控制失败必须区分四个不同根因：profile 不合适、aircraft XML 模型限制、JSBSim 行为、
     测试目标不合理。
- **反直觉点（XML override 的半冻结语义）**：XML `guidance/ref-speed-mps` 等是显式 TAS override——
  **被覆盖字段保持固定，未覆盖字段继续随重量/密度动态推导**。每个 `Autopilot::Update()` 开始时，
  使用上一 JSBSim step 已提交的当前重量与大气密度刷新 stall 速度，其余速度继续使用既有分类因子。
- **反直觉点（非法输入的降级）**：当前重量或密度非法时不提交新包线，保留最后一次有效值；构造阶段
  以标准海平面密度建立安全基线，并在初始密度非法时记录 warning。
- **证据**：[evidence: tests/unit/flight_dynamic/fd_adapter_test]
- **证据**：[evidence: tests/unit/flight_dynamic/fd_aircraft_performance_derivation_test]

## 航路点到达语义

- **意图**：`WaypointManager` 维护 active waypoint 和 leg start，判定到达/越过。
- **实现边界**：
  1. waypoint "到达"不是单一距离阈值，而是分两层：距离 threshold/capture radius，或法平面穿越 +
     cross-track corridor。
  2. **中间/最终航点完成语义分离**（由 `ManeuverExecutor::SetIntermediateWaypoint` 按队列后继
     设置）：队列中后继仍是 kFlyToWaypoint 的中间航点，完成 = 法平面穿越（corridor 内）或进入
     到达半径 `max(radius_m, 100m)`——纯导航事件，与机型无关；转弯可行性不进入完成门，路径
     圆角随各机型自身转弯半径自然缩放。最终航点（单航点或队列末尾）保持转弯量级捕获圈
     `max(radius_m, 1.5×v²/(g·tan(max_bank)))`——容差按机型/速度实时推导，不同型号不可一概
     而论用定值。若不分离，航点间距小于捕获圈的航路会在起步时被整条吞掉（飞机未飞即全部
     "到达"），且用户设置的 radius_m（默认 100 m）会被转弯项完全掩盖。
  3. **完成事件记录**：每个 kFlyToWaypoint 完成时保留 `WaypointSequencingEvent`
     决策快照（命中门 kWithinRadius/kPlaneCrossing/kFlyPastHeuristic、距离/侧距/
     沿航迹、有效阈值、航点索引、仿真时间、中间-最终语义），经
     `FlightManager::GetWaypointEvents()` 查询（容量 512 环形，丢最旧）。另经
     PROJECT_LOG 发射：每步 `[FLYTO]` DEBUG 决策轨迹（距离/沿航迹/侧距/阈值，
     供日志启用后观察收敛与门余量），完成时 `[FLYTO] waypoint complete` INFO 一行
     （门 + 距离 + 侧距 + 沿航迹 + 阈值，默认级别可见）。
  4. 全球 waypoint 航段使用球面大圆；orbit、figure-8、racetrack 等局部机动仍以各自参考中心的局部
     切平面构造路径。这两类几何服务于不同尺度，**不要求共享同一个纬经度投影 helper**。
- **反直觉点（大转弯提前切航点的防护）**：横向偏差超过 `max(3000m, radius * 3)` 时，不允许仅凭
  法平面穿越判定到达——否则大转弯中会提前切航点。
- **反直觉点（退化航段的保守策略）**：零长度、非有限坐标和近对跖点航段没有稳定的唯一短弧，
  几何解析失败时保守返回"尚未越过"（不抛异常、不猜测）。
- **反直觉点（经度归一化）**：经度差归一化到最短跨界弧，航段可跨越 ±180° 经度边界；高纬长航段
  不再使用平面 `cos(latitude)` 近似。
- **证据**：[evidence: tests/unit/flight_dynamic/fd_aircraft_maneuver_test]
- **证据**：[evidence: tests/unit/flight_dynamic/fd_adapter_test TightSpacedWaypointRouteFlowsSequentially]
- **证据**：[evidence: tests/unit/flight_dynamic/fd_adapter_test WaypointSequencingEventsRecorded]

## 机动执行

- **意图**：`ManeuverExecutor` 是机动状态机，按 `ManeuverType` 分发到不同机动逻辑。
- **实现边界**：
  1. `ManeuverCommand` 字段语义按 `type` 重载，这是现状设计；维护时必须以 `FlightManager.h` 注释和
     `ExecuteNextManeuver()` dispatch 为准。
  2. 状态机：`kIdle`（仅成员默认值）→ 构造成功/Reset 后 `kReady` → `PushManeuver` 后 `kExecuting` →
     队列耗尽 `kCompleted` / 中止 `kAborted`。
  3. `PushManeuver()` 总会追加命令；只有 `kReady` 时立即启动执行，执行中追加的命令排队。
  4. `kCompleted` 和 `kAborted` 的 `Step()` 都返回 `false`，必须通过 `Reset()` 重建组件并回到 `kReady`。
- **证据**：[evidence: tests/unit/flight_dynamic/fd_aircraft_maneuver_test]

## 起飞与降落

- **意图**：起飞按发动机类型设置 idle/static runup/throttle ramp/rotation；降落包括 pattern capture、
  approach、flare、touchdown/rollout。
- **实现边界**：
  1. 起飞逻辑按发动机类型（turboprop/turbine/piston）区分，不能用一个常数覆盖。
  2. 大量阈值可以被 XML `guidance/*` 覆盖。通用逻辑只表达阶段机理，aircraft-specific 数值应进入 XML。
  3. **步长上限**：六自由度机动（含地面滑跑/起落架等快动态）建议 `Step(dt)` 用 10-20 ms；
     100 ms 量级在起飞段发散（实测 roll 达 180° 量级后数值崩溃）。权威用法：
     `examples/flight_dynamic/takeoff_land_csv.cpp`（dt=0.01）；集成契约见
     `FlightManager.h` `Step()` 的 @note。
- **反直觉点（CAS vs TAS 不得混用）**：起飞旋转速度与默认进近速度是 **CAS 基准**，统一使用标准
  海平面密度计算，不随机场高度改变；飞行中的 autopilot 安全包线则是**动态 TAS**。两者不得混用。
- **证据**：[evidence: tests/unit/flight_dynamic/fd_adapter_test]、
  [evidence: examples/flight_dynamic/takeoff_land_csv.cpp 起飞段回归]

## 推进管理

- **意图**：`EngineManager` 和 adapter 的 propulsion helpers 管理发动机启动、油门、prop advance、刹车
  和起落架状态。
- **实现边界**：JSBSim 不同发动机模型对 throttle/prop advance/InitRunning 的响应差异较大，因此推进
  问题不能简单归为 autopilot 控制问题。
- **证据**：[evidence: tests/unit/flight_dynamic/fd_adapter_test]
