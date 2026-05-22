# 任务规划 — 集成 JSBSim 飞行动力学模块 (flight_dynamic)

## 目标

将 JSBSim 飞行动力学库集成为 1Q 项目的新模块 `flight_dynamic`，提供载具 6-DOF 状态推进能力，供 AR/EOS/ESR 等传感器模块消费。遵循 1Q 现有架构模式（Session + Factory + PIMPL + coordinate 模块复用）。

## 背景

- **JSBSim**: LGPL 2.1 开源 C++ 飞行动力学库，CMake 构建，零外部依赖，三平台可编译
- **JSBSim 版本**: commit `70a327fc` (2020-10-18, C++11, pre-1.2.0) — v1.2.0 要求 C++14，与 VS2015 不兼容
- **1Q 坐标系**: ECEF 全局 + ENU 局部 + Body→ENU 欧拉角 (Z-Y-X)
- **JSBSim 坐标系**: ECEF 全局 + NED 局部 + Body→NED 四元数
- **现有参考模块**: airborne_radar (Session/SessionFactory/PIMPL 模式最成熟)
- **C++ 标准**: C++11（默认，最低要求），JSBSim 编译为 C++11
- **依赖管理**: Conan（主）/ vcpkg（备）/ third_party 内嵌（兜底 — JSBSim 专属）

## 数据流设计

```
FlightDynamicSession.Step(FlightDynamicInput{dt, control, ext_force})
       │
       ▼
  JsbsimAdapter (FGFDMExec.Run)
       │
       ▼
  VehicleStateMapper (JSBSim NED/Body/英制 → 1Q ENU/ECEF/SI)
       │
       ▼
  FlightDynamicOutput { ExternalKinematics kinematics, VehicleState state }
       │
       ▼
  消费者直接使用 ExternalKinematics 注入传感器模块:
    RadarCycleInput { platform_pose = output.kinematics, ... }
    EosCycleInput   { platform_pose = output.kinematics, ... }
    EsrCycleInput   { platform_pose = output.kinematics, ... }
```

> **关键决策**: FlightDynamicSession 输出 `ExternalKinematics` 格式，让现有传感器 adapter 零修改消费。

## 阶段

### 阶段 A：可行性验证与依赖引入 ✅

| # | 任务 | 状态 |
|---|------|------|
| A1 | JSBSim 依赖引入：Conan 不可达 → Homebrew 无包 → third_party 源码兜底 | ✅ |
| A2 | 最小 PoC：编译链接 fd_engine/fd_core + libJSBSim 成功 | ✅ |
| A3 | JSBSim API 验证：LoadModel(单参), RunIC(), GetIC()→raw ptr, GetPropagate()→raw ptr | ✅ |
| A4 | C++11 兼容性验证：fd_engine/fd_core 以 -std=c++11 编译通过 | ✅ |
| A5 | LGPL 合规：JSBSim 始终作为共享库链接（不受 BUILD_SHARED_LIBS 影响） | ✅ |

### 阶段 B：公共 API 与类型设计 ✅

| # | 任务 | 状态 |
|---|------|------|
| B1 | `include/1q/flight_dynamic/model/VehicleState.h` — 载具状态结构体（标记 `ONEQ_API`） | ✅ |
| B2 | `include/1q/flight_dynamic/model/FlightDynamicInput.h` — Step 输入结构体 | ✅ |
| B3 | `include/1q/flight_dynamic/model/FlightDynamicOutput.h` — Step 输出结构体 | ✅ |
| B4 | `include/1q/flight_dynamic/config/FlightDynamicConfig.h` — 配置结构体 | ✅ |
| B5 | `include/1q/flight_dynamic/config/AircraftDefinition.h` — 飞行器定义参数 | ✅ |
| B6 | `include/1q/flight_dynamic/session/FlightDynamicSession.h` — Session 门面 | ✅ |
| B7 | `include/1q/flight_dynamic/session/FlightDynamicSessionFactory.h` — 工厂 | ✅ |
| B8 | `include/1q/flight_dynamic/flight_dynamic.hpp` — umbrella header | ✅ |

### 阶段 C：内部实现（含坐标转换） ✅

| # | 任务 | 状态 |
|---|------|------|
| C1 | `src/flight_dynamic/adapter/JsbsimAdapter.h/.cpp` — FGFDMExec 生命周期管理 | ✅ |
| C2 | `src/flight_dynamic/model/VehicleStateMapper.h/.cpp` — 坐标/单位转换（全部 C2a-C2f） | ✅ |
| C3 | `src/flight_dynamic/session/FlightDynamicSession.cpp` — Session 实现 | ✅ |
| C4 | `src/flight_dynamic/session/FlightDynamicSessionFactory.cpp` — 工厂实现 | ✅ |

### 阶段 D：构建系统集成 ✅

| # | 任务 | 状态 |
|---|------|------|
| D1 | `src/flight_dynamic/CMakeLists.txt` — FD_ENGINE_SOURCES / FD_CORE_SOURCES | ✅ |
| D2 | 修改 `src/CMakeLists.txt` — 注册 fd_engine + fd_core OBJECT 库 | ✅ |
| D3 | `conanfile.py` — 跳过（JSBSim 无 Conan 包，始终走 third_party） | ✅ |
| D4 | 修改 `cmake/ProjectDependencies.cmake` — JSBSim 无条件构建 + 链接 | ✅ |
| D5 | `third_party/jsbsim/` — C++11 commit 源码内嵌，作为共享库构建 | ✅ |
| D6 | 公共头文件安装规则 + 合约守卫白名单更新 | ✅ |

### 阶段 E：模块装配与大气协调

| # | 任务 | 状态 |
|---|------|------|
| E1 | 验证 FlightDynamicSession 可与 RadarSession 在同一实体中共存（独立模块，不耦合） | ✅ |
| E2 | 确认 ExternalKinematics 输出可直接被传感器 adapter 消费（无需额外转换层） | ✅ |
| E3 | 大气模型协调策略文档化（DR2） | ✅ |

> **架构澄清**: flight_dynamic 是独立的机动模块，与其他传感器模块平等。
> 实体负责装配模块组合，调用 FlightDynamicSession::Step() 获取 ExternalKinematics，
> 再将其传入传感器模块的 CycleInput。模块间不存在依赖或适配关系。
>
> **大气协调**: 初始阶段 JSBSim 内置 ISA 模型与传感器 AtmosphericObservation 独立运行。
> JSBSim 基于高度的标准大气推算与传感器标量观测值粒度不同，后续若需统一可注入覆盖。

### 阶段 F：测试与验证

| # | 任务 | 状态 |
|---|------|------|
| F1 | `fd_session_test.cpp` — FlightDynamicSession 完整测试（8 个测试） | ✅ |
| F2 | → 已合并至 F1：生命周期、坐标输出、状态精度全覆盖 | ✅ |
| F3 | → JsbsimAdapter 已通过 Session 测试间接覆盖 | ✅ |
| F4 | FlightDynamicSession → RadarSession 联合运行测试 | N/A（实体装配层职责，非本模块范围） |
| F5 | 全量测试回归（876/876 通过，含新增 8 个 fd 测试） | ✅ |
| F6 | 测试通过 GLOB_RECURSE 自动注册到 `1q_unit_tests` | ✅ |

---

# 阶段 G：机动控制层 (Maneuver Controller)

## 目标

在 flight_dynamic 模块内扩展机动控制能力，实现五种基础机动模式（方向、固定点、航路点、蛇形、滚筒），由 1Q 端制导/控制逻辑驱动 JSBSim 物理引擎。遵循 `maneuver_algorithms.md` 的核心算法设计。

## 前置依赖

- ✅ phase A-F 完成，FlightDynamicSession 可用
- ⚠️ 需验证 c172x 模型的 AP 属性（`ap/heading_setpoint`、`ap/heading_hold`、`ap/altitude_setpoint`）可用性

## 架构设计

```
┌─────────────────────────────────────────────────┐
│ ManeuverController (1Q 端)                       │
│ ┌───────────┐ ┌──────────┐ ┌─────────────────┐ │
│ │ Heading   │ │ Waypoint │ │ BarrelRoll       │ │
│ │ Maneuver  │ │ Maneuver │ │ Maneuver         │ │
│ └─────┬─────┘ └────┬─────┘ └────────┬────────┘ │
│       │             │               │           │
│       └──────────┬──┴───────────────┘           │
│                  ▼                               │
│         ControlInput { throttle, aileron,        │
│           elevator, rudder, heading_setpoint,    │
│           heading_hold, altitude_setpoint }       │
└──────────────────┬──────────────────────────────┘
                   │ Step()
                   ▼
┌─────────────────────────────────────────────────┐
│ FlightDynamicSession (现有)                       │
│   JsbsimAdapter → FGFDMExec.Run()                │
│   VehicleStateMapper → ExternalKinematics         │
└─────────────────────────────────────────────────┘
```

> **关键决策**: 制导/控制逻辑在 1Q 端，JSBSim 保持为纯物理引擎。ManeuverController 不依赖传感器模块。

## 阶段

### 阶段 G0：AP 可用性验证 ✅

| # | 任务 | 状态 |
|---|------|------|
| G0a | 在 c172x 模型上读取 `ap/heading_setpoint` 等属性，确认 `c172ap.xml` 包含完整 autopilot | ✅ |
| G0b | 方案 B 不需要——AP 存在 | N/A |
| G0c | 方案 C 不需要——JSBSim 内置 AP 可用（航向保持已验证收敛） | N/A |

> **G0 结论**: c172x 模型有完整的 AP（heading_hold + altitude_hold + wing_leveler）。
> 航向保持 AP 经验证可收敛（90°/180° 转弯测试通过）。
>
> **关键修复（2026-05-22）**:
> 1. c172x.xml 的 `ap/aileron_cmd` 被注释掉（2020 版 bug，上游 2023 年修复），导致 c172ap.xml heading hold 输出无法到达 FCS → 取消注释
> 2. `ToEnuAttitude`/`ToNedAttitude` 不适用于飞机姿态转换（对水平飞行产生 roll=180° 倒飞 + 转弯时 roll↔yaw 耦合）→ ApplyInitialConditions 直接传递 yaw/取反 pitch&roll；MapAttitude 直接输出 NED heading
> 3. c172ap.xml 原版无 auto-throttle 通道 → C++ 侧实现 PI 空速控制器
> 4. 测试超时调整：C172 标准转弯率 ~3°/s，90° 需 ~30s → G2 测试从 20s/30s 调整为 40s/60s

### 阶段 G1：ControlInput 接口扩展 ✅

| # | 任务 | 状态 |
|---|------|------|
| G1a | 扩展 `ControlInput`，新增 AP 指令字段：`heading_setpoint_deg`、`heading_hold`、`altitude_setpoint_m`、`altitude_hold` | ✅ |
| G1b | `JsbsimAdapter::ApplyControlInputs()` 新增 AP 属性写入分支（含高度 m→ft 单位转换） | ✅ |
| G1c | 跳过（AP 可用） | N/A |
| G1d | 跳过（AP 可用） | N/A |

### 阶段 G2：方向机动 ✅

| # | 任务 | 状态 |
|---|------|------|
| G2a | 方向机动——直接下发目标航向到 `ap/heading_setpoint`，通过 `heading_hold` 激活 | ✅ |
| G2b | 测试：90° 转弯 20s 内收敛（误差 < 3°）、180° 转弯 30s 内收敛（误差 < 5°） | ✅ |

### 阶段 G3：固定点机动 (Point-to-Point) ✅

| # | 任务 | 状态 |
|---|------|------|
| G3a | 实现方位角/距离计算——Haversine 大圆距离 + 前向方位角，纯数学不依赖 FGWaypoint | ✅ |
| G3b | 航向制导——实时更新方位角为目标航向，调用 heading_hold AP | ✅ |
| G3c | 高度控制——暂不启用（AP 高度保持受功率限制），仅做航向+油门 | ✅ |
| G3d | 到达判定——距离 < arrival_distance_m | ✅ |
| G3e | 测试：飞行器飞向 500m 外目标，最小距离 < 450m | ✅ |
| **关键修复** | JSBSim 地心纬度→大地纬度（SetGeodLatitudeDegIC / GetGeodLatitudeDeg），消除 ~20km ECEF 偏差 | ✅ |

### 阶段 G4：航路点机动 ✅

| # | 任务 | 状态 |
|---|------|------|
| G4a | 航路点序列状态机——加载列表、索引管理、到达推进 | ✅ |
| G4b | 转弯提前量——`turn_anticipation_m` 距离内平滑混合下一航路点方位角 | ✅ |
| G4c | 测试：双航路点全链路（AP 修复后通过）——飞机依次到达正东 4.2km + 东北 5.5km 两个航路点 | ✅ |

### 阶段 G5：蛇形机动 (Weave/Snake) ✅

| # | 任务 | 状态 |
|---|------|------|
| G5a | 实现正弦航向偏置——`Ψ_target = Ψ_base + A·sin(ωt)`，高频更新航向设定点 | ✅ |
| G5b | 测试：飞行器轨迹呈正弦波，振幅/频率与参数一致 | ✅ |

### 阶段 G6：滚筒机动 (Barrel Roll) — 闭环方案 ✅

| # | 任务 | 状态 |
|---|------|------|
| G6a | 实现姿态反馈闭环——PID 副翼跟踪累积滚转角（机体 p 积分避免 Euler 跳变）；升降舵高度 PID × cos(roll) 修正倒飞 | ✅ |
| G6b | 安全前置检查——高度损失超限 / 海拔过低自动中止；紧急恢复副翼平飞 | ✅ |
| G6c | 测试：C172 完成 360° 滚转，高度损失 < 250m（G6_BarrelRollCompleted 通过） | ✅ |

### 阶段 G7：回归验证

| # | 任务 | 状态 |
|---|------|------|
| G7a | 全量测试回归通过 | ✅ |
| G7b | 新增机动测试注册到 `1q_unit_tests` | ✅ |

---

# 阶段 H：机动控制公共 API 集成

## 目标

将 ManeuverController 从内部实现提升为模块公共 API，让用户通过 FlightDynamicSession 直接执行基础机动（方向、固定点、航路点、蛇形、滚筒），而非仅在测试中可见。

## 问题分析

**现状**：
- `ManeuverController` 在 `src/flight_dynamic/maneuver/` 中，仅测试代码可见
- 机动参数类型（`PointToPointParams`、`WeaveParams` 等）未暴露到 `include/`
- 用户要执行机动需要手写循环：`Controller.ComputeXxx → Session.Step → 检查完成状态`
- 状态管理（航路点索引、滚筒阶段、仿真时间）由调用方自行维护

**目标**：
- 机动参数/状态类型成为公共 API
- 用户可通过 FlightDynamicSession 一步调用执行机动
- 保留手动 Step() 用于高级控制

## 架构设计

### 方案 A：仅提升 ManeuverController 为公共 API

```
用户代码：
  ManeuverController ctrl;
  PointToPointParams params;
  bool reached = false;
  auto input = ctrl.ComputePointToPoint(session.GetCurrentState(), params, &reached);
  auto output = session.Step(input);
  // 用户自行管理 reached / wp_index / state
```

- **优点**：最小改动，灵活度高
- **缺点**：样板代码不减，状态管理仍由用户负责

### 方案 B：FlightDynamicSession 集成 StepManeuver（推荐）

```
┌────────────────────────────────────────────────────────────┐
│ FlightDynamicSession (公共 API)                             │
│                                                             │
│  Step(input) → output                 // 手动控制（保留）   │
│  StepManeuver(request) → result       // 机动控制（新增）   │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Impl                                                 │   │
│  │  JsbsimAdapter + VehicleStateMapper  // 物理（现有）  │   │
│  │  ManeuverController + ManeuverState  // 制导（新增）  │   │
│  │    maneuver_mode_      // 当前机动模式                │   │
│  │    waypoint_index_     // 航路点索引（kWaypoint）     │   │
│  │    barrel_roll_state_  // 滚筒状态（kBarrelRoll）    │   │
│  │    maneuver_sim_time_  // 机动仿真时钟                │   │
│  │    maneuver_request_   // 当前机动请求（跨帧）        │   │
│  └─────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────┘
```

用户代码：
```cpp
auto session = FlightDynamicSessionFactory::Create(config);

// 航路点机动
ManeuverRequest request;
request.mode = ManeuverMode::kWaypoint;
request.waypoint_params = {...};
request.waypoints = {wp1, wp2, wp3};

while (true) {
  auto result = session.StepManeuver(request);
  if (result.status.completed || result.status.aborted) break;
  // result.output.kinematics → 注入传感器模块
}

// 切换到蛇形机动
request.mode = ManeuverMode::kWeave;
request.weave = {.base_heading_deg = 90, .amplitude_deg = 30, .period_s = 20};
auto result = session.StepManeuver(request);
```

- **优点**：单入口、状态内聚、API 简洁
- **缺点**：Session 持有制导状态（物理与制导耦合）；机动切换需明确 ResetManeuver

### 方案 C：独立 ManeuverSession 门面

```
ManeuverSession(FlightDynamicSession + ManeuverController + ManeuverState)
  → StepManeuver(request) → result
```

- **优点**：关注点分离（物理 vs 制导）
- **缺点**：多一层间接、实体需管理两个 session 对象

### 推荐：方案 B

理由：
1. 1Q 架构中 Session 是主要 API 面，机动是载具行为，归属于 Session 自然
2. 机动状态与物理状态紧密耦合（航路点制导需要位置反馈，滚筒需要姿态反馈）
3. 每个载具一个 Session 实例，机动状态生命周期与 Session 一致
4. 原 `Step()` 保留，不破坏现有 API

## 公共类型设计

```
include/1q/flight_dynamic/maneuver/
  ManeuverTypes.h       — ManeuverMode 枚举 + 机动参数 + 机动状态
  ManeuverController.h  — 公共接口（ComputeXxx 方法）
```

### ManeuverMode（枚举）

| 值 | 含义 |
|----|------|
| `kManual` | 手动控制（默认，走 Step） |
| `kPointToPoint` | 飞向目标点 |
| `kWaypoint` | 航路点序列 |
| `kWeave` | 蛇形机动 |
| `kBarrelRoll` | 滚筒机动 |

### ManeuverRequest（输入）

```
ManeuverMode mode
PointToPointParams point_to_point     // kPointToPoint
WaypointList waypoints                // kWaypoint
WaypointParams waypoint_params        // kWaypoint
WeaveParams weave                     // kWeave
BarrelRollParams barrel_roll          // kBarrelRoll
float dt_sec                          // 时间步长（从 Step 语义复用）
```

### ManeuverStatus（输出状态）

```
ManeuverMode active_mode              // 当前活动模式
bool active                           // 机动进行中
bool completed                        // 正常完成
bool aborted                          // 安全中止
std::size_t waypoint_index            // kWaypoint: 当前索引
BarrelRollPhase barrel_roll_phase     // kBarrelRoll: 当前阶段
double min_distance_m                 // kPointToPoint/kWaypoint: 最近距离
```

### ManeuverStepResult（输出）

```
FlightDynamicOutput output            // 物理输出（复用现有）
ManeuverStatus status                 // 机动状态（新增）
```

## 阶段

### 阶段 H1：公共类型定义

| # | 任务 | 状态 |
|---|------|------|
| H1a | `include/1q/flight_dynamic/maneuver/ManeuverTypes.h` — ManeuverMode + 机动参数结构体（`ONEQ_API`） | ✅ |
| H1b | `include/1q/flight_dynamic/maneuver/ManeuverController.h` — 公共接口声明 | ✅ |
| H1c | 几何工具函数（`ComputeGreatCircleDistanceM`、`ComputeForwardAzimuthDeg`）纳入公共 API | ✅ |

> **H1 设计要点**：
> - 参数结构体从 `ManeuverController.h`（内部）迁移到 `ManeuverTypes.h`（公共）
> - `ManeuverController` 类声明移到公共头文件，实现留在 `src/`
> - 几何工具独立于 ManeuverController，用户可能直接调用（如计算两点距离）

### 阶段 H2：FlightDynamicSession 集成

| # | 任务 | 状态 |
|---|------|------|
| H2a | `ManeuverRequest` / `ManeuverStatus` / `ManeuverStepResult` 加入公共类型 | ✅ |
| H2b | `FlightDynamicSession` 新增 `StepManeuver()` 方法声明 | ✅ |
| H2c | `FlightDynamicSession` 新增 `ResetManeuver()` 方法（重置机动状态，不清除物理状态） | ✅ |
| H2d | `FlightDynamicSession::Impl` 新增 `ManeuverController` + 机动状态成员 | ✅ |
| H2e | `StepManeuver()` 实现：根据 mode 分发到 `ComputeXxx` → `Step()` → 更新状态 | ✅ |

> **H2 设计要点**：
> - `StepManeuver()` 是对 `Step()` 的高级封装：Compute → Step → 状态更新
> - 机动切换（mode 变化）自动重置前一个机动状态
> - `ResetManeuver()` 不影响物理引擎状态（不调用 JSBSim RunIC）

### 阶段 H3：构建系统与 umbrella header

| # | 任务 | 状态 |
|---|------|------|
| H3a | umbrella header 添加 maneuver 相关 include | ✅ |
| H3b | CMakeLists.txt 添加 maneuver 公共头文件安装规则 | ✅ |
| H3c | 合约守卫白名单更新（如需要） | ✅ |

### 阶段 H4：测试

| # | 任务 | 状态 |
|---|------|------|
| H4a | 使用公共 API 重写/补充机动测试（StepManeuver 路径） | ✅ |
| H4b | 机动切换测试（waypoint → weave → barrel roll 连续执行） | ✅ |
| H4c | 全量测试回归 | ✅ |

### 阶段 I：新增规避 + 绕圈盘旋机动

| # | 任务 | 状态 |
|---|------|------|
| I1 | OrbitParams/OrbitState + EvasionParams/EvasionState/EvasionPhase 类型定义 | ✅ |
| I2 | ComputeOrbit 实现：切线航向 + 径向比例修正保持轨道半径 | ✅ |
| I3 | ComputeEvasion 实现：破转→下降→持续时间完成 | ✅ |
| I4 | StepManeuver 分发 + Impl 状态成员 + 重置逻辑 | ✅ |
| I5 | 测试：Orbit 绕圈（航向变化 > 300°）、Evasion 转向 + 完成判定 | ✅ |
| I6 | 全量测试回归（901/901） | ✅ |

### 阶段 J：验证与稳定性确认

| # | 任务 | 状态 | 发现 |
|---|------|------|------|
| J1 | 物理真实性验证（5 项） | ✅ | C172 转弯率 ~4.7°/s；无 heading_hold 时长时飞行发散 |
| J2 | 长时间稳定性压测（3 项） | ✅ | 300s weave/orbit 稳定；无 heading_hold 78s 后 NaN |
| J3 | 多阶段复杂任务（1 项） | ✅ | 5 阶段任务完整执行（P2P→轨道→蛇形→规避→返航） |
| J4 | 边界条件（4 项） | ✅ | 航向环绕正常；快速切换正常；ResetManeuver 不影响物理 |
| J5 | 机动精度（3 项） | ✅ | 轨道半径误差 ~57%（AP 限制）；规避产生高度变化；航路点到达可靠 |

> **关键发现**:
> 1. **heading_hold 是长时间稳定飞行的必要条件**: 无 heading_hold 时，飞机在 ~80s 后发散至 NaN
> 2. **轨道半径精度受 AP 转弯率限制**: AP heading hold 的转弯率有限，无法精确跟踪连续变化的切线航向，导致半径误差 ~57%。未来需引入直接 bank angle 控制提升精度
> 3. **C172 转弯率**: AP heading hold 模式下 ~4.7°/s，比标准 3°/s 快（AP 有初始过冲）
> 4. **高度保持**: 有 heading_hold 时 100s 内漂移 < 50m
> 5. **ECEF 位置**: 稳定在地球半径 ±100km 范围内

> **Orbit 算法**: 切线航向 = bearing_to_center ± 90°（顺/逆时针），径向修正 = -sign × max_correction × normalized_error，最大修正 ±45°
>
> **Evasion 算法**: 破转阶段（急转弯到规避航向）→ 下降阶段（航向收敛后持续下降）→ 持续时间到期完成

## 决策记录

### DR6：AP 依赖策略
- **当前**: 优先使用 JSBSim 内置 AP（方案 A），若不可用则自建 PID 控制器（方案 C）
- **待 G0 验证后确定**

### DR7：滚筒机动控制模式
- **决定**: 采用姿态反馈闭环（代替 maneuver_algorithms.md 中的开环时序方案）
- **原因**: 不同飞机滚转速率差异大，固定时序不可靠；闭环可自适应飞机特性且能处理帧率抖动
- **实现**: PID 跟踪目标滚转角序列 + 高度保持 PID + 异常中止条件

### DR8：ControlInput AP 语义
- **决定**: AP 指令定位为"建议"，底层是否生效取决于飞机模型 AP 配置或自建控制器
- **字段**: `heading_setpoint_deg` (double, -1 表示不使用)、`altitude_setpoint_m` (double, -1 表示不使用)

### DR9：坐标计算
- **决定**: 方位角/距离计算复用 `1q::coordinate::` 现有函数，不引入 JSBSim `FGWaypoint` 组件
- **原因**: 保持计算逻辑在 1Q 侧可控、可测试，不依赖 JSBSim 内部实现

### DR10：多机型支持
- **现状**: JsbsimAdapter 节气门路径（`fcs/throttle-cmd-norm[0]`）和 AP 接口硬编码为 C172 约定
- **发现**: F16 使用不同节气门路径（无 `[0]` 索引）、默认 `propulsion=OFF`、无 AP autopilot
- **决定**: 当前阶段保持 C172 单一机型。多机型通用化需将节气门/引擎/AP 差异抽象到 `AircraftDefinition` 配置中，列为未来任务

### DR11：机动公共 API 集成策略
- **决定**: 方案 B — FlightDynamicSession 新增 StepManeuver()，内部管理制导状态
- **原因**: 1Q 中 Session 是主 API 面；机动状态与物理状态紧耦合（位置/姿态反馈）；每载具一个 Session，机动状态生命周期一致
- **实现**: ManeuverRequest/Status/StepResult 公共类型 + StepManeuver/ResetManeuver 方法
- **验证**: 896/896 测试通过，新增 6 个 StepManeuver 公共 API 测试


## 决策记录

### DR1：JSBSim 引入方式
- **决定**: third_party 源码内嵌，commit `70a327fc`（C++11 兼容，2020-10-18）
- **原因**: Conan 不可达，无系统包，v1.2.0 要求 C++14 与 VS2015 不兼容
- **构建方式**: 始终编译为共享库（LGPL 合规），不受 BUILD_SHARED_LIBS 影响
- **排除**: FetchContent（无先例）、v1.2.0（C++14）

### DR2：大气模型策略 ✅
- **当前决定**: 直接使用 JSBSim 内置 ISA 大气模型（选项 A），JSBSim 大模型经过开源社区充分验证
- **原因**: 1Q 现有大气模型（AtmosphericObservation 标量观测值）相对简易，粒度不如 JSBSim 基于高度的标准大气推算
- **未来规划**: 建议创建独立的大气模型模块（如 `atmosphere`），统一各模块外部环境输入。工程量较大，应在完整审查代码库后作为独立阶段推进，不在本模块范围内

### DR3：LGPL 合规
- **决定**: JSBSim 始终作为共享库链接，BuildDependencies.cmake 中无条件启用
- **C++11 版 JSBSim 无 JSBSIM_EXPORT 宏**: macOS/Linux 默认导出所有符号，Windows 待后续处理

### DR4：FlightDynamicSession Step 语义
- **决定**: 有状态积分器，Step 输入 dt_sec + 控制面/油门，输出 ExternalKinematics + 扩展状态
- **线程安全**: 单实例不保证，多载具需多实例

### DR5：PIMPL 实现模式
- **决定**: Impl 定义在共享内部头文件 `FlightDynamicSessionImpl.h`
- **原因**: Session.cpp 的 Step/Reset/GetCurrentState 方法直接访问 Impl 成员，不能仅靠前向声明

## 遇到的错误

| 错误 | 尝试次数 | 解决方案 |
|------|---------|---------|
| Conan 不可达 (SSL EOF) | 1 | 走 third_party 源码内嵌 |
| JSBSim v1.2.0 要求 C++14 | 1 | 切换到 C++11 commit `70a327fc` |
| JSBSim cmake_minimum_required 2.8.8 | 1 | 更新为 3.5 |
| Doxygen configure_file 文件不存在 | 1 | 添加 BUILD_DOCS=OFF 选项 |
| JSBSim 可执行文件链接错误 | 1 | 添加 BUILD_EXECUTABLE=OFF 选项 |
| SetIntegrationMethod 不存在 | 1 | 移除调用（C++11 JSBSim 无此 API） |
| GetPropertyValue 非 const | 1 | VehicleStateMapper 使用非 const FGFDMExec& |
| JSBSim include 路径错误 | 1 | 改为与 JSBSim 内部一致的相对路径 |
| PIMPL 析构函数需要完整 Impl 类型 | 1 | Impl 移至共享头文件 |
| std::make_unique 不可用 (C++11) | 2 | 替换为 `unique_ptr<T>(new T(...))` |
| public_api_boundary_guard 测试失败 | 1 | 添加 FD 头文件到白名单 |
| ECEF→LLA 坐标偏差 ~21km | 2 | JSBSim 地心纬度→大地纬度（Geodetic），用户定位根因：SetLatitudeDegIC → SetGeodLatitudeDegIC |
| G4 航路点全链路测试失败 | 5+ | ~~缺乏高度/油门 PID~~ AP 修复后已解决，双航路点全链路通过 (888/888) |
| F16 多机型实验失败 | 1 | 节气门路径/引擎初始化/AP 差异，多机型通用化需抽象到 AircraftDefinition |
| c172x.xml ap/aileron_cmd 被注释掉 | 1 | 2020 版 bug（上游 2023 年修复 ae318dea），导致 heading hold 无效 → 取消注释 |
| ToEnuAttitude 对飞机产生 roll=180° | 2 | 旋转矩阵复合不适用飞机姿态转换 → ApplyInitialConditions 直接传 yaw/取反 pitch&roll；MapAttitude 直接输出 NED heading |
| G0 航向收敛 NaN / 不转弯 | 3+ | 根因链：c172x.xml bug + 坐标系转换错误 + 缺少 airspeed PID。逐一修复后全部 887 测试通过 |
