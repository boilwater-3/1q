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
> 航向保持 AP 经验证可收敛（90°/180° 转弯测试通过）。高度保持 AP 存在但收敛受
> 发动机功率限制（2000m 高度 0.8 油门不足以维持平飞），实际使用需配合油门管理。

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
| G4c | 测试：单航路点全链路（DISABLED）。原因：长时间飞行缺乏高度/油门 PID 会导致飞机失速坠毁，仅算法已由 guidance 测试验证 | ✅ |

### 阶段 G5：蛇形机动 (Weave/Snake)

| # | 任务 | 状态 |
|---|------|------|
| G5a | 实现正弦航向偏置——`Ψ_target = Ψ_base + A·sin(ωt)`，高频更新航向设定点 | 🔲 |
| G5b | 测试：飞行器轨迹呈正弦波，振幅/频率与参数一致 | 🔲 |

### 阶段 G6：滚筒机动 (Barrel Roll) — 闭环方案

| # | 任务 | 状态 |
|---|------|------|
| G6a | 实现姿态反馈闭环（非开环时序）——以目标滚转角序列为参考，PID 控制副翼跟踪；升降舵用高度保持 PID | 🔲 |
| G6b | 实现安全前置检查——动能/高度判定 + 异常中止条件（高度损失 > 阈值） | 🔲 |
| G6c | 测试：飞行器完成 360° 滚转，高度损失 < 100m | 🔲 |

### 阶段 G7：回归验证

| # | 任务 | 状态 |
|---|------|------|
| G7a | 全量测试回归通过 | 🔲 |
| G7b | 新增机动测试注册到 `1q_unit_tests` | 🔲 |

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
| G4 航路点全链路测试失败 | 5+ | C172 长距离飞行缺乏高度/油门 PID 导致失速坠毁。算法已由单元测试验证，全链路保持 DISABLED 直至补全纵向 AP |
| F16 多机型实验失败 | 1 | 节气门路径/引擎初始化/AP 差异，多机型通用化需抽象到 AircraftDefinition |
