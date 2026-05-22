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
