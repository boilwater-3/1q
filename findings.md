# 研究发现 — JSBSim 飞行动力学集成

## 1. JSBSim 许可与跨平台

- **核心 C++ 库**: LGPL 2.1+（可动态链接集成到闭源项目）
- **Unreal 插件**: MIT；**MATLAB 接口**: BSD；**JSBSim.py 脚本**: GPL 3+
- **跨平台**: Windows (MSVC/MinGW) / Linux (GCC/Clang) / macOS (Clang)，CI 验证
- **构建系统**: CMake，与 1Q 一致
- **依赖**: 无外部依赖（Expat 内嵌），可选 SYSTEM_EXPAT=ON

## 2. JSBSim C++ API 集成模式

```cpp
JSBSim::FGFDMExec FDMExec;
FDMExec.SetRootDir(RootDir);
FDMExec.SetAircraftPath(SGPath("aircraft"));
FDMExec.SetEnginePath(SGPath("engine"));
FDMExec.SetSystemsPath(SGPath("systems"));
FDMExec.LoadModel("c172");  // 注意：新版 API 为单参数，路径已预设
FDMExec.SetPropertyValue("ic/h-sl-ft", 10000.0);
FDMExec.SetPropertyValue("ic/vc-kts", 100.0);
FDMExec.RunIC();
while (FDMExec.Run()) {
  double alt = FDMExec.GetPropertyValue("position/h-sl-ft");
  double spd = FDMExec.GetPropertyValue("velocities/vc-kts");
}
```

> ⚠️ **API 签名需在阶段 A PoC 中实际验证**。早期版本使用 4 参数
> `LoadModel("aircraft", "engine", "systems", "c172")`，新版已简化为单参数。

- **Property 系统**: `GetPropertyValue(string)` / `SetPropertyValue(string, double)` — 层次化命名路径
- **FGPropagate**: 负责运动方程积分，提供 `VehicleState` 结构体
- **积分方案**: Euler / Trapezoidal / Adams-Bashforth 2-5 / Buss 1-2 / LocalLinearization

## 3. FGPropagate 状态向量

| 字段 | 坐标系 | 说明 |
|------|--------|------|
| `vLocation` | ECEF | 位置 |
| `vUVW` | Body | 相对 ECEF 的速度 |
| `vPQR` | Body | 相对 ECEF 的角速度 |
| `vPQRi` | Body | 相对 ECI 的角速度 |
| `qAttitudeLocal` | Body→NED | 局部姿态四元数 |
| `qAttitudeECI` | Body→ECI | 惯性姿态四元数 |
| `vInertialVelocity` | ECI | 惯性速度 |
| `vInertialPosition` | ECI | 惯性位置 |

## 4. 坐标系映射 (1Q ↔ JSBSim)

| 维度 | 1Q | JSBSim | 映射 |
|------|-----|--------|------|
| 全局位置 | ECEF (`double`) | ECEF (`FGLocation`) | **直接对齐** |
| 局部位置 | ENU (E-N-U) | NED (N-E-D) | 轴重排 + Z取反 |
| 全局速度 | ECEF 速度 | Body 系 `vUVW` | 需旋转矩阵 |
| 姿态 | Body→ENU 欧拉角 (Z-Y-X) | Body→NED 四元数 | 四元数→欧拉角 + NED→ENU |
| 角速度 | — | Body 系 `vPQR` | 直接映射 |
| 单位 | SI (m, m/s, deg) | 英制 (ft, kts) | 系数转换 |

**1Q 已有转换工具可直接复用** (namespace `oneq::coordinate`):

### 位置转换 (`coordinate/position_transform.h`)

| 函数 | 签名 | 用途 |
|------|------|------|
| `ToEnu` | `EnuPositionM ToEnu(const NedPositionM& ned)` | NED→ENU 轴重排 |
| `ToNed` | `NedPositionM ToNed(const EnuPositionM& enu)` | ENU→NED 轴重排 |
| `TryEcefToEnu` | `bool TryEcefToEnu(const EcefPositionM&, const LlaPositionDegM& origin, EnuPositionM*)` | ECEF→ENU |
| `TryEnuToEcef` | `bool TryEnuToEcef(const EnuPositionM&, const LlaPositionDegM& origin, EcefPositionM*)` | ENU→ECEF |
| `TryEcefToLla` | `bool TryEcefToLla(const EcefPositionM&, LlaPositionDegM*)` | ECEF→LLA（JSBSim IC 需要） |
| `TryLlaToEcef` | `bool TryLlaToEcef(const LlaPositionDegM&, EcefPositionM*)` | LLA→ECEF |

### 速度转换 (`coordinate/velocity_transform.h`)

| 函数 | 签名 | 用途 |
|------|------|------|
| `TryEnuToEcefVelocity` | `bool TryEnuToEcefVelocity(const EnuVelocityMps&, const LlaPositionDegM& origin, EcefVelocityMps*)` | ENU→ECEF 速度 |
| `TryEcefToEnuVelocity` | `bool TryEcefToEnuVelocity(const EcefVelocityMps&, const LlaPositionDegM& origin, EnuVelocityMps*)` | ECEF→ENU 速度 |
| `ToEnuVelocity` | `EnuVelocityMps ToEnuVelocity(const NedVelocityMps&)` | NED→ENU 速度轴重排 |
| `ToNedVelocity` | `NedVelocityMps ToNedVelocity(const EnuVelocityMps&)` | ENU→NED 速度轴重排 |

> ⚠️ 注意：速度轴重排函数名带 `Velocity` 后缀（`ToEnuVelocity`），与位置函数（`ToEnu`）命名不同。

### 姿态转换 (`coordinate/attitude_transform.h`)

| 函数 | 签名 | 用途 |
|------|------|------|
| `ToEnuAttitude` | `EulerAnglesDeg ToEnuAttitude(const EulerAnglesDeg& ned)` | NED→ENU 姿态 |
| `ToNedAttitude` | `EulerAnglesDeg ToNedAttitude(const EulerAnglesDeg& enu)` | ENU→NED 姿态 |
| `BuildRotationMatrix` | `RotationMatrix3d BuildRotationMatrix(const EulerAnglesDeg&)` | 欧拉角→旋转矩阵 |
| `ToEulerAnglesDeg` | `EulerAnglesDeg ToEulerAnglesDeg(const RotationMatrix3d&)` | 旋转矩阵→欧拉角 |
| `RotateEnuToLocal` | `Vector3d RotateEnuToLocal(double e, double n, double u, const EulerAnglesDeg&)` | ENU→本体系向量旋转 |
| `RotateLocalToEnu` | `Vector3d RotateLocalToEnu(double x, double y, double z, const EulerAnglesDeg&)` | 本体系→ENU 向量旋转 |

## 5. ExternalKinematics 完整定义

```cpp
// include/1q/coordinate/types.h
namespace oneq { namespace coordinate {

enum class PositionFrame { kEcef = 0, kLla = 1 };

struct ONEQ_API ExternalKinematics {
  PositionFrame position_frame{PositionFrame::kEcef};  // 支持两种位置帧
  EcefPositionM position_ecef_m{};                     // ECEF 位置 (kEcef 时使用)
  LlaPositionDegM position_lla_deg_m{};                // LLA 位置 (kLla 时使用)
  EcefVelocityMps velocity_mps{};                      // 速度 (始终 ECEF, m/s)
  EulerAnglesDeg attitude_deg{};                       // 姿态 (Body→ENU, deg, Z-Y-X)
};

}}
```

> **关键设计**: `ExternalKinematics` 通过 `position_frame` 枚举支持 ECEF 和 LLA 两种位置帧。
> `FlightDynamicSession` 输出应产出此格式，让现有传感器模块 adapter 无需修改即可消费。

## 6. 外部力/力矩注入

- JSBSim 通过 `FGAccelerations` 计算加速度，`FGPropagate` 积分
- 可通过 Property 或 `FGExternalReactions` 注入外部力
- 自定义大气参数: `FGAtmosphere::SetTemperature()` / `SetPressureSL()`

## 7. 1Q 架构关键参考

- **Session 模式**: `RadarSession` (PIMPL, Step-based, non-copyable, movable)
  - `Step(const RadarCycleInput&) → TrackOutputFrame`
  - `StepWithResult(const RadarCycleInput&) → RadarCycleResult`
- **Factory 模式**: `RadarSessionFactory::Create(const RadarSessionConfig& = {})` 返回 `RadarSession` (by value/move)
  - Factory 是 RadarSession 的 friend class
- **输入结构** (`RadarCycleInput`):
  - `cycle_index` (`uint32_t`) — 周期号
  - `dt_sec` (`float`) — 步长
  - `platform_altitude_m` (`float`) — WGS84 海拔（**计划初版遗漏**）
  - `platform_pose` (`oneq::foundation::PoseState`) — 局部位姿
  - `scene` (`RadarSceneTargetList`) — 场景目标
  - `environment` (`RadarEnvironmentInput`) — 环境输入
- **运动学**: `ExternalKinematics` (见第 5 节)
- **OBJECT 库模式**: 每个 domain 拆分为 `_engine` + `_core` OBJECT 库，使用**缩写命名**：
  - `airborne_radar` → `airborne_engine` + `airborne_core`
  - `electronic_surveillance_radar` → `esr_engine` + `esr_core`
  - `electro_optical_sensor` → `eos_engine` + `eos_core`
- **公共类型必须标记 `ONEQ_API`** 导出宏（定义在 `include/1q/api.hpp`）

## 8. AtmosphericObservation 实际定义

```cpp
// include/1q/foundation/atmospheric_types.h
namespace oneq { namespace foundation {

struct ONEQ_API AtmosphericObservation {
  bool enable_physical_model{false};
  float pressure_hpa{1013.25f};       // 气压 (hPa)
  float temperature_k{288.15f};       // 温度 (K)
  float relative_humidity{0.5f};      // 相对湿度 [0, 1]
};

}}
```

> JSBSim 内置 ISA 大气模型基于高度推算温度/气压/密度，粒度与此结构体不同。
> 对接策略需在实现阶段明确（注入/反向提取/独立运行）。

## 9. 依赖管理现状

- **主方案**: Conan（`conanfile.py`），现有依赖: eigen/3.4.0, boost/1.83.0, nanoflann, flatbuffers, zlib, gtest
- **备选**: vcpkg（`cmake/PackageManagerVcpkg.cmake`）
- **兜底**: `third_party/` 源码内嵌（每个依赖提供 INTERFACE/OBJECT target）
- **无 FetchContent 先例** — 代码库中不存在 FetchContent 用法
- **依赖接线**: `cmake/ProjectDependencies.cmake` — 在 targets 创建后调用

## 10. JSBSim 可用性调研 (2026-05-22)

- **Conan**: 不可达 — center2.conan.io SSL EOF 错误
- **Homebrew**: 未安装 `jsbsim` formula
- **pkg-config**: 未找到 JSBSim
- **系统搜索**: 未找到 `FGFDMExec.h` 头文件
- **结论**: 唯一可行路径是 `third_party/` 源码内嵌

### third_party 集成方案

参考现有模式（flatbuffers, zlib）：

```
third_party/jsbsim/
  CMakeLists.txt              — 构建 JSBSim 为 OBJECT 库
  jsbsim-<version>/           — JSBSim 上游源码（需 git clone 或下载 tar）
    src/                      — JSBSim 核心源码
    ...
```

**JSBSim 源码获取**:
- 官方仓库: https://github.com/JSBSim-Team/jsbsim
- 最新稳定版: v1.2.0 (2024-09)
- CMake 构建，零外部依赖（Expat 内嵌）

**third_party CMakeLists.txt 模式**:
- 参考 `flatbuffers_objects` OBJECT 库：直接列出源文件，设置 include 路径
- 设置 `CXX_STANDARD 11` 与项目一致
- 创建 INTERFACE 目标 `jsbsim_interface` 供 `ProjectDependencies.cmake` 使用
- ALIAS `JSBSim::JSBSim` 指向 INTERFACE 目标

**需要注意**:
- JSBSim 使用 `SGPath` 等自己的路径类型，需要确保包含路径正确
- JSBSim 的 `FGLocation` 是 1-indexed（`loc(1)` = x），代码中已正确处理
- JSBSim 内部 Expat XML 解析器可静态编译，无需外部依赖

## 11. 已有代码审查发现 (2026-05-22)

- **命名空间**: `flight_dynamic::*` vs 其他模块 `oneq::<module>::*` — 风格不一致
- **ONEQ_API 宏**: 所有公共类型已正确标记
- **PIMPL 模式**: 最终采用共享内部头文件（Session.cpp 方法直接访问 Impl 成员）
- **坐标转换**: 完整覆盖 ECEF、NED→ENU、四元数→欧拉角、英制→SI
- **构建系统**: 全部完成（模块 CMakeLists.txt + 顶层集成 + 共享库链接）

## 12. JSBSim C++11 版本 (commit 70a327fc) API 差异

与 v1.2.0 的关键 API 差异：

| API | C++11 版 (70a327fc) | v1.2.0 |
|-----|---------------------|--------|
| `GetIC()` | `FGInitialCondition*` (raw ptr) | `shared_ptr<FGInitialCondition>` |
| `GetPropagate()` | `FGPropagate*` | `FGPropagate*` (相同) |
| `GetAccelerations()` | `FGAccelerations*` | `FGAccelerations*` (相同) |
| `GetPropertyValue()` | 非 const | 非 const (相同) |
| `SetIntegrationMethod()` | 不存在 | 存在于 v1.2.0 |
| `JSBSim_API.h` | 不存在 | 存在（DLL 导出） |
| C++ 标准 | C++11 | C++14 |
| CMake 最低版本 | 2.8.8 | 3.15 |

**C++11 版限制**:
- 无 DLL 导出宏（JSBSIM_EXPORT），Windows 共享库构建需后续处理
- utils/aeromatic++ 仍会编译但无影响
- FGXMLParse 有 vtable 链接问题（仅影响 JSBSim 可执行文件，已禁用）

## 13. JSBSim 机动模式调研 (2026-05-22)

通过 DeepWiki 查询 JSBSim 对五种机动模式的原生支持情况：

### JSBSim 内置能力

| 能力 | 支持程度 | 实现方式 |
|------|---------|---------|
| 方向/航向保持 | **内置** | `ap/heading_setpoint` + `ap/heading_hold` 属性，直接通过 `SetPropertyValue()` 控制 |
| 高度保持 | **内置** | `ap/altitude_setpoint` + `ap/altitude_hold` 属性 |
| 航路点导航 | **部分内置** | `FGWaypoint` 组件可计算目标方位角和距离，但无内置制导律 |
| 固定点/轨道 | **无** | 需组合 FGWaypoint + PID 控制器 + 油门/副翼/升降舵逻辑 |
| 蛇形/S转弯 | **无** | 需通过脚本事件系统定时操控舵面 |
| 滚筒 | **无** | 需通过脚本事件系统协调副翼/升降舵时序 |

### JSBSim FCS 组件清单

可用于构建制导律的低级组件：
- `pid` / `integrator` — PID 控制器
- `switch` — 条件逻辑
- `summer` — 求和器
- `gain` — 增益/调度增益
- `filter` — 滞后/超前/洗出/二阶滤波
- `deadband` — 死区
- `actuator` — 作动器（速率限制、滞后、死区）
- `sensor` / `accelerometer` / `gyro` / `magnetometer` — 传感器
- `kinematic` — 机械连杆
- `waypoint_heading` / `waypoint_distance` — 航路点方位/距离

### JSBSim 脚本事件系统

- XML 脚本：`<event>` 标签，基于时间或属性条件触发
- 动作类型：`FG_RAMP`（斜坡）、`FG_EXP`（指数）、直接赋值
- 持久事件：条件保持期间重复执行
- 典型用途：起飞序列、突风注入、舵面脉冲测试

### 架构影响

JSBSim **不提供高层机动原语**。五种机动模式都需要在 1Q 端实现制导/控制逻辑：

1. **方向机动** → 直接用 `ap/heading_setpoint`，trivial
2. **固定点机动** → 1Q 端计算相对目标点的方位/距离，控制航向+油门
3. **航路点机动** → 1Q 端管理航路点序列，到达判定，切换逻辑
4. **蛇形机动** → 1Q 端生成正弦航向偏置，叠加到基础航向
5. **滚筒机动** → 1Q 端生成时序化的副翼/升降舵控制指令

**推荐架构**：在 1Q 端实现 `ManeuverController` 层，根据机动模式计算控制输入 (`FlightDynamicInput.control`)，注入 `FlightDynamicSession::Step()`。保持 JSBSim 仅作为物理引擎，制导律完全在 1Q 侧。

## 14. 构建系统架构决策

- **JSBSim 始终共享库**: `cmake/ProjectDependencies.cmake` 中无条件 `add_subdirectory(jsbsim)`，设置 `BUILD_SHARED_LIBS=ON` 后恢复原值
- **不依赖 Conan**: JSBSim 无 Conan 包，不走 find_package 路径
- **C++11 兼容**: fd_engine/fd_core 无需提升 C++ 标准，与项目默认一致
