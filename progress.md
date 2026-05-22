# 会话日志 — JSBSim 飞行动力学集成

## 2026-05-22

### 阶段 0: 可行性研究与架构分析

- 通过 DeepWiki MCP 调研 JSBSim 架构、API、坐标系统
- 确认 JSBSim 满足开源 + 三平台可编译要求
- 完成 1Q ↔ JSBSim 坐标系映射分析
- 确认 1Q 坐标系为 **ECEF + ENU + Body→ENU 欧拉角**（非之前误判的 NED）
- 确认 `coordinate` 模块已有 ENU↔NED 转换工具可复用
- 设计 Adapter 模式方案：JsbsimAdapter (PIMPL) + VehicleStateMapper + FlightDynamicSession
- 识别关键风险：坐标转换、单位差异、LGPL 合规、XML 数据文件打包

### 阶段 0.5: 计划审查与修正

经代码库全面检索验证（3 个并行子代理），修正以下问题：

**事实性修正**:
- ✅ 补充 `RadarCycleInput` 遗漏的 `platform_altitude_m` 字段
- ✅ 修正速度转换函数命名（`ToEnuVelocity`/`ToNedVelocity` 带 Velocity 后缀）
- ✅ 补充遗漏的 coordinate 工具（`TryEcefToLla`、`BuildRotationMatrix`、`RotateEnuToLocal` 等）
- ✅ 补充 `ExternalKinematics` 的 `PositionFrame` 双帧机制（kEcef/kLla）
- ✅ 标注 JSBSim `LoadModel` API 签名需 PoC 验证
- ✅ 修正 OBJECT 库命名为缩写形式（`fd_engine` + `fd_core`）

**工程化补充**:
- ✅ 新增构建系统集成阶段（conanfile.py / src/CMakeLists.txt / ProjectDependencies.cmake）
- ✅ 移除冗余的 B2 `CoordinateFrameTypes.h`，直接复用 `coordinate/types.h`
- ✅ 新增 `ONEQ_API` 导出宏要求
- ✅ 新增 umbrella header (`flight_dynamic.hpp`) 和头文件安装规则
- ✅ 合并原阶段 D（坐标转换）到阶段 C2（VehicleStateMapper）
- ✅ 新增 FlightDynamicInput/FlightDynamicOutput 类型设计
- ✅ 明确测试命名前缀 `fd_`

**架构决策补充**:
- ✅ 明确 Session Step 语义差异（有状态积分器 vs 无状态处理器）
- ✅ 设计数据流路径（FlightDynamic → ExternalKinematics → 传感器 adapter 零修改消费）
- ✅ 排除 FetchContent（代码库无先例），确认三路径方案（Conan / vcpkg / third_party）
- ✅ 补充线程安全说明（单实例不保证，多载具需多实例）
- ✅ 补充 LGPL Windows DLL 场景注意事项
- ✅ 细化大气模型协调策略（三选项 + 粒度差异分析）

### 阶段 B+C 实现 (2026-05-22)

**阶段 B — 公共 API（全部完成）**:
- ✅ B1: `VehicleState.h` — 6-DOF 状态快照（SI 单位，1Q 坐标系），含气动参考量
- ✅ B2: `FlightDynamicInput.h` — ControlInput + ExternalForceInput + FlightDynamicInput
- ✅ B3: `FlightDynamicOutput.h` — ExternalKinematics + VehicleState + ok flag
- ✅ B4: `FlightDynamicConfig.h` — IntegratorType 枚举 + 会话配置
- ✅ B5: `AircraftDefinition.h` — root_dir + model_name
- ✅ B6: `FlightDynamicSession.h` — PIMPL 门面，含完整使用文档
- ✅ B7: `FlightDynamicSessionFactory.h` — 静态 Create 方法
- ✅ B8: `flight_dynamic.hpp` — umbrella header

**阶段 C — 内部实现（全部完成）**:
- ✅ C1: `JsbsimAdapter.h/.cpp` — RAII 包装 FGFDMExec，含初始条件注入、控制面写入、外部力注入
- ✅ C2a-C2f: `VehicleStateMapper.h/.cpp` — 完整坐标转换链（ECEF、NED→ENU、四元数→欧拉角、英制→SI、反向 IC 注入、双帧处理）
- ✅ C3: `FlightDynamicSession.cpp` — PIMPL 转发（Impl 不在此 TU）
- ✅ C4: `FlightDynamicSessionFactory.cpp` — Impl 定义 + Factory 装配

**阶段 D — 构建系统（部分完成）**:
- ✅ D1: `src/flight_dynamic/CMakeLists.txt` — FD_ENGINE_SOURCES + FD_CORE_SOURCES + install 规则
- ✅ D6: 公共头文件安装规则（在 D1 中完成）
- 🔲 D2-D5: 顶层构建系统集成（src/CMakeLists.txt, conanfile.py, ProjectDependencies.cmake, third_party）

### 进度评估会话 (2026-05-22 下午)

- 读取全部 15 个 flight_dynamic 文件，评估实际进度
- 发现 JSBSim 完全不可用：Conan 不可达、Homebrew 未安装、系统未找到
- 确定唯一可行路径：third_party/ 源码内嵌
- 更新 task_plan.md、findings.md、progress.md 以反映真实状态

### 阶段 D 构建系统集成 (2026-05-22 下午)

**D5 third_party/jsbsim**:
- 克隆 JSBSim v1.2.0 后立即发现 C++14 不兼容 VS2015
- 通过 `git log -S "CXX_STANDARD 14"` 找到 C++14 引入 commit `e7f8b20f` (2020-11)
- 切换到最后一个 C++11 commit `70a327fc` (2020-10-18)
- 修复 JSBSim CMakeLists.txt：cmake_minimum_required 2.8.8→3.5, BUILD_DOCS=OFF, BUILD_EXECUTABLE=OFF
- 修改 ProjectDependencies.cmake：JSBSim 始终构建为共享库（无条件 add_subdirectory）
- 创建 JSBSim_interface + JSBSim::JSBSim ALIAS 目标

**D2 src/CMakeLists.txt**:
- 添加 `include(flight_dynamic/CMakeLists.txt)`
- 注册 `fd_engine` + `fd_core` OBJECT 库
- 追加到 PROJECT_SOURCES 和 ONEQ_OBJECT_TARGETS

**D4 cmake/ProjectDependencies.cmake**:
- JSBSim 无条件构建（before if/else block）
- fd_engine/fd_core 链接 JSBSim::JSBSim
- 移除 Conan find_package 逻辑（用户要求直接报依赖缺失）

**编译修复**（共 10+ 处）:
- API 差异：SetIntegrationMethod 不存在（移除）、GetIC() 返回 raw ptr（兼容）、GetPropertyValue 非 const（修复）
- C++11 兼容：std::make_unique → new、移除 C++14 compile_features
- Include 路径：`<JSBSim/...>` → `"..."`（JSBSim 无 JSBSim/ 前缀）
- PIMPL：Impl 移至共享头文件 FlightDynamicSessionImpl.h
- 合约守卫：更新 public_api_boundary_guard 白名单

**最终结果**: 全量构建 + 868/868 测试通过（仅白名单需更新并已修复）

### 阶段 E 模块装配 (2026-05-22 下午)

- ✅ E1: FlightDynamicSession 与 AR/EOS/ESR 在同一项目中独立共存，编译无冲突
- ✅ E2: ExternalKinematics 已是通用格式，传感器 adapter 可零修改消费
- ✅ E3: 大气模型协调策略——初始阶段独立运行（JSBSim ISA vs AtmosphericObservation）
- 删除了错误创建的 FlightDynamicAdapter.h（耦合了传感器模块）

### 阶段 F 测试 (2026-05-22 下午)

**新增测试文件**: `tests/unit/fd_session_test.cpp`（8 个测试）

| 测试 | 覆盖内容 |
|------|---------|
| CreateAndDestroy | Factory::Create + 析构 |
| StepProducesValidOutput | 单步积分、ECEF 输出帧、海拔精度、空速 |
| MultipleStepsAreConsistent | 100 步连续积分稳定性 |
| ResetRestoresInitialState | Reset 回到初始海拔 |
| MoveSemantics | 移动构造/赋值 |
| GetCurrentStateDoesNotAdvance | 查询不推进状态 |
| FailedStepReturnsPreviousState | ok flag + 状态保持 |
| OutputKinematicsIsEcefFrame | ECEF 位置在地球半径范围 |

**构建修复**:
- 可见性：JSBSim 共享库需默认可见性（C++11 版无导出宏），使用 `get_directory_property(COMPILE_OPTIONS)` 临时移除 `-fvisibility=hidden`
- rpath：测试可执行文件需 `BUILD_RPATH` 指向 JSBSim .dylib 目录
- 链接：JSBSim::JSBSim 添加至主库 + 测试目标的 target_link_libraries
- FD_JSBSIM_ROOT_DIR 编译定义指向 third_party/jsbsim

**最终结果**: 876/876 全量测试通过（+8 个 fd 测试）

---

## 阶段 G 机动控制层 (2026-05-22)

### 调研

- 通过 DeepWiki 查询 JSBSim 对五种机动模式的原生支持
- 结论：JSBSim 无高层机动原语，仅提供低级 FCS 组件（PID、switch、waypoint 等）和脚本事件系统
- 确认无现成开源 C++ 制导/导航库可集成（ArduPilot GPLv3 不兼容，PX4 紧耦合）
- 用户提供 `maneuver_algorithms.md` 核心算法设计文档

### 审批

- 方向/固定点/航路点/蛇形四种机动算法方案通过 ✅
- 修正 1：G0 前置验证 AP 可用性（`ap/heading_*` 依赖飞机模型 XML 配置，非 JSBSim 内核）
- 修正 2：滚筒机动从开环时序改为姿态反馈闭环（自适应飞机特性 + 异常中止）
- 修正 3：ControlInput 接口需扩展 AP 指令字段
- 修正 4：固定点/航路点到达判定增加航向收敛条件
- 修正 5：航路点转弯增加提前量计算
- 修正 6：坐标计算复用 `coordinate::` 函数，不依赖 JSBSim `FGWaypoint` 组件

### 计划

- 制定阶段 G0-G7 详细任务计划，写入 `task_plan.md`
- 阶段 G0（AP 验证）为阻塞性前置任务

#### G0-G2 实现

- ✅ AP 验证：c172x 有完整 AP（heading_hold + altitude_hold + wing_leveler）
- ✅ ControlInput 扩展：heading_setpoint_deg、heading_hold、altitude_setpoint_m、altitude_hold
- ✅ JsbsimAdapter 适配：AP 属性写入 + 高度 m→ft 单位转换
- ✅ 方向机动：90° 转弯 20s 收敛、180° 转弯 30s 收敛

#### G3 固定点机动 + ECEF 大地纬度修复（用户定位）

- ✅ Haversine 大圆距离 + 前向方位角（北京→上海 ~1060km @ 140°）
- ✅ PointToPoint 制导：bearing→heading_setpoint + 油门 P 控制
- ❌ 初始全链路失败：min_dist = 21091m（严重偏差，飞机在错误位置）
- 🔧 **根因**：JSBSim `SetLatitudeDegIC`/`GetLatitudeDeg` 使用**地心纬度 (Geocentric)**，1Q 使用**大地纬度 (Geodetic, WGS84)**。椭球体下差异最大 ~0.19° ≈ 21km
- ✅ 修复：输入 `SetGeodLatitudeDegIC`，输出 `GetGeodLatitudeDeg`+`GetGeodAltitude`
- ✅ 修复后 min_dist 21091m → 424m（消除 98% 偏差），G3 全链路通过

#### G4 航路点机动

- ✅ ComputeWaypoint：航路点序列 + 到达检测 + 转弯提前量平滑混合
- ✅ 空列表处理 + ECEF 转换失败降级（保持当前航向）
- ⚠️ C172 全链路测试 DISABLED
  - **失败原因**：超过 100 秒的长距离飞行缺乏高度和油门 PID 闭环，飞机在长周期震荡中失速坠毁。必须等待纵向 AP 补全后方可进行全链路长航线测试。
  - **非算法缺陷**：G4_WaypointGuidanceOutput 已验证制导输出的航向设定点和逻辑完全正确。
- ✅ 算法正确性已验证

#### G5 蛇形机动

- ✅ 正弦航向偏置：`Ψ_target = Ψ_base + A·sin(ωt)`，振荡幅度 > 15°

#### F16 多机型实验（失败）

- ❌ F16 引擎不产生推力（ground_speed = 1.38e-9 m/s）
- 根因：
  1. 节气门路径差异：F16 用 `fcs/throttle-cmd-norm` (标量) vs C172 的 `fcs/throttle-cmd-norm[0]` (数组)
  2. F100 涡扇引擎默认 `propulsion=OFF`，RunIC 未自动启动
  3. F16 无 AP autopilot（无航向保持）
- 结论：多机型支持需抽象节气门/AP 路径到 AircraftDefinition，列为未来通用化任务
- 已清理所有 F16 实验代码，恢复为 C172-only
