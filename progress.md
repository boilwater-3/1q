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
