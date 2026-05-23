# 任务规划 — 统一大气模型模块

## 目标

创建统一大气模型模块，基于 ISA 1976 自实现提供精确的大气状态查询，统一各传感器模块的大气消费接口，消除重复的 14 字段手工填充代码。

## 当前阶段

全部完成 ✅

## 各阶段

### 阶段 1：Foundation — 统一大气状态类型 ✅

- [x] 1a. 创建 `include/1q/foundation/atmosphere_state.h` — `AtmosphericState` 结构体（SI 单位）
- [x] 1b. 创建 `include/1q/foundation/atmosphere_provider.h` — `IAtmosphereProvider` 抽象接口
- **状态：** complete

### 阶段 2：ISA 1976 标准大气实现 ✅

- [x] 2a. 创建 `src/common/atmosphere/StandardAtmosphere.h` — ISA 1976 实现头文件
- [x] 2b. 创建 `src/common/atmosphere/StandardAtmosphere.cpp` — ISA 1976 实现
- [x] 2c. 修改 `AtmospherePhysics.h` — 新增 `BuildPropagationInputs()` 便利函数声明 + `AtmosphericObservationRef`
- [x] 2d. 修改 `AtmospherePhysics.cpp` — GTD7→ISA 委托 + `BuildPropagationInputs()` 实现
- **状态：** complete

### 阶段 3：构建系统集成 ✅

- [x] 3a. 修改 `src/common/CMakeLists.txt` — 新增 `StandardAtmosphere.cpp` 到源文件列表
- [x] 3b. 修改 `src/common/CMakeLists.txt` — 新增公共头文件安装规则
- [x] 3c. 修改 `check_public_api_boundary.cmake` — 白名单添加新头文件
- **状态：** complete

### 阶段 4：Airborne Radar 消费层重构 ✅

- [x] 4a. 修改 `PropagationModel.cpp` — 使用 `BuildPropagationInputs()`
- [x] 4b. 修改 `DetectionExecution.cpp` — 使用 `BuildPropagationInputs()`
- **状态：** complete

### 阶段 5：ESR 消费层重构 ✅

- [x] 5a. 修改 `EsrEnvironmentService.cpp` — 使用 `BuildPropagationInputs()`
- **状态：** complete

### 阶段 6：测试与验证 ✅

- [x] 6a. 创建 `tests/unit/common_standard_atmosphere_test.cpp` — 14 个 ISA 精确值测试
- [x] 6b. 修改 `tests/unit/ar_atmosphere_physics_test.cpp` — 新增 2 个 `BuildPropagationInputs()` 测试
- [x] 6c. 编译验证 — debug preset 通过
- [x] 6d. 全量测试回归 — 12/12 通过（741/741 单元测试）
- **状态：** complete

## 已做决策

| 决策 | 理由 |
|------|------|
| 自实现 ISA 1976（方案 B） | ISA 是开放标准，数学明确；避免 JSBSim 内部耦合；直接 SI 单位 |
| 不修改公共输入类型 | 保持 ABI 兼容，FlatBuffers schema 不变 |
| 仅覆盖 0-86 km | 传感器工作高度 0-30 km，86 km 以上留 stub |
| 默认包含声速 | 计算量极小，EOS 可能间接需要 |
| 首版仅标准 ISA，无偏差注入 | 偏差注入作为 Phase 2 扩展 |
| GTD7 委托 ISA | 内部替换实现，保持 ABI 兼容 |
| BuildPropagationInputs 使用 AtmosphericObservationRef | 消除消费者 14 字段手工填充，保持灵活性 |
| 测试容差考虑几何→位势转换 | ISA 层边界定义在位势高度，API 接受几何高度 |

## 遇到的错误

| 错误 | 尝试次数 | 解决方案 |
|------|---------|---------|
| StandardAtmosphere 命名空间编译错误 | 1 | 使用 `foundation::AtmosphericState` 完整限定名 |
| public_api_boundary_guard 失败 | 1 | 白名单添加 atmosphere_state.h + atmosphere_provider.h |
| ISA 测试温度精度不匹配 | 1 | 修正期望值：位势高度≠几何高度，容差需反映转换偏差 |
