# 代码库审查报告

> 审查日期：2026-03-26

---

## HIGH — 高优先级

### 1. `SignalComponentFactory.h` 全头文件实现 + `std::abort()` 错误处理

- **位置**：`src/airborne_radar/signal/pipeline/SignalComponentFactory.h`
- **问题**：
  - 拉入 10+ 个头文件（`SignalPipeline.h`、`DataAssociation.h`、`SignalDetector.h`、`BoostTrackPool.h`、`KalmanPredictor.h`、`KalmanUpdater.h`、`SynchronizedTrackPool.h`、`TrackFilter.h`、`TrackLifecycleManager.h`、`ProjectLog.h`），所有工厂逻辑内联实现，导致任何被依赖头文件的变更都触发全量重编译。
  - 第 213、225 行直接调用 `std::abort()`，在仿真库中过于激进——调用方无法恢复或报告错误。
- **建议**：将实现移入 `.cpp` 减少编译扇出；用错误返回（如返回 `nullptr` ）替代 `std::abort()`, 注意本项目使用C++11标准，禁止使用`std::optional`。

---

## MEDIUM — 中优先级

### 2. 三套近乎相同的干扰源结构体

- **位置**：
  - `JammerEmitterState` — `include/.../environment/EnvironmentTypes.h`
  - `JammerSourceFact` — 同文件
  - `EccmJammerSourceInfo` — `include/.../common/DecisionSourceInfo.h`
- **问题**：三个结构体拥有相同的 10 个字段（`technique`、`power_db`、`js_db`、`frequency_overlap_ratio`、`prf_lock_risk`、`azimuth_deg`、`elevation_deg`、`angular_span_deg`、`in_sidelobe`、`confidence`）。`ToJammerSourceFact()` 仅做逐字段复制，零变换。新增字段容易只改一处而遗漏其余。
- **建议**：统一为一个共享类型，必要时用别名或 tag 区分语义。

### 3. `RadarControlProfile` 泄漏 reducer 内部状态到公共类型

- **位置**：`include/.../common/RadarControlProfile.h:28-33`
- **问题**：`lpi_hold_cycles_remaining`、`eccm_hold_cycles_remaining`、`lpi_cooldown_cycles_remaining`、`eccm_cooldown_cycles_remaining` 四个字段标注"仅供 reducer 运行态使用"，却暴露在公共数据契约中。同样的字段还出现在 `ITacticalDecisionEngine.h:40-41`。任何消费者都能观察或意外修改这些内部状态。
- **建议**：抽取独立的 `ControlReducerState` 内部结构体，从公共类型中移除。

### 4. `active_legacy_jammer_emitter_index_` 只写不读

- **位置**：`src/.../environment/EnvironmentService.h:84`，`.cpp:154/163` 赋值
- **问题**：该成员变量被赋值但从未被任何逻辑消费，疑为早期设计遗留。
- **建议**：确认后删除。

### 5. `kNoLegacyJammerEmitterIndex` 重复定义

- **位置**：
  - `EnvironmentService.h:75`（类静态 `constexpr` 成员）
  - `EnvironmentService.cpp:20`（匿名命名空间 `constexpr`）
- **问题**：`.cpp` 中的匿名命名空间版本遮蔽类级常量，冗余且容易混淆。
- **建议**：删除 `.cpp` 中的冗余定义，统一使用类静态成员。

### 6. Event Bus 基础设施疑似死代码

- **位置**：`src/airborne_radar/core/event/` 下 `EventBus.h`、`CycleEventBus.h`、`RadarEvents.h`（定义 7 种事件类型）
- **问题**：`RadarController` 中无任何事件总线引用——既不发布也不订阅。基础设施存在但未接入核心运行路径。
- **建议**：若已集成则补齐控制器事件发布；若已废弃则移除整套 event 目录。本事件机制原目的是为了决策层发送事件后/外部输入事件在当前步调函数执行前动态调整雷达的运行时参数，审查当前是否还需要这套机制。

### 7. `ISignalPipeline.h` 直接 include `IEnvironmentService.h`

- **位置**：`include/.../signal/pipeline/ISignalPipeline.h:11-12`
- **问题**：`RunCycle()` 按 `const IEnvironmentService&` 引用接收参数，前向声明即可满足头文件需要。当前写法导致信号层公共接口的每个消费者传递拉入整个 environment 类型链（`EnvironmentTypes.h` → `DecisionSourceInfo.h`）。
- **建议**：头文件用前向声明，`.cpp` 中 include 完整头文件。

### 8. `signal_bulk_data_test.cpp` 永久禁用

- **位置**：`tests/CMakeLists.txt:43-46`（注释掉），第 114 行（仅在 stress preset 中启用）
- **问题**：IMM 多线程回归路径在常规 preset 下无自动覆盖。注释说"等待 IMM 多线程优化实现后恢复"，但若优化无限期推迟，此测试将持续缺失。
- **建议**：不处理，本质上未引入多线程，仅在 stress preset 中启用是有意为之。

### 9. 无 mock 隔离测试

- **位置**：`tests/` 全局
- **问题**：GMock 已链接（`tests/CMakeLists.txt:65`）但无任何 `MOCK_METHOD` 使用。`ISignalPipeline`、`IEnvironmentService`、`ITacticalDecisionEngine` 三大核心接口缺少 mock 变体。`core_controller_test.cpp` 实质是集成测试而非单元测试。
- **建议**：为核心接口补充 mock 类，实现真正的单元测试隔离。

---

## LOW — 低优先级

### 10. `RadarWorkSubMode` 放在 `RadarOrientationConfig.h` 中

- **位置**：`include/.../common/RadarOrientationConfig.h:70-75`
- **问题**：`RadarWorkSubMode`（`kStby`、`kTas`、`kTws`、`kStt`）是任务/操作语义枚举，不属于几何/方位配置头文件。导致 `RadarSessionConfigBuilder.h` 因使用此枚举而依赖 `RadarOrientationConfig.h`。
- **建议**：移至独立的 `RadarWorkMode.h` 文件。

### 11. `TargetFeature.h` 在公共头文件中 include `<cmath>`

- **位置**：`include/.../common/TargetFeature.h:9, 78-79`
- **问题**：仅为内联构造函数中的 `std::sqrt()` 调用引入 `<cmath>`，传播到所有直接或间接包含此头文件的翻译单元。`DecisionTrackSnapshot.h` 存在同样问题。
- **建议**：将构造函数移入 `.cpp`。

### 12. `DecisionFrameBuilders.h` 跨层 include `DataAssociation.h`

- **位置**：`src/.../signal/pipeline/DecisionFrameBuilders.h:11`
- **问题**：pipeline 层内部头文件直接引用 association 层内部头文件，产生层级倒置。association 内部类型变更会间接触发 pipeline 层重编译。
- **建议**：将 `AssociationQualityMetrics` 等共享类型提取到共享位置，或让 `DecisionFrameBuilders` 仅依赖公共类型，自行判断一下哪种方法更好。

### 13. `SignalPipeline` 直接持有 `DataOutputManager` 具体类型

- **位置**：`src/.../signal/pipeline/SignalPipeline.cpp:321`
- **问题**：`DataOutputManager output_manager_` 按值持有。已存在 `IDataOutputManager` 接口，`RadarController` 通过 `unique_ptr<IDataOutputManager>` 使用。Pipeline 绕过抽象直接使用具体类型，无法替换。
- **建议**：统一通过 `IDataOutputManager` 接口持有。

### 14. PCH 中包含 `<stdexcept>`、`<iostream>`

- **位置**：`src/CMakeLists.txt:198-204, 211`
- **问题**：项目明确禁用 C++ 异常，`<stdexcept>` 拉入异常处理基础设施；`<iostream>` 和 `<fstream>` 在生产代码中未使用。
- **建议**：从 PCH 列表中移除。

### 15. `BuildDecisionSummary()` 每周期分配 `vector<string>`

- **位置**：`src/.../decision/pipeline/TacticalCoordinator.cpp:60-95`
- **问题**：未检查日志级别就构建摘要字符串，在周期执行路径上产生不必要的堆分配。违反"不在高频仿真循环中日志"的精神。
- **建议**：包裹在日志级别检查中（如 `if (spdlog::should_log(...))`），仅在需要时构建字符串。

### 16. `decision_layer_test.cpp` 使用 `using namespace`

- **位置**：`tests/decision_layer_test.cpp:20`
- **问题**：测试套件中唯一一处 `using namespace airborne_radar`，与其他测试的限定名风格不一致。
- **建议**：改为限定名或 `using` 声明。

---

## 优先行动建议

| 优先级 | 动作 | 预期收益 |
|--------|------|----------|
| 1 | `SignalComponentFactory` 移实现到 `.cpp` + 去 abort | 编译时间 + 运行时安全 |
| 2 | 统一干扰源结构体 | 减少映射代码和字段漂移风险 |
| 3 | `RadarControlProfile` reducer 状态下沉 | 公共 API 清洁度 |
| 4 | 清理死代码（event bus / 只写字段 / 重复常量） | 降低认知负担 |
| 5 | `ISignalPipeline.h` 前向声明 | 编译解耦 |

---

# 第二轮深度审查

> 覆盖范围：ESR 模块、决策管线、通用/共享代码、核心层、信号内部算法、CMake 模块、横切关注点

---

## HIGH — 高优先级

### 17. `HasPositionMeasurement` 将原点 (0,0,0) 误判为"无量测"

- **位置**：`src/airborne_radar/signal/association/DataAssociation.cpp:395-397`
- **问题**：
  ```cpp
  return target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
  ```
  位于局部坐标原点的合法目标会被判定为"无位置量测"，触发第 407 行的 `AbortContractViolation`，导致整个仿真进程 abort。
- **建议**：为 `TargetFeature` 添加 `bool has_cartesian_position` 标志（与 `TrackMeasurement` 中已有的 `raw_measurement.has_cartesian_position` 对齐）。

### 18. 关联门控预测使用硬编码 `dt = 1.0`

- **位置**：`src/airborne_radar/signal/association/DataAssociation.cpp:379`
- **问题**：
  ```cpp
  kalman_predictor_.Predict(track.gaussian_state, 1.0f);
  ```
  门控先验计算使用 `dt = 1.0` 秒而非实际周期间隔。对 10 Hz 传感器（dt ≈ 0.1s），预测位置偏移量是实际值的 10 倍，导致关联门过大，可能错误关联。实际 `dt_sec` 在管线上下文中可用，但未传入此函数。
- **建议**：将实际 `dt_sec` 作为参数传入 `BuildExternalPositionAssociationPriors`。

### 19. `ComputeAzimuthDifferenceDeg` 对非有限输入存在无限循环风险

- **位置**：`src/common/geometry/GeometryTransform.cpp:77-86`
- **问题**：
  ```cpp
  while (diff > 180.0f)  { diff -= 360.0f; }
  while (diff <= -180.0f) { diff += 360.0f; }
  ```
  若上游泄漏 `Inf` 或 `NaN`，此循环将永不终止。该函数被机载雷达和 ESR 模块的多个热路径调用（`BeamwidthResolution`、`RadarOrientationUtils`、`InterceptPipeline`、`InterceptGate`）。
- **建议**：改用 `std::fmod` 实现，天然有界：
  ```cpp
  float diff = std::fmod(lhs_deg - rhs_deg + 540.0f, 360.0f) - 180.0f;
  ```

### 20. `ThreatAssessmentEvaluator::Evaluate` 空航迹列表时插入虚假 "UNKNOWN" 条目

- **位置**：`src/airborne_radar/decision/classifier/ThreatAssessmentEvaluator.cpp:57-62`
- **问题**：当 `input_frame.tracks` 为空时，仍向 `target_classification_result` 追加一个 "UNKNOWN" 条目。下游消费者预期 `tracks` 与 `classification_result` 1:1 对应，会产生 0 个航迹 vs 1 个分类的不匹配。
- **建议**：空航迹时直接返回，不追加任何条目。

---

## MEDIUM — 中优先级

### 21. `IsLegacyDefaultBeamState` 将合法波束参数误判为"未配置"

- **位置**：`src/electronic_surveillance_radar/pipeline/InterceptPipeline.cpp:90-95`
- **问题**：用 `az=0°, el=0°, beamwidth=20°` 的精确浮点比较判定波束是否为"历史默认值"。一个实际 20° 波束居中于视轴的合法发射源将被误判为"未配置"，直接返回 `overlap_ratio = 1.0`，绕过几何计算。
- **建议**：为 `EmitterBeamState` 添加 `bool beam_state_valid` 标志，而非通过域值本身推断有效性。

### 22. DBSCAN `ContainsIndex` 线性搜索导致 O(n³) 最坏复杂度

- **位置**：`src/electronic_surveillance_radar/pipeline/KdTreeClusterer.cpp:80-87, 140`
- **问题**：每次 DBSCAN 扩展邻居时调用 `ContainsIndex` 对 `deque` 做 O(n) 线性扫描。总复杂度 O(|N|·|Q|) / 种子点，在密集观测场景下退化为 O(n³)。
- **建议**：使用 `std::unordered_set<size_t>` 或直接利用 `labels` 数组检查 `labels[candidate] != -1` 实现 O(1) 查询。

### 23. IMM 滤波器 coast 周期不更新模型权重

- **位置**：`src/airborne_radar/signal/tracking/ImmFilter.cpp:36-46`（`Predict` 路径）
- **问题**：predict-only 路径调用 `MixStates()` → `PredictModels(dt)` → 替换 `model_states_` → `CombineEstimates()`，但**不更新模型权重**。标准 IMM 算法在 coast 周期仍应通过转移矩阵扩散模型概率。多步连续丢失时权重始终停留在最后一次更新值，偏离标准算法。
- **建议**：在 coast 路径中加入权重更新步骤（至少通过转移矩阵扩散）。

### 24. `Pd` 下限被强制钳位到 `pfa` 而非 0

- **位置**：`src/airborne_radar/common/timing/TimingRegimeModel.cpp:194`
- **问题**：
  ```cpp
  return Clamp01(std::max(static_cast<float>(pd), NormalizePfa(params.pfa)));
  ```
  对 SNR 远低于阈值的目标，Pd 理应趋近 0，但被钳位到 `pfa`。这导致极弱目标仍产生非零检测概率，在 ESR 管线中会虚增低 SNR 发射源的检测计数。
- **建议**：移除 `pfa` 下限，或仅在 SNR 接近阈值时应用。

### 25. 决策层双重 ECCM 保持计数器无协调

- **位置**：
  - `SurvivabilityEvaluator.cpp:19` — `kEccmHoldCycles = 2`
  - `ControlReducer` — `lpi_hold_cycles_after_request`（配置值）
- **问题**：`TacticalStateStore::eccm_hold_cycles_remaining`（evaluator 管理）和 `RadarControlProfile::eccm_hold_cycles_remaining`（reducer 管理）是两个独立计数器，各自递减。有效 ECCM 持续时间是两者的最大值，但修改任一配置值不会影响另一个。
- **建议**：合并为单一权威计数器，或建立明确的主从关系。

### 26. 评估器通过共享可变 `evaluation_state` 通信

- **位置**：`src/airborne_radar/decision/pipeline/TacticalCoordinator.cpp:101-109, 126-127`
- **问题**：`BackfillAssociationDrivenEccmTrigger` 在评估器执行前修改 `evaluation_state.eccm_source_info`。三个评估器（threat → LPI → survivability）通过读写同一个 `TacticalEvaluationState` 对象隐式通信。当前执行顺序固定所以安全，但若调整顺序或并行化将产生数据竞争。
- **建议**：评估器之间通过返回值或显式输出结构通信，而非共享可变状态。

### 27. ESR 模块 `Clamp01`、`ResolveTechnique` 等函数重复 3-4 次

- **位置**：
  - `Clamp01`：`InterceptPipeline.cpp:64`、`HypothesisAssociator.cpp:24`、`EsrEnvironmentService.cpp:16`、`JammingAggregator.h:158`
  - `ResolveTechnique`/`HasSuppressionEffect`/`HasDeceptionEffect`：`EsrEnvironmentService.cpp:30-57` vs `JammingAggregator.h:122-151` 逐字复制
- **建议**：提取到 ESR 共享内部头文件。

### 28. `TrackLifecycleManager::ResolveEffectiveCycleDeltaTimeSec` dt 回退用周期索引差代替时间

- **位置**：`src/airborne_radar/signal/tracking/TrackLifecycleManager.cpp:505-508`
- **问题**：
  ```cpp
  return static_cast<float>(cycle_index - last_cycle_index_);
  ```
  回退值将周期索引差当作秒数。在 10 Hz 系统中，索引差 3 返回 3.0 秒（实际应为 0.3 秒），导致速度外推完全错误。
- **建议**：回退路径应使用配置的周期间隔乘以索引差。

### 29. `coherent_integration` 参数被 `SignalDetector::Detect` 静默丢弃

- **位置**：`src/airborne_radar/signal/detection/SignalDetector.cpp:25-26`
- **问题**：公共 API 接受 `coherent_integration` 标志但始终 `(void)` 忽略。`RadarEquations::ComputeIntegrationGain` 实际区分了相参/非相参积累，但 `Detect` 从未将该标志传递下去。
- **建议**：在检测计算中实际使用。

### 30. Windows 下 spdlog 完全禁用，静默无日志

- **位置**：`src/CMakeLists.txt:93-97`
- **问题**：`PROJECT_ENABLE_SPDLOG` 在 Windows 上为 `OFF`，所有 `PROJECT_LOG_*` 编译为空操作。Windows 开发者调试时无诊断输出。
- **建议**：暂不处理。

---

## LOW — 低优先级

### 31. `RadarSession::StepWithResult` 无线程安全文档

- **位置**：`src/airborne_radar/core/session/RadarSession.cpp:62-66`
- **问题**：`UpdateSceneState` 写入状态后 `StepWithResult` 读取冻结。若双线程同时操作会产生不一致。缺少单线程假设的文档说明。
- **建议**： 暂不处理。本库为单线程。

### 32. `RadarController::Impl` 裸指针 `control_profile` 可空但无防护

- **位置**：`RadarController.cpp:127-131`
- **问题**：`control_profile` 是裸指针，与 `owned_control_profile` 配合的"可能拥有"模式存在生命周期风险。
- **建议**：考虑使用 `std::reference_wrapper` 使其不可空。

### 33. `ImmFilter::MixStates` 与 `UpdateModels` 重复计算 `c_bar`

- **位置**：`src/airborne_radar/signal/tracking/ImmFilter.cpp:106-113`
- **问题**：`MixStates()` 中用局部变量 `float c_bar`，`UpdateModels()` 中设置成员 `c_bar_`，计算逻辑相同但执行两次。

### 34. `HypothesisAssociator::confirmed` 长期丢失后不撤销

- **位置**：`src/electronic_surveillance_radar/pipeline/HypothesisAssociator.cpp:219-228`
- **问题**：`hit_streak = 0` 但 `confirmed = true` 永不重置。已确认的假设在失去聚类关联多个周期后仍以"已确认"状态输出直到被剪枝。

### 35. CMake `ProjectOptions.cmake` 残留调试 `message(STATUS "DEBUG: ...")`

- **位置**：`cmake/ProjectOptions.cmake:101-102`
- **问题**：`message(STATUS "DEBUG: PACKAGE_MANAGER = ...")` 污染每次配置输出。`${CMAKE_MATCH_0}` 在该上下文无意义。

### 36. `InterceptPipeline` 构造函数复制而非移动 config 参数

- **位置**：`src/electronic_surveillance_radar/pipeline/InterceptPipeline.cpp:506-511`
- **问题**：`config` 和 `runtime_config` 按值传入后赋值（复制），应使用 `std::move`。config 中含 `std::vector`，移动可避免堆拷贝。`HypothesisAssociator` 构造函数同理。

### 37. `BoostTrackPool::Release` 的 `in_use_count_` 计数可静默失步

- **位置**：`src/airborne_radar/signal/tracking/BoostTrackPool.cpp:39-41`
- **问题**：对非本池指针或重复释放仅做 `count > 0` 防护，不报告异常。

### 38. `BuildRotationMatrix` pitch 取反约定无测试覆盖

- **位置**：`src/common/geometry/GeometryTransform.cpp:107`
- **问题**：`DegToRad(-euler_deg.pitch_deg)` 的符号翻转是 load-bearing 行为，但无直接验证三轴符号约定的测试。

### 39. `EmissionControlEvaluator::Evaluate` 忽略其 `input_frame` 参数

- **位置**：`src/airborne_radar/decision/emission/EmissionControlEvaluator.cpp:32`
- **问题**：`(void)input_frame;` 直接丢弃。该评估器仅读取共享 `evaluation_state` 中由上游设置的标志，作为独立类存在的价值有限。

### 40. CMake `USE_CCACHE` Linux/macOS 分支完全相同

- **位置**：`cmake/ProjectOptions.cmake:39-42`
- **问题**：`if(UNIX AND NOT APPLE)` 和 `elseif(APPLE)` 设置相同选项和默认值，可合并为 `if(UNIX)`。

---

## 第二轮优先行动建议

| 优先级 | 动作 | 预期收益 |
|--------|------|----------|
| 1 | 修复 `HasPositionMeasurement` 原点误判 | 消除仿真 abort 风险 |
| 2 | 关联门控传入实际 `dt_sec` | 关联精度 |
| 3 | `ComputeAzimuthDifferenceDeg` 改用 `fmod` | 消除无限循环 |
| 4 | 修复空航迹虚假分类条目 | 下游数据一致性 |
| 5 | `IsLegacyDefaultBeamState` 改用显式标志 | ESR 建模正确性 |
| 6 | DBSCAN 查重改用 `unordered_set` | 性能（密集场景） |
| 7 | IMM coast 权重更新 | 跟踪精度 |

---

# 第三轮深度审查

> 覆盖范围：Kalman 滤波实现、数据关联算法、控制 Reducer 逻辑、输出组装、运行时/时序基础设施、ESR 截获域、特征库、公共 API Builder、示例代码、测试正确性

---

## HIGH — 高优先级

### 41. `ScanPatternGenerator::NormalizeAzimuthDeg` 无界 `while` 循环（第三处）

- **位置**：`src/electronic_surveillance_radar/intercept/ScanPatternGenerator.h:159-168`
- **问题**：与 #19 `ComputeAzimuthDifferenceDeg` 同类 bug。对极大浮点输入（如 `1e20f`），`1e20f - 360.0f == 1e20f`（精度丢失），导致死循环。
- **建议**：统一替换为 `std::fmod` 实现。代码库中至少存在 **3 处** 相同的无界角度归一化模式，应提取共享函数。

### 42. `FeatureRepository::FetchRawRowsFromDataSource` 永远返回 `false`

- **位置**：`src/airborne_radar/environment/database/FeatureRepository.cpp:108-117`
- **问题**：
  ```cpp
  if (connection_string_.empty()) { return false; }
  return false;  // 无条件 false
  ```
  `ConnectDataSource("...")` 返回 `true`（非空字符串即成功），但 `ReloadFromDataSource()` 始终失败。公共 API 契约断裂：连接"成功"后加载永远静默失败，调用方无法区分"未连接"与"已连接但未实现"。
- **建议**：`ConnectDataSource` 在未实现时也应返回 `false` 并记录日志，或文档明确标注 stub 状态。

### 43. `BoundarySearchSolver::Solve` 谓词全假时返回 `converged=true`

- **位置**：`src/electronic_surveillance_radar/intercept/BoundarySearchSolver.h:51-57`
- **问题**：
  ```cpp
  if (!low_ok) {
    result.boundary_range_m = min_range_m;
    result.converged = true;
    return result;
  }
  ```
  当谓词在 `min_range_m` 处为 `false`（目标在最小距离都无法截获），返回 `converged=true, boundary=min_range`。调用方若仅检查 `converged` 会误认为截获可行。
- **建议**：谓词全假时应返回 `converged=false` 或添加 `bool predicate_satisfied` 字段。

### 44. IMM 滤波器 `GaussianLikelihood` float 精度 log 行列式可能下溢为 `-inf`

- **位置**：`src/airborne_radar/signal/tracking/ImmFilter.cpp:149-166`
- **问题**：
  ```cpp
  const float log_det = 2.0f * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();
  ```
  对极小新息协方差（量测非常接近预测、不确定性极低），LLT 对角元素趋近 0，`log(0) = -inf`。后续 `exp(log_likelihood)` 产生 `+inf`，权重归一化变成 `NaN`，所有模型权重污染。
- **建议**：将 LLT 和 log-det 计算提升到 `double` 精度，或对对角元素加 `max(epsilon, L_ii)` 防护。

---

## MEDIUM — 中优先级

### 45. `Hypothesiser` 共享可变 metric 指针在 4 参数和 2 参数 `Generate` 调用间状态泄漏

- **位置**：`src/airborne_radar/signal/association/Hypothesiser.cpp:115-130`
- **问题**：4 参数 `Generate` 调用 `covariance_metric_->SetInnovationCovariance(...)` 修改共享 metric 对象的内部 LLT 分解。后续 2 参数 `Generate` 调用使用同一 metric 指针时，会继承上次遗留的 LLT 状态。若调用者混用两种重载，关联距离计算将使用错误的协方差。
- **建议**：每次 `Generate` 调用开始时重置 metric 状态，或为 4 参数重载使用独立 metric 实例。

### 46. `PropagationModel` 静默钳位负的 `terrain_reflection_db`

- **位置**：`src/airborne_radar/environment/simulation/PropagationModel.cpp:11-14`
- **问题**：
  ```cpp
  result.propagation_loss_db = std::max(0.0f, base + atmospheric + terrain_reflection);
  ```
  `terrain_reflection_db` 可为负值（多径增益）。`std::max(0.0f, ...)` 阻止传播总量为增益。用户设置 `SetTerrainReflectionDb(-3.0f)` 表示 3 dB 多径增益会被静默取消，且无文档说明此约束。
- **建议**：移除下限钳位，或在 builder 中验证并文档化约束。

### 47. `FeatureRepository::ComputeDistance` 扩展特征距离不对称——惩罚更丰富的原型记录

- **位置**：`src/airborne_radar/environment/database/FeatureRepository.cpp:176-183`
- **问题**：对输入中缺失的扩展特征隐式使用 0 作为默认值。原型记录中该特征值越大，距离惩罚越大。这偏向于特征集较小的记录，抑制向原型添加新维度。
- **建议**：仅在输入和原型都包含该特征时才计算距离，或使用"缺失特征不参与距离计算"策略。

### 48. `RadarSessionConfigBuilder` 无物理参数验证

- **位置**：`include/.../common/RadarSessionConfigBuilder.h:42`
- **问题**：`WithTransmitterPeakPowerW(-1.0f)` 等非物理值静默传入仿真。对外部平台集成者，错误配置可能仅表现为异常的检测结果（负 SNR、负 Pd），无任何错误信号。
- **建议**：`Build()` 中至少验证关键物理约束（功率 > 0、带宽 > 0、噪声系数 >= 0 等）。

### 49. 示例代码只更新 x 分量位置，忽略 y/z

- **位置**：`examples/example_quick_start.cpp:128-130, 154-156, 171-172, 196-197`
- **问题**：
  ```cpp
  t.position_x += t.current_track_velocity_x * input.dt_sec;
  // position_y 和 position_z 未更新
  ```
  P2 目标有 `velocity_y = 5.0f`，但 y 位置从未更新。用户复制此模式会在集成测试中得到 y/z 静止的目标。
- **建议**：更新全部三个分量。

### 50. `KalmanUpdater` / `EkfUpdater` Kalman 增益通过 `S.llt().solve(I)` 计算显式逆

- **位置**：`src/airborne_radar/signal/tracking/KalmanUpdater.cpp:38-40`，`EkfFilter.cpp:54-56`
- **问题**：
  ```cpp
  const KalmanGainMatrix K = P * Hᵀ * S.llt().solve(Identity());
  ```
  `S.llt().solve(I)` 等价于计算 `S⁻¹`，违背 Cholesky 分解的数值稳定性优势。正确惯用法是 `K = (S.llt().solve(H * P)).transpose()`。
- **建议**：对 3×3 矩阵实际影响有限，但作为数值算法库应采用最佳实践。

### 51. `rejected_cost = unassigned_cost + 1.0f` 在 `unassigned_cost` 接近 `float_max` 时溢出

- **位置**：`src/airborne_radar/signal/association/DataAssociation.cpp:227`
- **问题**：若调用方设置 `unassigned_cost = std::numeric_limits<float>::max()`，则 `rejected_cost = float_max + 1.0f = float_max`，`rejected_cost == unassigned_cost` 为 `true`，拒绝逻辑（`matched_cost <= unassigned_cost`）失效，所有未门控对也会被接受。
- **建议**：验证 `unassigned_cost` 上界，或用 `rejected_cost = nextafterf(unassigned_cost, INFINITY)` 替代。

---

## LOW — 低优先级

### 52. `OutputAssemblySupport` auto-lifecycle 路径跳过 `output_manager->BuildTrackOutputFrame`

- **位置**：`src/airborne_radar/signal/pipeline/OutputAssemblySupport.cpp:56-70`
- **问题**：生产路径（auto lifecycle）提前返回，`DataOutputManager::BuildTrackOutputFrame` 成为死代码。

### 53. `TargetFeatureBuilder::Build()` 非 `const` 且修改内部状态

- **位置**：`include/.../common/TargetFeatureBuilder.h:88-91`
- **问题**：`Build()` 调用 `NormalizeTargetGeometry(&target_)` 修改内部状态。连续调用两次 `Build()` 时，第二次对已归一化的数据再次归一化。虽然该操作幂等，但 builder 模式通常期望 `Build()` 可安全重复调用。

### 54. `EnvironmentSceneBuilder` typed-Add 方法静默覆盖 `technique`

- **位置**：`src/airborne_radar/environment/EnvironmentSceneBuilder.cpp:32-51`
- **问题**：`AddNoiseJammer(emitter)` 无条件将 `emitter.technique` 覆盖为 `kNoiseSuppression`。示例代码中先手动设置 technique 再调用 typed-Add 方法，使覆盖变成冗余操作。传错方法则静默篡改。

### 55. `kalman_filter_test.cpp` 收敛断言过宽

- **位置**：`tests/kalman_filter_test.cpp:418`
- **问题**：初始位置方差 1.0，5 次 predict-update 后断言 `< 100.0`。阈值是初始值的 100 倍，即使滤波器严重发散也能通过。
- **建议**：收紧到稳态 Riccati 解的合理倍数（如 2-3 倍）。

### 56. `advanced_filter_test.cpp` 单模型 IMM 测试缺少协方差等式断言

- **位置**：`tests/advanced_filter_test.cpp:162-206`
- **问题**：仅比较 `mean`，未比较 `covariance`。单模型 IMM 的协方差应与纯 KF 完全一致，`CombineEstimates` 中的外积项 bug 将逃逸。

### 57. `decision_layer_test.cpp` 分类测试未断言 `probability` 字段

- **位置**：`tests/decision_layer_test.cpp:137-151`
- **问题**：仅断言 `target_type == "HIGH_THREAT_FIGHTER"`，未检查 `probability`。归一化逻辑 bug（如 `probability = NaN`）将逃逸。

### 58. `track_lifecycle_test.cpp` `CountingTrackPool::Acquire` 不清零返回内存

- **位置**：`tests/track_lifecycle_test.cpp:23-71`
- **问题**：Release + re-Acquire 后返回的 `TrackState*` 携带前一轮生命周期的残留状态。测试可能因残留值恰好合法而偶然通过。

### 59. 示例代码 mid-session 配置变更注释误导

- **位置**：`examples/example_quick_start.cpp:166-186`
- **问题**：注释称"需要连续命中更多周期才能重新确认"，但已确认航迹不会被降级——仅新航迹受新阈值影响。注释描述的行为与实际不符。

### 60. `FeatureRepository` `score_sum <= 0` 守卫为不可达死代码

- **位置**：`src/airborne_radar/environment/database/FeatureRepository.cpp:82-105`
- **问题**：`exp(-distance) > 0` 对所有有限距离成立，`score = exp(-d) * prior` 在 `prior > 0` 时始终为正，`score_sum > 0` 恒成立。`return false` 分支永不执行。

---

## 第三轮优先行动建议

| 优先级 | 动作 | 预期收益 |
|--------|------|----------|
| 1 | 统一提取角度归一化函数（消除 3 处无界 while） | 消除 3 处死循环风险 |
| 2 | `BoundarySearchSolver` 谓词全假返回 `converged=false` | ESR 截获边界正确性 |
| 3 | IMM `GaussianLikelihood` log-det 改用 double 精度 | 跟踪稳定性 |
| 4 | `FeatureRepository` stub API 契约修正 | 外部集成可靠性 |
| 5 | `Hypothesiser` metric 状态隔离 | 关联正确性 |
| 6 | 示例代码 3D 位置更新 | 用户引导正确性 |
