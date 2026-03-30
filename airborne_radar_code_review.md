# 机载雷达模块代码审查报告

> 审查日期：2026-03-30 | 审查范围：`src/airborne_radar/` 全模块源码

---

## 1. 前次审查问题跟进

| 问题 | 状态 | 详情 |
|:-----|:----:|:-----|
| `throw` 违反无异常约定 | ✅ **已修复** | 全部替换为 `AbortContractViolation` + `std::abort` |
| `const_cast` UB 隐患 | ✅ **已修复** | `Hypothesiser` 拆分为双构造函数，内部分离 `distance_metric_` / `covariance_metric_` |
| `SignalPipeline.cpp` 上帝文件 | ✅ **已修复** | 已拆分为 11 个独立文件（~200-400 行/文件） |
| 硬编码魔法数字 | ✅ **已修复** | 已提取为 `constexpr` 常量并配置化到 `JammingEffectsConfig` |
| `target_sources` 双重添加 | ✅ **已修复** | CMake 重构为 OBJECT 库模式 |
| PCH 包含 C++17 头 | ✅ **已修复** | 已加 `if(CMAKE_CXX_STANDARD GREATER_EQUAL 17)` 条件守护 |

所有高/中优先级前序问题已修复。

---

## 2. 本次发现的问题

### 2.1 🔴 高优先级

#### 2.1.1 `ResolveDominantJammingSemantic` 中 `second_index` 初始化逻辑缺陷

[JammingEffects.cpp:292-301](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/JammingEffects.cpp#L292-L301)

```cpp
std::size_t best_index = 0U;
std::size_t second_index = 0U;   // ← 初始化与 best_index 相同
for (std::size_t i = 1; i < 3U; ++i) {
  if (type_scores[i] > type_scores[best_index]) {
    second_index = best_index;
    best_index = i;
  } else if (i != best_index && type_scores[i] > type_scores[second_index]) {
    second_index = i;
  }
}
```

**问题**：当 `best_index` 保持为 `0` 时（即 `type_scores[0]` 始终最大），`second_index` 也初始化为 `0`。此时后续逻辑 `second_index != best_index` 为 `false`，永远不会判定为 `kMixed`，即使 `type_scores[1]` 和 `type_scores[2]` 中存在足够接近的分数。

**影响**：在噪声压制主导的场景下，即使欺骗/转发贡献接近噪声压制，也不会被判为 Mixed 语义，导致下游决策层遗漏混合干扰态势。

**建议**：

```cpp
std::size_t second_index = (best_index == 0U) ? 1U : 0U;
```

或将初始遍历改为标准 top-2 选择逻辑。

---

### 2.2 🟡 中优先级

#### 2.2.1 `ControlProfileEffects.cpp` 频率跳变方向由 `version % 2` 决定

[ControlProfileEffects.cpp:217](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/ControlProfileEffects.cpp#L217)

```cpp
const float hop_factor = (control_profile.version % 2U == 0U) ? 1.015f : 0.985f;
runtime_config->detection.radar_system.transmitter.frequency_hz *= hop_factor;
```

**问题**：`version` 是控制剖面的版本号（单调递增），用其奇偶性决定频率跳变方向意味着：
1. 每次 `version` 改变都会翻转方向，即使变更与频率捷变无关（如仅改了 LPI 配置）
2. 缺乏物理含义——真实频率捷变应基于随机或编码跳频序列

**建议**：引入独立的 `frequency_hop_state` 字段或 PRNG seed，替代 `version % 2` 的耦合。

#### 2.2.2 `DataAssociationEngine::HasPositionMeasurement` 的 `or` 判定过于宽松

[DataAssociation.cpp:398-401](file:///Users/aurora/Code/1q/src/airborne_radar/signal/association/DataAssociation.cpp#L398-L401)

```cpp
bool DataAssociationEngine::HasPositionMeasurement(const common::model::TargetFeature& target) const {
  return target.has_cartesian_position ||
         target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
}
```

**问题**：当 `has_cartesian_position == false` 但恰好有个别位置分量非零（例如噪声、默认值残留）时，该函数仍返回 `true`，可能导致关联阶段误用无效位置数据。

**建议**：优先依赖语义标志 `has_cartesian_position`，仅将坐标非零作为辅助校验：

```cpp
return target.has_cartesian_position;
```

或至少用 `&&` 代替 `||`：

```cpp
return target.has_cartesian_position &&
       (target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f);
```

#### 2.2.3 `TrackLifecycleManager::ResetForReuse` 未重置 `track_id` 和 `generation`

[TrackLifecycleManager.cpp:665-681](file:///Users/aurora/Code/1q/src/airborne_radar/signal/tracking/TrackLifecycleManager.cpp#L665-L681)

```cpp
void TrackLifecycleManager::ResetForReuse(TrackState& track) const {
  track.batch_id = 0;
  track.external_target_id = 0;
  // ... 其他字段重置
  // 注意：track_id 和 generation 未在此处重置
}
```

**问题**：`ResetForReuse` 旨在"将对象重置为可复用状态"，但 `track_id` 保留了上一次的值，而 `generation` 在 `PromoteState` 中递增但在 `ResetForReuse` 中不会归零。虽然 `EnsurePhase` 中会覆写 `track_id`，但 `generation` 可能导致后续复用的轨迹携带不正确的代数信息。

**建议**：明确文档化意图——如果 `generation` 有意保留累计值用于追踪复用次数则 OK，否则应重置为 0。

#### 2.2.4 `SimpleTrackUpdater::Update` 中 `relief_scale` 计算疑似多余

[TrackFilter.cpp:97](file:///Users/aurora/Code/1q/src/airborne_radar/signal/tracking/TrackFilter.cpp#L97)

```cpp
const float relief_scale = std::min(0.10f, 0.10f * std::max(0.0f, context.jamming_severity));
```

**问题**：`jamming_severity` 已被 `ClampFloat(..., 0, 1)` 约束在 `[0, 1]`。因此 `0.10f * max(0, severity)` 的取值范围是 `[0, 0.10]`，与 `min(0.10f, ...)` 的效果完全等价——即 `relief_scale` 始终等于 `0.10f * severity`，`min` 冗余。这不是 bug，但暗示原始意图可能是使用更大的乘数并用 `min` 限制上界。

**建议**：验证设计意图，如果 `0.10` 确实是上界，简化为 `0.10f * severity`。

#### 2.2.5 `BoostTrackPool` 缺乏对 `pool_.construct()` 返回 `nullptr` 的防护日志

[BoostTrackPool.cpp:27](file:///Users/aurora/Code/1q/src/airborne_radar/signal/tracking/BoostTrackPool.cpp#L27)

```cpp
track = pool_.construct();
// 无日志，直接返回 nullptr
```

`prewarm` 阶段（L12-17）有 `nullptr` 检查但不输出日志，`Acquire` 阶段同样静默。在高负载场景下 `pool_` 分配失败应输出 `PROJECT_LOG_ERROR`。

#### 2.2.6 `SurvivabilityEvaluator` 匿名常量使用 `const` 而非 `constexpr`

[SurvivabilityEvaluator.cpp:19-116](file:///Users/aurora/Code/1q/src/airborne_radar/decision/evaluators/SurvivabilityEvaluator.cpp#L19-L116)

所有匿名命名空间常量用 `const float` 而非 `constexpr float` 声明。与 `JammingEffects.cpp` / `ControlProfileEffects.cpp` 中的 `constexpr` 风格不一致，后者可在编译期求值并消除运行时初始化开销。

---

### 2.3 🟢 低优先级

#### 2.3.1 `ImmFilter::GaussianLikelihood` 可能的数值下溢

[ImmFilter.cpp:187](file:///Users/aurora/Code/1q/src/airborne_radar/signal/tracking/ImmFilter.cpp#L187)

```cpp
return static_cast<float>(std::exp(log_likelihood));
```

当 `log_likelihood` 非常负时（大 Mahalanobis 距离），`exp` 会精确为 0。虽然后续 `UpdateModels` 中有 `total < 1e-30f` 的兜底，但若**所有模型**的似然均为 0（极端异常量测），则所有权重会被强制为相等——这种降级行为应被记录在文档中。

#### 2.3.2 `RadarEquations::MarcumQ` 中 `a` 参数被修改

[RadarEquations.cpp:175-177](file:///Users/aurora/Code/1q/src/airborne_radar/signal/detection/RadarEquations.cpp#L175-L177)

```cpp
if (a < 0.0) {
  a = 0.0;
}
```

参数 `a` 是按值传入的 `double` 并在函数内被修改。虽然语义正确（负值无物理含义），但建议在函数签名中使用 `const` 参数并在内部创建局部变量，或将 clamp 放到调用方，以保持函数接口的不可变语义。

#### 2.3.3 Windows 日志仍完全禁用

```cmake
if(WIN32)
    set(PROJECT_ENABLE_SPDLOG OFF)
```

前次审查已指出，本次确认仍未修复。建议至少提供 `fprintf(stderr, ...)` 的 fallback backend。

#### 2.3.4 `TrackLifecycleManager.cpp` 行数仍偏大

尽管 `SignalPipeline.cpp` 已拆分，`TrackLifecycleManager.cpp` 仍有 **686 行**，集中了 5 个 Phase 方法、IMM 管理、Kalman 更新、状态推进和快照导出逻辑。建议将：
- Phase 1-5 提取为独立的 `TrackLifecyclePhases.cpp`
- 快照构建（`BuildFeatureSnapshot`、`BuildDecisionSnapshot`）提取到 `TrackSnapshotBuilder.cpp`

#### 2.3.5 `MathUtils` 重复定义

`EnvironmentService.cpp`（L19-25）自行定义了 `Clamp01` / `ClampNonNegative`，而 `MathUtils.h` 中已有 `ClampFloat`。建议统一使用公共工具。

#### 2.3.6 `SurvivabilityEvaluator.cpp` 匿名命名空间函数过多

该文件中匿名命名空间有 **9 个函数 + 5 个结构/枚举**，总计 438 行中超过 400 行在匿名命名空间内。建议提取到独立的辅助文件 `EccmProposalResolver.cpp`。

---

## 3. 整体质量评估

| 维度 | 评分 | 变化 | 说明 |
|:-----|:----:|:----:|:-----|
| **架构设计** | ★★★★☆ | → | 分层清晰，Pipeline 拆分显著改善 |
| **代码质量** | ★★★★☆ | ↑ | `throw`/`const_cast` 已消除，风格更统一 |
| **正确性** | ★★★½☆ | — | `second_index` 初始化 bug 需修复 |
| **测试覆盖** | ★★★★☆ | → | 未在本次审查范围 |
| **可维护性** | ★★★★☆ | ↑ | 拆分后单文件职责清晰，常量命名佳 |

---

## 4. 推荐行动优先级

```mermaid
graph LR
    A["🔴 修复 second_index 初始化"] --> B["🟡 解耦频率跳变与 version"]
    B --> C["🟡 修正 HasPositionMeasurement"]
    C --> D["🟡 确认 ResetForReuse 语义"]
    D --> E["🟢 统一 const→constexpr"]
    E --> F["🟢 拆分 TrackLifecycleManager"]
    F --> G["🟢 Windows 日志 fallback"]
```

---

> **结论**：相比前次审查，代码库在异常策略一致性、文件组织、常量管理方面有显著进步。发现的最严重问题是 `ResolveDominantJammingSemantic` 中的 top-2 选择逻辑缺陷，可能在特定干扰场景下导致混合干扰误判。其余问题为中/低优先级的代码卫生改进。
