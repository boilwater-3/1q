# EOS 对外 API 与配置反直觉设计审查

> 审查日期：2026-07-31
> 审查范围：`include/1q/electro_optical_sensor/` 全部公开头文件、`src/` 中配置映射与校验实现
> 审查目标：识别四域配置中冗余、静默失败、语义误导等反用户直觉的设计

---

## 审查结论

共发现 **12 个反用户直觉的设计问题**：高严重度 4 个、中严重度 4 个、低严重度 4 个。

---

## 🔴 高严重度

### 1. `CreateWithValidation` 校验失败仍返回 Session

**文件**：`include/1q/electro_optical_sensor/session/EosSession.h:63-64`

```cpp
static EosSession CreateWithValidation(const config::EosSessionConfig& config,
                                       config::ValidationIssueList* issues);
```

实现（`src/electro_optical_sensor/session/EosSession.cpp:63-70`）中，**无论校验是否通过都会调用 `Create(config)`**。

**反直觉点**：名字暗示"校验通过才创建"，实际只是附加了诊断信息。若校验发现 `horizontal_fov_deg <= 0`，Session 仍然被返回，后续计算中 `Clamp` 会静默修正为默认值，用户无感知。

**建议**：重命名为 `CreateWithDiagnostics`，或在存在 error 时返回空 Session / `std::optional`。

---

### 2. `EosRuntimeConfigPatch` 中 `power_on` 存在两层冗余控制

**文件**：`include/1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h`

```cpp
bool has_mission{false};
EosMissionConfig mission{};        // ← mission.power_on 控制 sensor_enabled

bool has_sensor_enabled{false};
bool sensor_enabled{true};         // ← 直接控制 sensor_enabled
```

解析器（`src/electro_optical_sensor/runtime/EosRuntimeConfigResolver.cpp:133-134`）中，`has_mission` 会把 `mission.power_on` 写入 `sensor_enabled`；随后 `has_sensor_enabled`（L191-192）又会覆盖它。

**反直觉点**：用户设 `has_mission=true, mission.power_on=false`，期望关机；但若同一个 patch 还设了 `has_sensor_enabled=true, sensor_enabled=true`，则 `power_on=false` 被静默覆盖。用户无法从结构上直观看出优先级关系（后者胜出），且没有日志警告冲突。

**建议**：移除 `has_sensor_enabled` / `sensor_enabled` leaf shortcut，统一通过 `has_mission` + `mission.power_on` 控制；或在两者同时存在时记录 warning 日志。

---

### 3. `Step()` 在校验失败/传感器关机时静默返回旧数据

**文件**：`include/1q/electro_optical_sensor/session/EosSession.h`

```cpp
EosOutputFrame Step(const EosCycleInput& input);      // ← 返回旧帧，无失败标识
EosCycleResult StepWithResult(const EosCycleInput& input);  // ← 包含完整诊断
```

`Step()` 内部调用 `controller.RunOnce()` + `BuildCycleResult()`，但只返回 `output_frame`。当校验失败时，`EosController::RunOnce`（L76-85）将 `latest_output` 保留为上一帧旧值，`Step()` 直接返回它。

**反直觉点**：用户拿到的 `EosOutputFrame` 看起来完全合法（`cycle_index`、`detections` 都有值），但实际是上一周期的过期数据。没有 `bool` 标志或特殊 `cycle_index` 来区分"本轮计算"和"复用旧值"。

**建议**：在 `EosOutputFrame` 中增加 `bool executed_this_cycle` 字段；或废弃 `Step()`，统一使用 `StepWithResult()`。

---

### 4. `EosInputValidation.h` 声明了 7 个环境相关校验码，全部是死代码

**文件**：`include/1q/electro_optical_sensor/session/EosInputValidation.h:32-45`

```cpp
kInvalidSolarIrradiance,          // 从未使用
kInvalidCloudCoverageRatio,       // 从未使用
kInvalidAmbientWindSpeed,         // 从未使用
kInvalidBackgroundTemperature,    // 从未使用
kNonFiniteSolarAngles,            // 从未使用
kInvalidSolarAltitudeRange,       // 从未使用
kInconsistentDayNightType,        // 从未使用
```

这些字段（太阳辐照度、云量、风速、背景温度、太阳角、昼夜类型）已在环境域重构中从 `EosCycleInput` 移入 `EosEnvironmentConfig`（会话初始化配置），但校验码声明未同步清理。`EosInputValidation.cpp` 中 `ValidateEosCycleInput()` 不引用这些码。

**反直觉点**：用户看到 `kInvalidSolarIrradiance`，以为可以在 `EosCycleInput` 中传入太阳辐照度并获得校验反馈，但该字段根本不在 CycleInput 中，校验码永远不会触发。

**建议**：删除这 7 个枚举值，或将其移入 `EosSessionConfigValidation.h` 的 `ConfigValidationCode` 中。

---

## 🟡 中严重度

### 5. `EosSessionConfigBuilder` 无 `PolicyEditor`，Mission Profile 隐式跨域改策略

**文件**：`src/electro_optical_sensor/session/EosSessionConfigBuilder.cpp:27-61`

```cpp
// EosSessionConfigBuilder.h — 只有三个 Editor
MissionEditor Mission() noexcept;
HardwareEditor Hardware() noexcept;
EnvironmentEditor Environment() noexcept;
// PolicyEditor? ❌ 不存在
```

`ApplyEosMissionSemanticConfig()` 在设置 Mission Profile 时会**顺带覆写** `policy.detection.minimum_snr_db`：

```cpp
case EosMissionProfile::kWideAreaSearch:
    d.minimum_snr_db = 6.0f;     // ← 隐式修改了策略域！
case EosMissionProfile::kLongRangeSurveillance:
    d.minimum_snr_db = 3.0f;     // ← 隐式修改了策略域！
```

**反直觉点**：
- Mission Profile 名字暗示只影响"任务"参数，实际跨域改写 `policy.detection.minimum_snr_db`。
- 用户若想独立设置 `minimum_snr_db`，Builder 中没有对应的 Editor 入口。
- 杂散光（StrayLight）配置无任何 Profile 预设，只能手动逐字段填写。

**建议**：增加 `PolicyEditor`，至少覆盖 `minimum_snr_db` 和杂散光开关；或在 Mission Profile 文档中明确标注跨域副作用。

---

### 6. `ApplyRuntimeConfig()` 吞掉返回值

**文件**：`src/electro_optical_sensor/session/EosSession.cpp:83-85`

```cpp
void EosSession::ApplyRuntimeConfig(const config::EosRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);  // ← 返回值被显式丢弃
}
```

`TryApplyRuntimeConfig` 在校验失败时返回 `false`，但 `ApplyRuntimeConfig` 无视它。

**反直觉点**：名字不含 "Try"，暗示"总是成功"；实际上可能静默失败。

**建议**：返回 `bool`，或废弃此方法，统一使用 `TryApplyRuntimeConfig`。

---

### 7. `has_mission=true` 隐式覆写 `sensor_enabled`

**文件**：`src/electro_optical_sensor/runtime/EosRuntimeConfigResolver.cpp:133-135`

```cpp
if (patch.has_mission) {
    resolved.next_config.scan = patch.mission;
    resolved.next_config.sensor_enabled = patch.mission.power_on;  // ← 隐式副作用
    resolved.reset_scan_phase = true;
}
```

用户只打算修改扫描参数（如 `scan_rate_deg_per_sec`），通过 `has_mission=true` 传入完整的 `EosMissionConfig`。但如果构造 `EosMissionConfig` 时 `power_on` 被设为 `false`，传感器会被意外关机。

**反直觉点**：Mission 域 patch 的"扫描参数"和"设备开关"两个职责耦合。Leaf-level shortcut `has_scan_rate_deg_per_sec` 可以单独改扫描角速度而不触发此副作用，但这个更安全的替代路径不够显眼。

**建议**：在 `EosRuntimeConfigPatch` 文档中明确标注 `has_mission` 会覆写 `sensor_enabled`；或拆分 `has_scan_params` 与 `has_power_state`。

---

### 8. `range_m` 校验与 pipeline 兜底策略不一致

**文件**：
- `include/1q/electro_optical_sensor/session/EosSceneTypes.h` — `range_m` 默认 `0.0f`
- `src/electro_optical_sensor/session/EosInputValidation.cpp:67-70` — 校验 `range_m <= 0` 为 error
- `src/electro_optical_sensor/pipeline/EosPipeline.cpp:192` — `SafePositive(target.range_m, 1000.0f)`

**反直觉点**：校验层说 `range_m <= 0` 非法，但 pipeline 内部用 `SafePositive` 兜底为 1000m。如果用户绕过校验直接调 pipeline，`range_m=0` 不会崩溃但会静默变为 1000m。校验和 pipeline 的兜底策略不一致。

**建议**：pipeline 内不兜底，依赖校验层保证输入合法；或在兜底时记录 warning 日志。

---

## 🟢 低严重度

### 9. `EosPipelineAbortReason::kSensorPoweredOff = 4` 跳跃编号

**文件**：`include/1q/electro_optical_sensor/session/EosOutputTypes.h`

```cpp
enum class EosPipelineAbortReason {
    kNone = 0,
    kValidationRejected,          // = 1
    kOutputContractViolation,     // = 2
    kRuntimeStateRestoreRejected, // = 3
    kSensorPoweredOff = 4         // ← 显式 = 4，与其他隐式编号混排
};
```

前 4 个值是隐式编号（0,1,2,3），最后一个显式写 `= 4`。混合隐式/显式编号让读者怀疑是否有意跳过了某个值。

**建议**：统一隐式编号，删除 `= 4`。

---

### 10. `input_cycle_index` 与 `output_frame.cycle_index` 语义重叠

**文件**：`include/1q/electro_optical_sensor/session/EosCycleResult.h`

```cpp
struct EosCycleResult {
    std::uint32_t input_cycle_index{0U};   // ← 输入的 cycle index
    EosOutputFrame output_frame{};          // ← 内含 cycle_index
    ...
};
```

Controller 中 `output_frame.cycle_index` 被赋值为 `input.cycle_index`（`EosController.cpp:126`），两者始终相等。

**建议**：移除 `EosCycleResult::input_cycle_index`，统一使用 `output_frame.cycle_index`。

---

### 11. `EnvironmentEditor` 只能设 preset，不能设其他环境字段

**文件**：`include/1q/electro_optical_sensor/config/EosSessionConfigBuilder.h`

```cpp
class EnvironmentEditor {
    EnvironmentEditor& WithEnvironmentPreset(config::EosEnvironmentPreset preset) noexcept;
    // ← 没有 WithSolarAltitudeDeg()、WithCloudCoverageRatio() 等
};
```

用户如果想在 Builder 中修改太阳高度角或云量，必须绕过 Editor 直接操作 `config_.environment.scenario_config.solar_altitude_deg`，破坏了 Builder 的封装。

**建议**：在 `EnvironmentEditor` 中增加常用字段的快捷设置方法。

---

### 12. 跨模块一致性瑕疵

EOS 与其余模块的公开 API 对比：

| 问题 | EOS 状态 | 对比模块 |
|------|----------|---------|
| `dt_sec` 类型 | `float` ✅ | AR 用 `double`（其余 `float`） |
| 输出帧命名 | `EosOutputFrame` ✅ | AR 用 `TrackOutputFrame`（历史遗留） |
| `ExternalInputAdapter` | 有 ✅ | ESR 缺失 |
| Session 析构 `noexcept` | 有 ✅ | AR/ESR 缺失 |
| `ApplyRuntimeConfigWithResult` | 无 | ESR 独有 |
| Builder `PolicyEditor` | **无** ❌ | 其余模块各有覆盖策略域的 Editor |

EOS 在多数项上是"正确"的一方，但 `PolicyEditor` 缺失是 EOS 独有的问题。

---

## 汇总

| # | 问题 | 严重度 | 类型 | 文件 |
|---|------|--------|------|------|
| 1 | `CreateWithValidation` 校验失败仍创建 | 🔴 | 语义误导 | `EosSession.h:63` |
| 2 | `power_on` 两层冗余控制 | 🔴 | 冗余冲突 | `EosRuntimeConfigPatch.h` |
| 3 | `Step()` 静默返回旧数据 | 🔴 | 静默失败 | `EosSession.h:72` |
| 4 | 7 个环境校验码是死代码 | 🔴 | 死代码 | `EosInputValidation.h:32-45` |
| 5 | Builder 无 PolicyEditor | 🟡 | 隐式副作用 | `EosSessionConfigBuilder.h` |
| 6 | `ApplyRuntimeConfig()` 吞返回值 | 🟡 | 静默失败 | `EosSession.cpp:83` |
| 7 | `has_mission` 隐式覆写 `sensor_enabled` | 🟡 | 耦合副作用 | `EosRuntimeConfigResolver.cpp:133` |
| 8 | `range_m` 校验与 pipeline 兜底不一致 | 🟡 | 策略不一致 | `EosPipeline.cpp:192` |
| 9 | `kSensorPoweredOff = 4` 跳跃编号 | 🟢 | 命名瑕疵 | `EosOutputTypes.h` |
| 10 | `input_cycle_index` 与 `output_frame.cycle_index` 重叠 | 🟢 | 冗余字段 | `EosCycleResult.h` |
| 11 | EnvironmentEditor 只能设 preset | 🟢 | 粒度不对称 | `EosSessionConfigBuilder.h` |
| 12 | 跨模块 PolicyEditor 缺失 | 🟢 | 一致性 | — |
