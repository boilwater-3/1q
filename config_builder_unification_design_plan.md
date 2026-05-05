# 三模块 SessionConfigBuilder 统一设计方案

## 1. 背景与问题

当前 AR/EOS/ESR 都有 `SessionConfigBuilder`，但设计语言不一致：

- AR 使用分域 editor：`Detection()/Beam()/Tracking()/Lifecycle()/Environment()`，语义表达能力强，但调用链偏长。
- EOS 使用扁平 `WithWorkMode()/WithDetectionProfile()/WithEnvironmentPreset()`，入口短，但领域边界不明显。
- ESR 也是扁平 `WithWorkMode()/WithPowerOn()/WithScanRateHz()/WithDetectionProfile()/WithEnvironmentPreset()`，mission/policy/environment 混在同一层。
- AR 已删除库内 session config preset factory，但 EOS/ESR 文档和 builder 注释仍使用 `preset` 作为“推荐语义输入”的语言，容易和“业务场景推荐配置”混淆。

用户视角的核心问题是：常见场景需要推荐配置，但这些推荐配置不应作为库内固定 preset API 暴露。库应提供稳定、可组合、边界清晰的语义积木；常见场景应由 example 或业务层具名函数组合这些积木。

## 2. 统一设计原则

### 2.1 三层边界

三模块统一保留以下三层：

1. **四域详细配置层**
   - 类型：`*SessionConfig` 聚合 `hardware / mission / policy / environment`。
   - 用途：精细建模、测试、特殊工程参数。
   - 方式：直接字段赋值。

2. **语义 Builder 层**
   - 类型：`*SessionConfigBuilder`。
   - 用途：用高层语义 profile/mode 构造初始化配置。
   - 要求：不承载业务场景命名，不提供 `MakeXxxScenarioConfig()` 这类库内 preset factory。

3. **业务推荐场景层**
   - 位置：`examples/` 或调用方业务代码。
   - 形态：具名函数返回 `*SessionConfig`，再传入 `*SessionFactory::Create(...)`。
   - 示例：`MakeWideAreaSearchConfig()`、`MakeDenseJammingConfig()`、`CreateStableTrackSession()`。

### 2.2 命名规则

- `With*Profile(...)`：设置策略/算法语义档位。
- `With*Mode(...)`：设置 mission 工作模式。
- `With*Enabled(bool)` 或 `Enable*(bool)`：设置显式开关。
- `With*Default(...)`：整块替换初始化默认域。
- `With*Details(...)`：只用于确实需要公开的少量工程细节；不作为推荐主路径。
- 不新增库内 `presets` namespace。
- 不新增 `MakeDefault*Config()`、`MakeDetection*Config()` 这类公共预设函数。

### 2.3 Builder 不做的事

- 不命名业务场景。
- 不隐藏完整 `SessionConfig` 返回值。
- 不直接创建 session。
- 不提供 runtime patch 能力；运行期热更新统一由 `*RuntimeConfigBuilder` 负责。
- 不把环境事实输入和初始化环境默认配置混为一谈。

## 3. 统一 API 形态

### 3.1 推荐外层骨架

三模块 `SessionConfigBuilder` 应收敛为同一类设计语言：

```cpp
auto config = ModuleSessionConfigBuilder()
                  .Mission()
                  .WithWorkMode(...)
                  .End()
                  .Detection()
                  .WithDetectionProfile(...)
                  .End()
                  .TrackingOrAssociation()
                  .WithTrackingProfile(...)
                  .End()
                  .Environment()
                  .WithEnvironmentDefault(...)
                  .End()
                  .Build();
```

不是每个模块都必须有完全相同的 editor 名称，但相同概念应落到相同层：

| 概念 | AR | EOS | ESR |
| --- | --- | --- | --- |
| 工作模式/扫描/指向 | `Mission()` 或 `Beam()` | `Mission()` | `Mission()` |
| 探测语义 | `Detection()` | `Detection()` | `Detection()` |
| 跟踪/关联语义 | `Tracking()` / `Lifecycle()` | 可无；若后续存在跟踪则加 `Tracking()` | `Association()` 或 `Emitter()` |
| 环境初始化默认 | `Environment()` | `Environment()` | `Environment()` |

### 3.2 AR 目标形态

AR 当前 builder 已接近目标，但应做一次瘦身和命名收敛：

- 将 `Beam()` 更名或补充为 `Mission()`，让工作模式、扫描中心、驻留中心、波束宽度都属于 mission editor。
- `Detection()` 保留探测链路语义，但不要继续扩张为全部硬件细节入口。
- `Tracking()` 保留跟踪策略语义。
- `Lifecycle()` 保留生命周期策略语义。
- `Environment()` 保留初始化环境默认和干扰灵敏度语义。
- 删除内部“preset signature”命名，测试命名改为 profile/builder 语义。

建议 AR 最终示例：

```cpp
auto config = RadarSessionConfigBuilder()
                  .Mission()
                  .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
                  .WithScanCenterDeg({0.0f, 0.0f})
                  .End()
                  .Detection()
                  .EnablePhysicsDetection(false)
                  .WithDetectionIntentProfile(
                      config::profiles::DetectionIntentProfile::kDetectionPriority)
                  .End()
                  .Tracking()
                  .EnableTrackingFilter(true)
                  .WithTrackingPolicyProfile(
                      config::profiles::TrackingPolicyProfile::kFastAssociation)
                  .End()
                  .Lifecycle()
                  .WithLifecyclePolicyProfile(
                      config::profiles::LifecyclePolicyProfile::kFastConfirm)
                  .End()
                  .Build();
```

### 3.3 EOS 目标形态

EOS 当前 builder 是扁平 API，应改成分域 editor：

```cpp
auto config = EosSessionConfigBuilder()
                  .Mission()
                  .WithWorkMode(config::EosWorkMode::kSearch)
                  .WithScanRateDegPerSec(30.0f)
                  .WithFrameRateHz(20.0f)
                  .End()
                  .Detection()
                  .WithDetectionProfile(config::EosDetectionProfile::kBalanced)
                  .End()
                  .StrayLight()
                  .WithStrayLightProfile(config::EosStrayLightProfile::kStandard)
                  .End()
                  .Environment()
                  .WithEnvironmentDefault(default_environment)
                  .End()
                  .Build();
```

EOS 迁移重点：

- `WithWorkMode()` 迁入 `MissionEditor`。
- `WithDetectionProfile()` 迁入 `DetectionEditor`。
- `WithStrayLightProfile()` 迁入 `StrayLightEditor` 或 `OpticsEditor`。
- `WithEnvironmentModelType()` / `WithEnvironmentPreset()` 迁入 `EnvironmentEditor`。
- 文档中避免把 `preset` 描述为推荐 session 配置路径；环境 preset 若保留，应明确它只是环境模型语义字段，不是 session preset factory。

### 3.4 ESR 目标形态

ESR 当前 builder 也是扁平 API，应改成分域 editor：

```cpp
auto config = EsrSessionConfigBuilder()
                  .Mission()
                  .WithWorkMode(config::EsrWorkMode::kSearch)
                  .WithPowerOn(true)
                  .WithScanRateHz(5.0f)
                  .End()
                  .Detection()
                  .WithDetectionProfile(config::EsrDetectionProfile::kBalanced)
                  .End()
                  .Environment()
                  .WithEnvironmentDefault(default_environment)
                  .End()
                  .Build();
```

ESR 迁移重点：

- `WithWorkMode()`、`WithPowerOn()`、`WithScanRateHz()` 迁入 `MissionEditor`。
- `WithDetectionProfile()` 迁入 `DetectionEditor`。
- `WithEnvironmentPreset()` 迁入 `EnvironmentEditor`。
- 运行期 `EnvironmentRuntimeConfigPatch::has_preset` 已标注不支持，应在本轮统一中评估是否彻底删除该弃用字段；若用户允许破坏兼容，应删除。

## 4. 常见场景推荐配置的承载方式

常见场景不进入公共库 API，而是进入 example：

```cpp
RadarSessionConfig MakeWideAreaSearchConfig();
RadarSessionConfig MakeStableTrackConfig();
RadarSessionConfig MakeDenseJammingConfig();
RadarSession CreateWideAreaSearchSession();
RadarSession CreateDenseJammingSession();
```

EOS/ESR 也应补类似 example，而不是新增 `EosSessionConfigPresets` / `EsrSessionConfigPresets`。

推荐 example 组织：

```text
examples/
|-- airborne_radar_session_usage.cpp
|-- electro_optical_sensor_session_usage.cpp
`-- electronic_surveillance_radar_session_usage.cpp
```

每个 example 都展示：

- 推荐配置函数返回 `*SessionConfig`
- 推荐 session 函数返回初始化后的 `*Session`
- 输入构造
- 环境输入/默认环境
- `StepWithResult`
- `Step`
- runtime config patch
- 输出查询/关键结果字段

## 5. 迁移步骤

### 阶段一：标准化文档与命名

- 更新三个 builder 头注释，明确三层边界：
  - `SessionConfigBuilder`：初始化语义积木。
  - 直接字段赋值：详细四域参数。
  - `RuntimeConfigBuilder`：运行期热更新。
  - example/业务层函数：常见场景推荐配置。
- 将测试名中的 `Preset` 改为 `Profile` 或 `BuilderProfile`。
- 清理 AR 残留 preset 语言。

### 阶段二：EOS/ESR Builder 分域化

- 为 EOS 新增 nested editor：
  - `MissionEditor`
  - `DetectionEditor`
  - `StrayLightEditor`
  - `EnvironmentEditor`
- 为 ESR 新增 nested editor：
  - `MissionEditor`
  - `DetectionEditor`
  - `EnvironmentEditor`
- 删除或迁移扁平 `With*` 方法；若已确定不保兼容，直接删除旧扁平入口。

### 阶段三：AR Builder 命名收敛

- 将 `Beam()` 收敛为 `Mission()`。
- 评估是否保留 `Beam()` 作为兼容别名；若按当前项目“不保兼容”的方向，应删除旧名并更新调用点。
- 保持 `Detection/Tracking/Lifecycle/Environment` 不变。

### 阶段四：示例统一

- 保留并完善 `airborne_radar_session_usage.cpp`。
- 新增 EOS/ESR usage example。
- examples 中所有推荐配置都以具名函数返回 `*SessionConfig`，再显式传入 `*SessionFactory::Create` 或 session 构造入口。

### 阶段五：测试与边界

- 更新 public header smoke tests。
- 更新 consumer tests。
- 更新 contract tests。
- 检索禁止项：
  - `SessionConfigPresets`
  - `config::presets`
  - `MakeDefault*SessionConfig`
  - `Make*Mission*SessionConfig`
- 对 builder 做跨模块契约测试：
  - 默认 builder 结果等价默认 config。
  - 从 existing config 起步时，未编辑域保持原值。
  - 编辑某一域不会意外覆盖其他域。
  - `Build()` 返回可创建 session 的完整 config。

## 6. 验收标准

- 三模块 builder 文档使用同一套术语。
- 三模块都不暴露 session config preset factory。
- 常见场景只在 example 或业务层函数中命名。
- EOS/ESR 不再把 mission/policy/environment 扁平混在同一 builder 层。
- AR 不再使用 `Beam()` 作为会话初始化主入口名，统一到 `Mission()`。
- public API smoke/consumer/contract tests 全部更新。
- `cmake --build --preset llvm-ninja-debug-local` 通过。
- `ctest --preset llvm-ninja-debug-local -Q --output-on-failure` 通过。

