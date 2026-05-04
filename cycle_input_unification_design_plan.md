# 三模块 CycleInput 统一设计方案

## 1. 背景

AR、EOS、ESR 的单周期执行入口都已经暴露 `CycleInput`，但实际设计语言仍不统一：

- AR 的公开 `RadarCycleInput` 当前只有 `cycle_index / dt_sec / platform_pose / scene`，环境变化通过 `RadarSession::Step(input, EnvironmentSceneState)` 和 trace/replay 的独立 `scene_state` 事件旁路提交。
- EOS 的 `EosCycleInput` 包含 `environment`，并直接铺开太阳、云量、风速、背景温度等事实字段。
- ESR 的 `EsrCycleInput` 包含 `environment`，并在运行期冻结为环境快照，但输入校验目前主要覆盖周期、平台和场景，环境事实校验不完整。

这会导致外部接入方无法形成稳定认知：同样是“本周期外部世界事实”，AR 要走额外 overload，EOS 与 ESR 的环境字段语言不一致；trace/replay 也无法保证三模块都以 `cycle_input` 作为完整周期事实边界。

## 2. 统一原则

1. `CycleInput` 是唯一的单周期事实入口。
2. `environment` 表示“来自外部世界、且库内无法可靠自造的本周期环境事实输入”。
3. `environment` 不承载模型配置、调参开关、策略阈值或 runtime patch。
4. `environment` 直接承载高层环境事实字段，不再额外包一层 `observation`。
5. `session config` 表示初始化基线，`runtime config patch` 表示热更新配置，二者不替代 `CycleInput.environment`。
6. trace/replay 的 `cycle_input` 事件必须足以复现一个周期的外部输入事实，不再依赖模块私有的额外事件补齐环境。

## 3. 标准公开形状

三个模块的 `*CycleInput` 统一采用以下外层语言：

```cpp
struct *CycleInput {
  std::uint32_t cycle_index;
  float dt_sec;
  *PoseState platform_pose;
  *Scene... scene;
  *EnvironmentInput environment;
};
```

`*EnvironmentInput` 统一采用直接字段语言：

```cpp
struct *EnvironmentInput {
  // module-specific high-level environment facts
};
```

其中：

- `platform_pose`：当前周期平台位姿和运动状态。AR/EOS 可继续使用 `oneq::foundation::PoseState`，ESR 是否继续使用 `EsrPoseState` 单独评估，但字段角色和文档语义必须一致。
- `scene`：当前周期目标或辐射源事实列表。模块领域名可以不同，但都必须是外部场景事实输入，不包含执行策略。
- `environment`：当前周期环境事实输入。字段应是外部接入方可获得或可合理估计的高层语义量。

## 4. 环境输入入口标准

### 4.1 结论

三个模块的执行链路必须统一为：

```text
外部增量维护接口 -> 生成完整 CycleInput.environment 快照 -> 校验 -> 冻结环境上下文 -> pipeline
```

其中 `CycleInput.environment` 的语义保持“完整快照”，但调用方不应被迫每周期重复填写所有长期不变字段。长期不变字段由统一的环境输入状态对象或 builder 保留，只有进入 `StepWithResult(input)` 的那一刻才 materialize 为完整快照。

### 4.2 必须区分的三类对象

| 层级 | 对象 | 语义 | 是否允许局部更新 | 是否进入 replay |
| --- | --- | --- | --- | --- |
| 执行输入 | `*CycleInput.environment` | 本周期完整环境事实快照 | 否 | 是 |
| 输入状态 | `*EnvironmentInputState` 或等价 builder 内部状态 | 调用方侧的当前环境事实状态 | 是 | 否 |
| 配置更新 | `*RuntimeConfigPatch.environment` | 模型、策略、预设、参数热更新 | 是 | 作为 runtime patch |

规则：

1. `CycleInput.environment` 不表达“只改某几个字段”，所有字段都按完整快照解释。
2. `EnvironmentInputState` 才表达“沿用上一周期，只改某几个字段”。
3. replay 只记录 materialized 后的完整 `cycle_input.environment`，不记录环境 delta。
4. runtime config patch 不承担环境事实输入职责。

### 4.3 推荐公共入口

每个模块提供同构的状态型环境输入辅助对象：

```cpp
class *EnvironmentInputState {
 public:
  *EnvironmentInputState& Reset(const *EnvironmentInput& snapshot);
  *EnvironmentInputState& Update(const *EnvironmentInputPatch& patch);
  *EnvironmentInput Snapshot() const;
};
```

每个模块提供同构的 patch 类型：

```cpp
struct *EnvironmentInputPatch {
  bool has_<field>{false};
  <field_type> <field>{};
};
```

设计约束：

- `Reset(snapshot)` 表示替换整份当前环境事实。
- `Update(patch)` 只更新 `has_* == true` 的字段，其余字段沿用状态对象当前值。
- `Snapshot()` 返回完整 `*EnvironmentInput`，可直接写入 `CycleInput.environment`。
- patch 类型只用于调用方侧输入维护，不直接进入 `StepWithResult()`，避免执行链路出现隐式状态。

### 4.4 CycleInputBuilder 的统一职责

现有 `*CycleInputBuilder` 应扩展为同一入口形态，而不是各模块自行决定是否接收环境：

```cpp
static bool Build(const *ExternalPoseInput& platform,
                  const std::vector<*ExternalSceneInput>& scene,
                  float dt_sec,
                  const *EnvironmentInput& environment,
                  *CycleInput* out);
```

保留无 environment 参数的便捷重载时，必须明确它使用默认环境快照：

```cpp
static bool Build(const *ExternalPoseInput& platform,
                  const std::vector<*ExternalSceneInput>& scene,
                  float dt_sec,
                  *CycleInput* out);
```

该便捷重载不得读取 session 内部上一周期环境，也不得表达“沿用上一周期”。如果调用方要沿用上一周期，应显式持有 `*EnvironmentInputState`：

```cpp
EosEnvironmentInputState environment_state;
environment_state.Update(patch_for_cloud_only);

EosCycleInput input;
EosCycleInputBuilder::Build(platform, targets, dt_sec, environment_state.Snapshot(), &input);
session.StepWithResult(input);
```

### 4.5 三模块统一验收点

- AR、EOS、ESR 都有 `*EnvironmentInput`，且 `*CycleInput.environment` 是完整快照。
- AR、EOS、ESR 都有同构的 `*EnvironmentInputPatch` 与 `*EnvironmentInputState`，用于调用方侧局部更新。
- AR、EOS、ESR 的 `*CycleInputBuilder::Build(...)` 都支持显式传入完整 environment。
- 无 environment 参数的 builder 重载只使用默认环境，不暗中沿用上一周期。
- `StepWithResult(input)` 不接受 environment patch，不读取上一周期 environment 来补字段。
- replay codec 只编码完整 `cycle_input.environment`。
- validation 按完整 environment 校验，不因 patch 缺字段而跳过字段校验。

## 5. 模块目标形状

### 5.1 AR

目标公开形状：

```cpp
struct RadarCycleInput {
  std::uint32_t cycle_index{0U};
  float dt_sec{1.0f};
  oneq::foundation::PoseState platform_pose{};
  RadarSceneTargetList scene{};
  RadarEnvironmentInput environment{};
};

struct RadarEnvironmentInput {
  environment::AtmosphericPhysicsConfig atmospheric_observation{};
  environment::AtmosphericDerivedContext atmospheric_context{};
  environment::VegetationScatterPhysicsConfig surface_observation{};
  environment::JammerEmitterStateList jammer_sources{};
};
```

AR 当前先把原 `EnvironmentSceneState` 可消费的环境事实收敛到 `RadarCycleInput.environment`，删除空结构和旁路 overload。后续若继续提高语义纯度，应把 `AtmosphericPhysicsConfig`、`AtmosphericDerivedContext`、`VegetationScatterPhysicsConfig` 这类偏模型名的类型进一步拆为更面向外部接入方的天气、地表与空间天气事实类型。

建议 AR 观测语言按高层事实划分：

- `atmospheric_observation`：湿度、降水、能见度、温度、气压或折射环境等级等外部天气事实。
- `surface_observation`：地表/植被/海况/地形粗糙度等影响杂波和衰减的高层事实。
- `jammer_sources`：本周期可见干扰源列表，保留中心频率、带宽、功率、激活状态、置信度等事实字段。

内部通过 `RadarEnvironmentInputMapper` 或同等私有组件把观测映射为现有 `EnvironmentSceneState` / 环境服务需要的冻结状态。

实施时删除公开 `RadarSession::Step(input, EnvironmentSceneState)`、`StepWithResult(input, EnvironmentSceneState)`、`RadarTraceSession` 对应 overload，以及 trace/replay 中独立 `scene_state` 周期事件。AR replay 的 `cycle_input` schema 中已有空环境表，应改为真实 `RadarEnvironmentInput` 载荷。

### 5.2 EOS

目标公开形状：

```cpp
struct EosCycleInput {
  std::uint32_t cycle_index{0U};
  float dt_sec{1.0f};
  oneq::foundation::PoseState platform_pose{};
  EosSceneTargetList scene{};
  EosEnvironmentInput environment{};
};

struct EosEnvironmentInput {
  float solar_altitude_deg{45.0f};
  float solar_azimuth_deg{180.0f};
  float solar_irradiance_w_m2{800.0f};
  float cloud_coverage_ratio{0.2f};
  float ambient_wind_speed_mps{0.0f};
  DayNightType day_night_type{DayNightType::kDay};
  float background_temperature_k{290.0f};
};
```

当前 `EosEnvironmentInput` 中的太阳高度角、太阳方位角、太阳辐照度、云量、风速、昼夜类型、背景温度都是合理的外部事实字段，应继续作为 `input.environment.*` 的直接字段或映射后的冻结上下文字段。

EOS 当前“环境冻结”更像是 pipeline 直接读取输入。统一后建议补一个明确的环境冻结阶段，即使内部实现仍很薄，也应让执行链路与 AR/ESR 对齐：

1. 校验 `cycle_input.environment`。
2. 将观测映射为本周期 EOS 环境上下文或快照。
3. pipeline 只消费冻结后的上下文，避免不同 pipeline 分支重复解释原始输入。

### 5.3 ESR

ESR 当前公开形状最接近目标：

```cpp
struct EsrCycleInput {
  std::uint32_t cycle_index{0U};
  float dt_sec{1.0f};
  EsrPoseState platform_pose{};
  EsrSceneEmitterList scene{};
  EsrEnvironmentInput environment{};
};

struct EsrEnvironmentInput {
  environment::EsrPropagationEnvironmentProfile propagation_profile{
      environment::EsrPropagationEnvironmentProfile::kTypical};
  environment::EsrClutterDensityLevel clutter_density{environment::EsrClutterDensityLevel::kMedium};
  float spectrum_occupancy_ratio{0.0f};
  environment::EsrAtmosphericObservation atmospheric_observation{};
  environment::EsrJammerSourceList jammer_sources{};
};
```

需要补齐的不是主形状，而是契约闭环：

- 在 `EsrInputValidation` 中校验 `environment` 的范围、枚举和 jammer 字段。
- trace/replay schema 与 codec 继续以 `cycle_input.environment` 为完整周期输入事实，不引入额外环境事件。
- 文档统一使用“环境观测输入”“冻结环境快照”等术语，避免与环境配置混用。

## 6. 标准执行链路

每个模块的 `StepWithResult(input)` 应遵循同一顺序：

1. 校验 `cycle_index / dt_sec / platform_pose / scene / environment`。
2. 提交 pending runtime config patch，并记录可回滚状态。
3. 从 `input.environment` 冻结本周期环境上下文或快照。
4. 将 `cycle_index / dt_sec / platform_pose / scene` 写入本周期执行上下文。
5. 执行控制器和 pipeline。
6. 组装带同一 `cycle_index` 的输出帧。
7. 若校验后、输出前失败，回滚 runtime config 与环境冻结状态，返回结构化失败结果。

`Step(input)` 继续作为便捷入口，但不得拥有不同语义；它只应包一层 `StepWithResult(input)` 并返回输出帧或上一有效输出。

## 7. 标准 replay/trace 语义

统一后 replay 事件边界如下：

- `session_config`：初始化配置。
- `runtime_config_patch`：热更新配置。
- `cycle_input`：完整单周期外部事实输入，包含 `environment`。
- `cycle_output`：本周期输出。

AR 的独立 `scene_state` 事件属于旧旁路，应在实施阶段删除。EOS/ESR 不应新增类似旁路。非法 trace 行为按统一规则拒绝：

- `cycle_input` 出现在 `session_config` 前：拒绝。
- `cycle_output` 没有 pending `cycle_input`：拒绝。
- 同一 pending `cycle_input` 被多个环境事件补写：目标架构中不存在这种事件，因此 schema 层面消除。

## 8. 文件级实施计划

### 阶段一：公开输入类型收敛

- `include/1q/airborne_radar/session/RadarCycleInput.h`：恢复 `environment` 字段，但使用真实 `RadarEnvironmentInput`。
- `include/1q/airborne_radar/session/RadarEnvironmentInput.h`：新增公开输入类型，并直接承载环境事实字段。
- `include/1q/electro_optical_sensor/session/EosEnvironmentInput.h`：保持 `input.environment.*` 直接字段语言。
- `include/1q/electro_optical_sensor/environment/EosEnvironmentTypes.h`：新增或承载 `EosEnvironmentObservation`。
- `include/1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h`：保留形状，修正文档用语。
- `include/1q/airborne_radar/session/RadarEnvironmentInputPatch.h`：新增调用方侧环境局部更新 patch。
- `include/1q/electro_optical_sensor/session/EosEnvironmentInputPatch.h`：新增调用方侧环境局部更新 patch。
- `include/1q/electronic_surveillance_radar/session/EsrEnvironmentInputPatch.h`：新增调用方侧环境局部更新 patch。
- `include/1q/*/session/*EnvironmentInputState.h`：新增调用方侧环境状态维护对象。

### 阶段二：运行链路收敛

- `src/airborne_radar/session/RadarSession.cpp`：删除 `EnvironmentSceneState` overload，改为从 `input.environment` 冻结环境。
- `include/1q/airborne_radar/session/RadarSession.h`、`RadarTraceSession.h`：删除公开旁路 overload。
- `src/airborne_radar/environment/*`：增加观测到内部环境状态的私有 mapper。
- `src/electro_optical_sensor/session/*`、`src/electro_optical_sensor/runtime/*`：补明确环境冻结组件或上下文。
- `src/electro_optical_sensor/signal/pipeline/EosPipeline.cpp`：从冻结上下文读取环境。
- `src/electronic_surveillance_radar/session/EsrInputValidation.cpp`：补环境观测校验。
- `include/1q/*/session/*EnvironmentInputState.h`：以轻量 header-only 方式实现 `Reset / Update / Snapshot`。
- `src/*/session/*CycleInputBuilder.cpp`：增加显式 environment 参数重载。

### 阶段三：trace/replay 收敛

- `schemas/replay/airborne_radar_replay.fbs`：把空 `RadarCycleEnvironmentInput` 改为真实观测载荷，删除 `scene_state` event 依赖。
- `src/airborne_radar/session/RadarReplayFlatbufferCodec.cpp`、`RadarReplaySession.cpp`、`RadarTraceSession.cpp`：移除 `scene_state` codec/callback/write path。
- `schemas/replay/eos_replay.fbs`：保持环境字段直接位于 `EosEnvironmentInput`。
- `schemas/replay/esr_replay.fbs`：确认命名与 public type 一致，必要时只做命名清理。
- 更新 generated headers、codec 单测、trace session adapter 测试。

### 阶段四：适配器、示例和契约测试

- 更新 `*CycleInputBuilder`，统一提供 `WithEnvironment(...)` 或等价语义方法。
- 更新 external input adapter，保证外部输入被填入 `cycle_input.environment`。
- 更新 examples 和 consumer/contract 测试。
- 更新 `tests/contract/check_public_api_boundary.cmake` 和 public header smoke test。

## 9. 测试策略

最小验证集合：

- AR：session step、trace session、replay、runtime rollback、环境冻结 mapper 单测。
- EOS：input validation、pipeline 环境影响、trace/replay codec、冻结上下文单测。
- ESR：environment validation、environment updater、replay codec、pipeline 输出影响单测。
- 三模块 environment input state：`Reset` 整包替换、`Update` 只改 `has_*` 字段、`Snapshot` 返回完整快照。
- 三模块 cycle input builder：显式 environment 重载写入完整快照；便捷重载使用默认环境，不沿用任何上一周期状态。
- 三模块 contract：public header smoke、public API boundary、cycle input builder convenience。

完整验收使用：

```bash
cmake --build --preset llvm-ninja-debug-local > /tmp/1q-build.log 2>&1 || { tail -n 80 /tmp/1q-build.log; false; }
ctest --preset llvm-ninja-debug-local -Q --output-on-failure
```

## 10. 验收标准

- 三模块 `CycleInput` 都包含 `cycle_index / dt_sec / platform_pose / scene / environment`。
- 三模块 `EnvironmentInput` 都直接承载本周期环境事实载荷，不再使用空结构或 `observation` 包装层。
- AR 不再公开 `Step(input, EnvironmentSceneState)` 旁路，也不再需要 replay `scene_state` 周期事件。
- EOS 的环境输入字段通过统一的 `cycle_input.environment` 参与校验、replay 和 pipeline。
- ESR 对 `environment` 有完整输入校验。
- trace/replay 中 `cycle_input` 足以表达完整周期输入事实。
- 所有新增公开头文件都纳入安装、public header smoke 和 public API boundary 检查。
- 不保留旧字段、旧 overload、旧 replay 环境旁路或兼容解析分支，除非后续明确要求兼容迁移。
