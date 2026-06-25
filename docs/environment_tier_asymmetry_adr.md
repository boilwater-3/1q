# Environment 子系统层次非对称决策（ADR）

Date: 2026-06-25
Status: Accepted. 基于跨域对称性评审 P3-a 评估结论。
关联分支：`refactor/cross-domain-symmetry-contract`。

## 背景

跨域对称性评审将「Environment 子体系统一」列为 P3-a（最高工作量、最高风险、最深认知负担根因）。四域当前环境配置层次如下：

| 域 | 层次结构 | `BuildModelConfigFromScenario` 公开入口 | Preset 枚举 | Model↔Scenario 映射 |
| --- | --- | --- | --- | --- |
| AR (airborne_radar) | Scenario / Model / Default | ✅ public inline | 无（用 `JammingSensitivityProfile`） | 恒等（字段一对一拷贝） |
| ESR | Scenario / Model / Default | ✅ public inline | ✅ `EsrEnvironmentPreset` | 恒等（字段一对一拷贝） |
| EOS | Scenario / Model / Default | ⚠️ 仅 internal（`EosPipelineConfigMapper`） | ✅ `EosEnvironmentPreset` | **非恒等**（preset → 实际物理参数解析） |
| SAR | **flat**（单一 `SarEnvironmentConfig`） | ❌ 无 | ❌ 无 | N/A |

SAR 的 `SarEnvironmentConfig` 是 5 个直接物理参数（`terrain_reference_altitude_m`、`atmospheric_loss_db_per_km`、`surface_backscatter_sigma0_db`、`use_flat_earth_geometry`、`enable_atmospheric_attenuation`），无 preset 概念。

## 决策

**不强制统一四域 Environment 层次结构。** 维持现状：EOS/ESR/AR 三层、SAR 扁平。

理由：经过评估，强统一在所有可行方向上都是负收益或高风险。

### 否决的方向 A：给 SAR 补完整三层（加法）

- SAR 当前全是直接物理参数，没有「场景语义 → 物理参数」的解析需求。
- 硬补 `SarEnvironmentScenarioConfig` / `SarEnvironmentModelConfig` / `SarEnvironmentDefaultConfig` 会引入空壳层 + 恒等映射函数。
- **结论：纯粹的形式主义，无语义收益，徒增认知负担与维护面。** ❌

### 否决的方向 B：把 EOS/ESR/AR 拍平到 SAR 形态（减法）

- EOS 的三层是「真三层」：`ScenarioConfig.preset` → ModelConfig 物理参数做了**非恒等映射**（`EosPipelineConfigMapper.cpp` 中 `if (preset == kHumid) ...`）。拍平会破坏 preset→params 解析语义。
- AR/ESR 的 `BuildModelConfigFromScenario` 已被 contract test 锁死为公开入口，拍平属破坏性变更。
- **结论：破坏既有语义且违反已冻结的公开合同。** ❌

### 否决的方向 C：仅给 SAR 补轻量 Default 包裹层

- 即便只加 `SarEnvironmentDefaultConfig` 包裹 flat struct，仍是为对称而对称的空壳，且会把「初始化语义」强行引入一个本无该区分的子系统。
- **结论：与方向 A 同病，规模更小但性质相同。** ❌

### 采纳：维持非对称 + 文档化

接受「四域 Environment 子系统层次形态有意不同」，并在本文记录根因，防止后续以「对称性」为由再次发起空壳化改造。

## 根因分析：为什么非对称是正确的

三层结构的存在意义是**隔离外部场景语义输入与内部算法消费参数**。这一隔离只有当存在「场景语义 → 物理参数」的非平凡解析时才有价值：

- **EOS**：有 preset（Standard/Humid/Dusty/Turbulent/Maritime）→ 物理参数解析。三层有价值，且 Scenario→Model 是非恒等映射。
- **ESR**：有 preset（Standard/LowClutter/DenseClutter/Jammed）。目前映射恒等，但预留了未来 preset→参数解析的扩展点。
- **AR**：无 preset，但有 `JammingSensitivityProfile` 语义档位（控制干扰功率门限），以及丰富的场景事实（气象、植被、干扰源）。三层用于隔离场景事实与运行期派生量。
- **SAR**：无 preset、无语义档位，5 个字段全是用户直接可调的物理参数。没有「需要解析的场景语义」，因此没有 Scenario/Model 分层的语义基础。扁平结构是对其数据形态的诚实表达。

**关键洞察**：认知负担的根因不是「SAR 少了两层」，而是「四域看起来像但本质不同」。强行让 SAR 长得像 EOS 反而制造**假对称**（pseudo-symmetry）——形式上对齐，语义上空洞，调用方必须记住「SAR 的 ScenarioConfig 其实就是 ModelConfig 的别名」，这比明确告诉调用方「SAR 环境就是一组物理参数」负担更重。

## 已知遗留 gap（独立于本决策）

EOS 的 `BuildModelConfigFromScenario` 目前是 internal（`EosPipelineConfigMapper`），未与 AR/ESR 的 public inline 入口对齐。这是**入口对称性 gap**，不涉及层次结构，可单独修复且低风险。本 ADR 不阻塞该修复，但将其列为独立后续项，不混入「层次统一」语境。

## 守护

本决策不由 contract test 强制（因为「不统一」无法用 lint 守护）。后续若有人提议给 SAR 加三层，应先引用本 ADR，并证明存在「场景语义 → 物理参数」的真实解析需求（而非对称性诉求）。

## 参考

- `include/1q/airborne_radar/environment/EnvironmentConfig.h`（三层模板）
- `include/1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h`
- `include/1q/electro_optical_sensor/environment/EosEnvironmentConfig.h`
- `include/1q/sar/config/SarEnvironmentConfig.h`（flat）
- `src/electro_optical_sensor/runtime/EosPipelineConfigMapper.cpp`（EOS preset→params 非恒等映射）
- `docs/public_api_customization_boundary_contract.md`
