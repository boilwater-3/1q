# EnvironmentConfig 统一设计与落地规划

## 1. 背景与问题定义

当前 `airborne_radar`(AR)、`electro_optical_sensor`(EOS)、`electronic_surveillance_radar`(ESR) 在 EnvironmentConfig 体系上存在明显设计语言分裂：

- AR：`environment/` 层有较清晰的 `Scenario -> Model -> Default` 分层，`config/` 仅做别名。
- EOS：`environment/` 仅承载少量模型枚举，`config/` 承载大部分环境字段。
- ESR：`environment/` 与 `config/` 都定义了环境配置结构，存在语义重叠。

这种分裂会导致：

- 同一类语义在不同层重复定义，演化成本高。
- 运行期映射分散在 resolver/mapper，隐式优先级增加行为不确定性。
- 测试覆盖难以收敛为统一标准（默认值、预设映射、patch 行为）。

本方案旨在统一环境配置建模语言，确保仿真模型逻辑清晰、职责边界稳定、后续扩展成本可控。

## 2. 设计目标

### 2.1 必达目标

- 统一三模块环境域配置分层语义。
- 将环境域“真源类型”收敛到 `environment/`，消除 `config/` 并行结构。
- 建立单一的场景到模型映射入口，避免多点语义解释。
- 明确运行时可变与不可变边界，提高仿真可追溯性。

### 2.2 工程目标

- 不引入异常。
- 不在高频仿真环路新增日志。
- 新增/调整字段时，测试增量可预测。
- 便于 future module（新传感器）复用同一套模板。

### 2.3 非目标

- 本方案不讨论 UI/可视化层配置编辑器。
- 不对物理模型公式本身做改造，仅调整配置契约和映射边界。

## 3. 设计原则

- 事实优先：`ScenarioConfig` 只表达场景事实，不混入算法调参语义。
- 单向映射：`Scenario -> Model` 只允许单入口转换。
- 结构单源：同一语义字段只能在一处定义为真源类型。
- 显式优先级：任何覆盖规则（preset/custom/patch）必须可文档化且可测试。
- 最小可变面：Runtime patch 仅允许必要字段热更新。

## 4. 目标架构（统一后）

## 4.1 统一分层

每个模块环境域都采用以下四类类型：

1. `*EnvironmentScenarioConfig`
- 外部输入事实（观测、环境边界条件、场景语义档位）。

2. `*EnvironmentModelConfig`
- 环境服务/算法执行直接消费参数。

3. `*EnvironmentDefaultConfig`
- 初始化默认容器，持有 `scenario_config`。

4. `*EnvironmentRuntimeConfigPatch`
- 运行期热更新 payload（严格受限）。

## 4.2 统一映射入口

每个模块都应提供：

- `BuildModelConfigFromScenario(const ScenarioConfig&) -> ModelConfig`

约束：

- resolver/pipeline/service 禁止再实现第二套语义映射。
- pipeline 仅消费 `ModelConfig`。

## 4.3 config 层职责

`config/` 层不再定义独立环境 struct，统一改为别名：

- `using XxxEnvironmentConfig = environment::XxxEnvironmentDefaultConfig;`

这样 `session/config` 仍可维持 API 聚合层角色，但环境语义真源只在 `environment/`。

## 5. 关键设计决策

## 5.1 是否保留 DefaultConfig

结论：保留，但保持“薄包装”。

理由：

- 与现有 SessionConfig 语义更契合（默认初始化概念仍有价值）。
- 可以承载极少量域级策略开关（若确有必要）。
- 避免初始化调用链大面积重写。

约束：

- `DefaultConfig` 不得复制 `ModelConfig` 全量字段。
- `DefaultConfig` 首选包含 `scenario_config`，而不是散落原子字段。

## 5.2 preset 与 custom 字段冲突处理

现状问题：EOS/ESR 里 `preset + use_preset_defaults + 自定义字段` 并存，易产生隐式覆盖。

统一策略：

- 预设仅作为“场景模板选择器”，在 `ScenarioConfig` 内表达。
- 不使用 `use_preset_defaults` 这类隐式总开关。
- 若需要“模板 + 局部覆盖”，使用显式字段：
  - `preset`
  - `has_custom_overrides`
  - `custom_overrides`（仅允许有限字段）
- `BuildModelConfigFromScenario` 中明确覆盖顺序并单测固化。

## 5.3 Runtime patch 边界

- 仅允许对运行中确需热变的模型字段 patch。
- 禁止 patch 全量 `ScenarioConfig`（避免事实与状态混杂）。
- patch 应尽量语义化（例如“scan center”），并在 resolver 层做强校验。

## 6. 模块级目标形态

## 6.1 AR（基线模块）

目标定位：将 AR 从“可作为参考”提升为“可直接复制的标准模板模块”，为 EOS/ESR 提供结构、命名、约束、测试样板。

### 6.1.1 AR 现状评估（为何可作为模板）

AR 当前已经具备以下关键特征：

- 在 `environment/EnvironmentConfig.h` 内形成了 `ScenarioConfig -> ModelConfig -> DefaultConfig` 的主干结构。
- `config/RadarEnvironmentConfig.h` 使用 alias（`RadarEnvironmentConfig = environment::EnvironmentDefaultConfig`），没有并行定义第二套环境 struct。
- 提供了明确的场景到模型映射入口 `BuildModelConfigFromScenario(...)`，并且当前是“恒等映射”实现，便于后续演进为非恒等映射。
- Builder 与 RuntimePatch 已分离，初始化配置与运行期覆盖职责边界清晰。

### 6.1.2 AR 模板化目标（补齐后应满足）

AR 在本次方案中不追求重构字段，而是补足“规范化能力”，达到可复用模板标准：

1. 类型分层语义可直接复用到其他模块。
2. 字段注释规则统一，避免后续模块语义漂移。
3. 映射函数契约、边界值行为、回退策略可测试且可文档化。
4. Builder/Resolver 的职责边界写成显式规范，可被 EOS/ESR 直接照搬。

### 6.1.3 AR 目录结构（环境配置相关）

以下是 AR 环境配置相关的推荐目录视图（按“公开契约/配置聚合/实现消费”分层）：

```text
include/1q/airborne_radar/
├── environment/
│   ├── EnvironmentConfig.h                    # 环境域核心契约：Scenario/Model/Default + 映射函数
│   ├── EnvironmentDefaultConfigBuilder.h      # 默认配置构造器（仅构造 DefaultConfig）
│   ├── EnvironmentRuntimeConfigPatch.h        # 运行时 patch 契约
│   ├── EnvironmentRuntimeConfigPatchBuilder.h # 运行时 patch 构造器
│   └── airborne_radar_environment.hpp         # 环境域公开聚合头
└── config/
    ├── RadarEnvironmentConfig.h               # 对 environment::EnvironmentDefaultConfig 的别名
    └── airborne_radar_config.hpp              # 配置域公开聚合头

src/airborne_radar/
├── session/
│   ├── RuntimeConfigResolver.h                # 运行时配置解析接口（含环境场景输入位）
│   ├── RadarSessionCompositionRoot.h          # 组合根中持有 runtime 环境配置
│   └── RadarTraceSession.cpp                  # 环境配置序列化/会话输出
└── config/mapping/
    └── RuntimePatchMapper.h                   # 运行时 patch 到内部结构映射

tests/
├── contract/
│   ├── check_public_api_boundary.cmake        # 公共 API 边界检查（含 AR 环境相关头）
│   └── public_headers_smoke_test.cpp          # 公共头可编译性冒烟
└── unit/
    └── ar_runtime_config_resolver_test.cpp    # 运行时 resolver 行为测试（含环境路径）
```

目录约束说明：

- `environment/` 下定义“真源类型”；`config/` 只做 alias 与聚合，不重复定义环境结构。
- `session/` 负责配置解析与运行态装配；`config/mapping/` 负责 patch 语义映射，不承载模型物理语义。
- 测试层保持 `contract -> unit -> integration` 递进，环境契约先验证“可见性与可编译”，再验证行为。

### 6.1.4 AR 标准类型契约（建议固化）

建议把 AR 现有类型契约在文档和注释层面明确为如下标准：

- `EnvironmentScenarioConfig`
  - 只承载外部输入事实：基础大气观测、空间天气上下文、植被场景、干扰源事实列表。
  - 禁止出现“内部算法调参字段”（例如阈值缩放系数这类执行参数）。
- `EnvironmentModelConfig`
  - 当前为 `using EnvironmentModelConfig = EnvironmentScenarioConfig`。
  - 契约上允许未来解耦为独立 struct；调用方不得假设二者永远同型。
- `EnvironmentDefaultConfig`
  - 用于初始化默认值，包含 `scenario_config` 与必要策略枚举（当前为 `jamming_sensitivity_profile`）。
  - 禁止向 `DefaultConfig` 注入运行期热更新语义。
- `EnvironmentRuntimeConfigPatch`
  - 只表达运行期可变项，不能替代初始化配置对象。

### 6.1.5 AR 映射函数契约（建议显式化）

将以下函数视为 AR 环境域“语义边界函数”，并要求每个函数有明确输入/输出与回退规则说明：

- `BuildModelConfigFromScenario(const EnvironmentScenarioConfig&)`
  - 当前行为：恒等映射。
  - 未来约束：若改为非恒等映射，必须同步补充映射单测和变更说明。
- `ResolveEffectiveKFactor(...)`
  - 输入：`AtmosphericDerivedContext + AtmosphericPhysicsConfig`。
  - 输出：有效 `k_factor`，并定义越界回退区间和默认值来源。
- `ResolveEffectiveDayOfYear(...)`
  - 输入：上下文时间信息。
  - 输出：`[1, 366]`，并定义缺失时间时的默认策略。
- `ResolveJammingSensitivityProfile(float threshold_db)`
  - 输入：阈值 dB。
  - 输出：语义档位；阈值区间映射必须在注释和测试中保持一致。

### 6.1.6 AR Builder 与 Resolver 职责（模板规则）

将 AR 的调用边界固化为规则（供 EOS/ESR 迁移时对齐）：

- `EnvironmentDefaultConfigBuilder`
  - 只改 `EnvironmentDefaultConfig` 的直接字段。
  - 不做参数合法性复杂校验（复杂校验交给 resolver）。
  - 不做场景到模型映射。
- Session resolver
  - 初始化阶段执行 `Scenario -> Model` 映射。
  - 将默认策略（如 jamming sensitivity）转换到运行域可用形态。
- Runtime resolver
  - 只处理 `RuntimeConfigPatch`。
  - 对非法输入执行 reject 并保证“配置原子不变”（失败不半生效）。

### 6.1.7 AR 字段分组建议（便于横向复制）

为减少 EOS/ESR 迁移时的命名分歧，建议在 AR 先形成稳定字段分组约定：

- `atmospheric_*`：大气与空间天气事实。
- `vegetation_*`：地表植被散射场景。
- `jammer_*`：外部干扰源事实输入。
- `*_profile`：语义档位（离散策略），用于高层配置表达。
- `*_physics`：物理事实或可观测量，不用于表示内部调参。

### 6.1.8 AR 测试基线（建议新增/补齐）

AR 作为模板模块，应先具备完整基线测试，后续 EOS/ESR 直接照抄测试模式：

1. 类型与别名合同测试
- `config::RadarEnvironmentConfig` 与 `environment::EnvironmentDefaultConfig` 的等价性。

2. 默认值测试
- `EnvironmentDefaultConfig`、`EnvironmentScenarioConfig` 默认值稳定性。

3. 映射函数测试
- `BuildModelConfigFromScenario` 恒等行为测试。
- `ResolveEffectiveKFactor`：正常值、越界回退、缺省值路径。
- `ResolveEffectiveDayOfYear`：有时间戳与无时间戳路径。
- `ResolveJammingSensitivityProfile`：阈值分段边界测试。

4. runtime patch 行为测试
- 合法 patch 生效。
- 非法 patch reject 且原配置不变。

### 6.1.9 AR 文档化输出要求（作为模板资产）

AR 完成模板化后，建议新增/更新以下文档资产（为 EOS/ESR 迁移直接复用）：

- 环境配置类型关系图（Scenario/Model/Default/Patch）。
- 映射函数行为表（输入、输出、默认、回退）。
- Builder 与 Resolver 职责矩阵。
- 新增字段 checklist（“该字段属于 Scenario 还是 Model”判定流程）。

### 6.1.10 AR 执行步骤（仅 AR 范围）

1. 在 AR 头文件注释中补齐类型契约与函数回退语义描述。
2. 补齐 AR 映射与默认值测试，形成“模板级覆盖”。
3. 校验 AR resolver 是否严格遵守“映射单入口 + patch 白名单”。
4. 将 AR 最终结构固化为迁移样板，作为 EOS/ESR 改造前置门禁。

### 6.1.11 AR 到“纯单向架构”的剩余差距（关键补充）

尽管 AR 已是当前最佳基线，但距离严格意义的 `Scenario -> Model -> RuntimePatch` 纯单向架构仍有两项差距：

1. `ModelConfig` 与 `ScenarioConfig` 仍为同型别名
- 现状：`using EnvironmentModelConfig = EnvironmentScenarioConfig;`
- 问题：分层语义存在，但结构边界未硬化，调用方容易把两层当作同一层长期耦合。
- 目标：将 `EnvironmentModelConfig` 收敛为独立类型，哪怕初期字段与 `ScenarioConfig` 一致，也不再使用 type alias。

2. `DefaultConfig` 仍包含 `scenario_config` 之外的策略字段
- 现状：`EnvironmentDefaultConfig` 除 `scenario_config` 外还有 `jamming_sensitivity_profile`。
- 问题：初始化容器与策略控制耦合，弱化了单向分层。
- 目标：将 `DefaultConfig` 薄化为仅承载初始化场景输入；策略项迁移到明确层位（`Scenario` 语义字段或映射规则输入）并通过单入口映射落到 `Model`。

### 6.1.12 AR 纯化收敛计划（仅 AR，先于 EOS/ESR 复制）

为避免一次性重构风险，建议采用两步纯化：

1. 类型解耦（先做）
- 新建独立 `EnvironmentModelConfig` struct。
- `BuildModelConfigFromScenario` 改为显式字段映射（即使暂时 1:1）。
- 增加编译期与单测约束，禁止再次退化为 alias。

2. Default 薄化（后做）
- 审核 `jamming_sensitivity_profile` 的语义归属：
  - 若是场景事实表达，迁入 `ScenarioConfig`。
  - 若是映射策略控制，改为映射函数输入策略并在 resolver 明确赋值来源。
- 完成后令 `EnvironmentDefaultConfig` 仅保留 `scenario_config`（或等价薄包装）。

验收标准（AR 纯化完成判据）：

- `EnvironmentModelConfig` 不再是 `ScenarioConfig` 的 alias。
- `EnvironmentDefaultConfig` 不再承载额外策略字段。
- `BuildModelConfigFromScenario` 成为唯一语义入口且有边界测试覆盖。

## 6.2 EOS（重点改造）

目标定位：EOS 从“`config` 层承载环境主语义”迁移到“`environment` 层单源建模”，并对齐 AR 的单向架构约束。

### 6.2.1 EOS 现状评估（主要问题）

EOS 当前结构的核心问题：

- `environment/EosEnvironmentConfig.h` 只定义了 `model_type` 级别契约，环境事实输入承载不足。
- `config/EosEnvironmentConfig.h` 承载了 `preset/use_preset_defaults/radiative_transfer/aerosol/turbulence` 等主字段，形成“`config` 主导”。
- `runtime/EosPipelineConfigMapper.cpp` 内同时承担 preset 解释和环境参数映射，映射职责偏分散。
- `environment/EosEnvironmentRuntimeConfigPatch.h` 仅支持 `model_type` patch，而 `session/EosRuntimeConfigPatch` 支持整块环境覆盖，两层粒度不一致。

### 6.2.2 EOS 目录结构（环境配置相关）

以下是 EOS 环境配置相关的推荐目录视图（按“环境真源/会话聚合/实现消费”分层）：

```text
include/1q/electro_optical_sensor/
├── environment/
│   ├── EosEnvironmentConfig.h                    # 目标：Scenario/Model/Default 核心契约
│   ├── EosEnvironmentConfigBuilder.h             # 仅构造 DefaultConfig
│   ├── EosEnvironmentTypes.h                     # 环境模型输入输出类型（与 ModelConfig 对齐）
│   ├── EosEnvironmentRuntimeConfigPatch.h        # 环境域 runtime patch 契约
│   ├── EosEnvironmentRuntimeConfigPatchBuilder.h # 环境域 runtime patch builder
│   └── electro_optical_sensor_environment.hpp    # 环境域公开聚合头
└── config/
    ├── EosEnvironmentConfig.h                    # 目标：收敛为 alias
    ├── EosSessionConfig.h                        # 会话聚合（引用 alias 后的环境类型）
    └── EosRuntimeConfigPatch.h                   # 会话级 patch（含 environment patch 入口）

src/electro_optical_sensor/
├── runtime/
│   ├── EosPipelineConfigMapper.cpp               # 目标：只消费 ModelConfig，不再解释场景语义
│   └── EosRuntimeConfigResolver.cpp              # 运行期 patch 校验和应用
└── environment/
    ├── EosEnvironmentModel.h/.cpp                # 环境模型实现

tests/
├── unit/
│   └── eos_environment_model_test.cpp
└── contract/
    ├── check_public_api_boundary.cmake
    └── public_headers_smoke_test.cpp
```

目录约束说明：

- `environment/` 是 EOS 环境语义真源，`config/` 不再重复定义字段结构。
- `runtime/` 负责“消费模型参数”，不负责“定义场景语义”。
- 会话层 patch 入口可以存在，但环境字段解释权归 `environment` + resolver。

### 6.2.3 EOS 标准类型契约（目标形态）

建议 EOS 收敛为以下类型：

- `EosEnvironmentScenarioConfig`
  - 场景事实与语义输入，例如：`model_type`、`preset`、辐射传输语义、气溶胶/湍流观测语义、对抗扩展开关。
  - 禁止放入 pipeline 内部执行细节参数。
- `EosEnvironmentModelConfig`
  - pipeline 直接消费参数，例如最终 `radiative_transfer_model/aerosol_density_factor/turbulence_factor/...`。
  - 必须是独立 struct，不得继续使用 type alias 伪分层。
- `EosEnvironmentDefaultConfig`
  - 仅包含 `scenario_config`（薄包装）。
  - 不再承载与场景并列的额外策略字段。
- `EosEnvironmentRuntimeConfigPatch`
  - 仅包含运行时允许热更新的环境模型字段（白名单）。

### 6.2.4 EOS 映射函数契约（单入口）

EOS 应引入并强制使用：

- `BuildModelConfigFromScenario(const EosEnvironmentScenarioConfig&) -> EosEnvironmentModelConfig`

约束：

- `EosPipelineConfigMapper` 禁止直接读取 `ScenarioConfig`。
- preset 与自定义覆盖规则只在该函数内定义并测试固化。
- 当输入无效或缺失时，回退规则必须显式（默认值来源固定且可测）。

### 6.2.5 EOS Builder 与 Resolver 职责（模板规则）

- `EosEnvironmentConfigBuilder`
  - 只构造 `EosEnvironmentDefaultConfig`（或其 `scenario_config` 直接字段）。
  - 不做复杂合法性校验。
- Session resolver
  - 初始化阶段执行 `Scenario -> Model` 映射并产出 pipeline 可用配置。
- Runtime resolver
  - 仅处理 `EosEnvironmentRuntimeConfigPatch` 白名单字段。
  - 非法值 reject 且保持配置原子不变。

### 6.2.6 EOS 关键差距与纯化收敛

EOS 相对 AR 还需解决两类关键差距：

1. 真源层错位
- 现状：主环境字段在 `config/EosEnvironmentConfig.h`。
- 收敛：迁移到 `environment/EosEnvironmentConfig.h`，`config` 层改 alias。

2. preset 与 custom 混搭隐式优先级
- 现状：`preset + use_preset_defaults + 自定义字段` 并存，解释逻辑散落。
- 收敛：在 `BuildModelConfigFromScenario` 定义唯一优先级，并通过单测锁定。

推荐优先级（EOS）：

1. 先套用 `preset` 模板得到基线模型参数。
2. 再应用显式 custom override（若存在）。
3. 最后由 runtime patch 做运行中最小覆盖。

### 6.2.7 EOS 测试基线（对齐 AR 模板）

1. 合同测试
- `config::EosEnvironmentConfig` 是否 alias 到 `environment::EosEnvironmentDefaultConfig`。
- 公共头可编译性与 include 边界稳定。

2. 映射单测
- `BuildModelConfigFromScenario`：默认、各 preset、override、非法输入回退。

3. resolver 单测
- runtime patch 合法更新生效。
- 非法 patch reject 且状态不变。

4. 集成验证
- EosSession 初始化路径只走单入口映射，无二次语义解释。

### 6.2.8 EOS 执行步骤（仅 EOS）

1. 在 `environment` 新增 `Scenario/Model/Default` 并引入映射函数。
2. 将 `config/EosEnvironmentConfig.h` 收敛为 alias（保持会话层 API 聚合）。
3. 改造 `EosPipelineConfigMapper` 只消费 `ModelConfig`。
4. 收敛 `EosRuntimeConfigPatch` 与 `EosEnvironmentRuntimeConfigPatch` 的边界，统一白名单策略。
5. 完成合同/单元/集成测试后，删除冗余路径。

### 6.2.9 EOS 验收标准

- `environment` 成为 EOS 环境类型唯一真源。
- `config` 层不再定义独立环境字段结构。
- `Scenario -> Model` 映射入口唯一且可测试。
- preset/custom/runtime patch 优先级唯一且行为稳定。

## 6.3 ESR（结构收敛）

目标定位：ESR 基于现有 `environment` 能力快速收敛，消除 `config/environment` 并行结构，并作为 EOS 迁移前的“先行样板”。

### 6.3.1 ESR 现状评估（为何可先落地）

ESR 当前具备较好基础：

- `environment/EsrEnvironmentConfig.h` 已有 `EsrEnvironmentModelConfig + EsrEnvironmentDefaultConfig`。
- `environment/EsrEnvironmentRuntimeConfigPatch.h` 已具备环境 runtime patch 结构。
- resolver 已有较完整 patch 优先级处理逻辑（整域先、叶子后）。

当前主要问题：

- `config/EsrEnvironmentConfig.h` 仍定义并行环境结构，字段与 `environment` 重叠。
- `DefaultConfig` 当前承载 `model_config`，尚未对齐“Default 薄化 + Scenario 显式化”。
- `ApplyEnvironmentConfig(...)` 仍直接消费 `config::EsrEnvironmentConfig`，边界未完全收敛。

### 6.3.2 ESR 目录结构（环境配置相关）

```text
include/1q/electronic_surveillance_radar/
├── environment/
│   ├── EsrEnvironmentConfig.h                    # 目标：Scenario/Model/Default 核心契约
│   ├── EsrEnvironmentConfigBuilder.h             # DefaultConfig builder
│   ├── EsrEnvironmentRuntimeConfigPatch.h        # 环境 runtime patch 契约
│   ├── EsrEnvironmentRuntimeConfigPatchBuilder.h # 环境 runtime patch builder
│   ├── EsrEnvironmentTypes.h                     # 环境观察/类型契约
│   ├── EsrEnvironmentSceneBuilder.h              # 环境场景构造器
│   └── electronic_surveillance_radar_environment.hpp
└── config/
    ├── EsrEnvironmentConfig.h                    # 目标：收敛为 alias
    ├── EsrEnvironmentPreset.h                    # 可保留为公共语义枚举定义处
    ├── EsrSessionConfig.h                        # 会话聚合
    └── EsrRuntimeConfigPatch.h                   # 会话级 patch（引用环境 patch）

src/electronic_surveillance_radar/
├── session/
│   ├── EsrSessionConfigResolver.cpp              # 初始化映射与组装
│   └── EsrRuntimeConfigResolver.cpp              # 运行期 patch 解析
└── environment/
    └── EsrEnvironmentService.cpp                 # 环境服务消费 ModelConfig

tests/
├── unit/
│   └── esr_environment_service_test.cpp
└── contract/
    ├── check_public_api_boundary.cmake
    └── public_headers_smoke_test.cpp
```

目录约束说明：

- 环境字段定义权只保留在 `environment/EsrEnvironmentConfig.h`。
- `config/EsrEnvironmentPreset.h` 可继续作为跨层共享枚举，但配置 struct 不再双定义。
- 会话 resolver 只做映射与校验，不新增环境语义源。

### 6.3.3 ESR 标准类型契约（目标形态）

建议 ESR 最终形态：

- `EsrEnvironmentScenarioConfig`
  - `preset`
  - `atmospheric_physics`
  - `atmospheric_context`
- `EsrEnvironmentModelConfig`
  - 环境服务直接消费参数（可保留现有结构并按需扩展）。
- `EsrEnvironmentDefaultConfig`
  - 仅包含 `scenario_config`，不直接暴露 `model_config` 字段。
- `EsrEnvironmentRuntimeConfigPatch`
  - 白名单热更新：preset/atmospheric_physics/atmospheric_context（及未来明确允许项）。

### 6.3.4 ESR 映射函数契约（单入口）

ESR 应补齐并统一使用：

- `BuildModelConfigFromScenario(const EsrEnvironmentScenarioConfig&) -> EsrEnvironmentModelConfig`

约束：

- `EsrSessionConfigResolver` 初始化路径只调用该入口。
- `ApplyEnvironmentConfig(...)` 改为消费 `ScenarioConfig` 或 `DefaultConfig` 后统一映射，不再依赖 `config` 层并行 struct。
- `ResolveEffectiveKFactor/ResolveEffectiveDayOfYear` 作为推导边界函数，输入与回退规则显式化。

### 6.3.5 ESR Builder 与 Resolver 职责（模板规则）

- `EsrEnvironmentConfigBuilder`
  - 面向 `EsrEnvironmentDefaultConfig` 直接字段（即 `scenario_config`）。
  - 不操作 `ModelConfig` 内部执行态字段。
- `EsrSessionConfigResolver`
  - 负责初始化 `Scenario -> Model` 映射。
- `EsrRuntimeConfigResolver`
  - 负责 `RuntimePatch -> Model` 最小更新。
  - 保持“整域先、叶子后”的显式优先级策略。

### 6.3.6 ESR 关键差距与收敛策略

1. 双 struct 并行
- 现状：`config::EsrEnvironmentConfig` 与 `environment::EsrEnvironmentModelConfig/DefaultConfig` 并行。
- 收敛：移除 `config` struct 真源地位，改 alias。

2. Default 语义偏重
- 现状：`DefaultConfig` 持有 `model_config`，初始化与执行边界耦合。
- 收敛：引入 `ScenarioConfig`，`DefaultConfig` 改持有 `scenario_config`。

3. 会话层映射边界未统一
- 现状：resolver 中有对 `config` struct 的直接字段解释。
- 收敛：统一通过 `BuildModelConfigFromScenario`。

### 6.3.7 ESR 测试基线（对齐 AR 模板）

1. 合同测试
- `config::EsrEnvironmentConfig` alias 约束验证。

2. 映射单测
- `BuildModelConfigFromScenario`：默认、preset、大气上下文推导、边界回退。

3. resolver 单测
- patch 顺序（整域先、叶子后）行为不变。
- 非法 patch reject 且状态不变。

4. 服务级测试
- `EsrEnvironmentService` 消费 `ModelConfig` 的行为回归稳定。

### 6.3.8 ESR 执行步骤（仅 ESR）

1. 新增 `EsrEnvironmentScenarioConfig` 并定义映射函数。
2. `EsrEnvironmentDefaultConfig` 从持有 `model_config` 改为持有 `scenario_config`。
3. 改造 resolver 初始化路径统一走映射函数。
4. 将 `config/EsrEnvironmentConfig.h` 收敛为 alias 并修复调用点。
5. 完成映射/patch/服务回归测试后，清理并行结构。

### 6.3.9 ESR 验收标准

- ESR 不再存在 `config/environment` 并行环境 struct。
- `DefaultConfig`、`ScenarioConfig`、`ModelConfig` 分层清晰且职责稳定。
- 初始化与运行期都遵循单入口映射与 patch 白名单策略。

## 7. 文件与职责规划

建议按以下模式统一（示意）：

- `include/1q/<module>/environment/*EnvironmentConfig.h`
  - 声明 Scenario/Model/Default 与映射函数。
- `include/1q/<module>/environment/*EnvironmentConfigBuilder.h`
  - 仅构造 DefaultConfig 的直接字段。
- `include/1q/<module>/config/*EnvironmentConfig.h`
  - 仅别名。
- `src/<module>/session/*SessionConfigResolver.cpp`
  - 仅调 `BuildModelConfigFromScenario`，不再重复解释环境语义。

## 8. 数据流与调用关系

初始化路径：

1. 外部组装 `SessionConfig.environment`（本质为 `DefaultConfig`）。
2. resolver 提取 `scenario_config`。
3. 调 `BuildModelConfigFromScenario` 生成 `ModelConfig`。
4. environment service / pipeline 消费 `ModelConfig`。

运行期路径：

1. 接收 `RuntimeConfigPatch`。
2. resolver 校验并更新 `ModelConfig` 有限字段。
3. 服务热更新内部状态。

## 9. 测试策略

## 9.1 合同测试（public API）

- `config` 层环境类型为 alias 的可编译性。
- Builder API 可用性与默认值稳定性。

## 9.2 单元测试

- `BuildModelConfigFromScenario`：
  - 默认输入
  - preset 映射
  - custom override 优先级
  - 边界值/非法值回退
- runtime patch resolver：
  - 合法 patch 更新生效
  - 非法 patch 拒绝且配置不变

## 9.3 集成测试

- 典型会话初始化后环境模型行为一致。
- EOS/ESR 与 AR 在统一语义下的行为可预期（结构一致，不要求物理结果同值）。

## 10. 分阶段迁移计划

## Phase 0：冻结规则与评审（0.5~1 天）

- 输出本设计文档并评审确认。
- 明确 preset/override 语义约束。

交付物：

- 本文档 + 决策记录（ADR 可选）。

## Phase 1：引入新类型但不切流（1~2 天）

- EOS/ESR 在 `environment/` 新增 `ScenarioConfig` 与 `BuildModelConfigFromScenario`。
- 保留旧字段/旧路径。

交付物：

- 新头文件与最小实现。
- 映射函数单元测试初版。

## Phase 2：resolver/pipeline 切换到单入口（1~2 天）

- SessionConfigResolver 改为只走统一映射。
- 删除 pipeline 内环境语义二次解释代码。

交付物：

- resolver/pipeline 改造 PR。
- 回归测试通过。

## Phase 3：config 层收敛为 alias（1 天）

- EOS/ESR `config/*EnvironmentConfig.h` 改为别名。
- 修复调用点与 include 关系。

交付物：

- 对外头文件边界测试通过。

## Phase 4：清理与固化（0.5~1 天）

- 删除冗余结构和无用映射。
- 更新 README/示例。

交付物：

- 最终清理 PR。
- 设计与测试文档同步。

## 11. 风险与缓解

风险 1：API 迁移期间调用点爆炸

- 缓解：分阶段引入 alias 过渡，先切 resolver 再收敛类型。

风险 2：preset 行为变化引发回归

- 缓解：在映射函数级别建立 golden case 单测并锁定优先级。

风险 3：patch 可变边界不清导致线上行为波动

- 缓解：白名单化 patch 字段，resolver 严格 reject。

## 12. 完成定义（DoD）

以下条件同时满足视为完成：

- AR/EOS/ESR 三模块环境配置都满足统一分层模板。
- `config` 层不再维护独立环境 struct。
- `Scenario -> Model` 映射入口唯一且被 resolver/pipeline 一致使用。
- 新增/更新测试覆盖默认值、preset/override、patch reject 行为。
- 目标 preset 构建通过并相关测试通过。

## 13. 建议执行顺序（落地优先级）

1. 先改 ESR（结构更接近目标，收益快）。
2. 再改 EOS（重构幅度更大）。
3. 最后对 AR 做命名/注释规范化以形成模板样板。

这样可以先建立“可运行样板”，再处理最大改造面，降低整体不确定性。

## 14. EOS 实施差异标注（保留原文用于复核）

说明：本节仅记录**当前实际执行路径**与上文原始规划的差异；上文原文全部保留，不做替换，便于后期审计与回看。

### 14.1 差异总览（仅 EOS）

1. `config/EosEnvironmentConfig.h` 已收敛为 alias（与原规划一致）。

2. `environment/EosEnvironmentConfig.h` 采用了 `Scenario/Model/Default` 三层（与原规划一致），并引入：
  - `EosEnvironmentScenarioConfig`
  - `EosEnvironmentModelConfig`
  - `EosEnvironmentCustomOverrides`
  - `BuildModelConfigFromScenario(...)`

3. runtime patch 已按破坏式重构收敛：不再支持 `preset` 热更新，`EosEnvironmentRuntimeConfigPatch` 仅允许模型可变字段（`model_type` + 模型叶子 `has_*` 字段）。

4. `session::EosRuntimeConfigPatch` 保留 `has_environment` 入口，但内部语义已完全白名单化：不允许整块 `EnvironmentConfig` 覆盖，仅应用 `environment::EosEnvironmentRuntimeConfigPatch` 指定的可变字段。

5. trace 序列化/回放路径已按破坏式重构清理 legacy 兼容：不再读写 `use_preset_defaults`、不再解析 runtime patch 的 `has_preset/environment_preset`，统一使用新 schema（`has_custom_overrides/custom_overrides` 与 patch 叶子 `has_*` 字段）。

### 14.2 复核关注点（建议）

1. 是否将 `has_environment` 包装层进一步简化为“仅在叶子 `has_*` 任一为 true 时视为请求更新”，减少重复语义位。

2. 是否在回放工具中对缺失新 schema 关键字段（例如声明 `has_custom_overrides=true` 但缺失 `custom_overrides`）增加更严格的错误提示与失败策略。

### 14.3 变更判定原则

若后续复核发现实现与本节记录不一致，以“代码 + 单测行为”为准，并在本节追加修订记录（不覆盖历史差异项）。

## 15. ESR 实施差异标注（保留原文用于复核）

说明：本节仅记录**当前实际执行路径**与上文 ESR 原始规划的差异；上文原文全部保留，不做替换，便于后期审计与回看。

### 15.1 差异总览（仅 ESR）

1. `config/EsrEnvironmentConfig.h` 已收敛为 alias：
  - `using EsrEnvironmentConfig = environment::EsrEnvironmentDefaultConfig;`
  - `config` 层不再定义并行环境 struct 真源。

2. `environment/EsrEnvironmentConfig.h` 已采用 `Scenario/Model/Default` 三层，并引入：
  - `EsrEnvironmentScenarioConfig`
  - `EsrEnvironmentModelConfig`
  - `BuildModelConfigFromScenario(...)`

3. `EsrEnvironmentDefaultConfig` 已薄化为持有 `scenario_config`，不再持有 `model_config`。

4. `session::ResolveEsrSessionConfig` 初始化路径已切换为单入口映射：
  - 统一通过 `BuildModelConfigFromScenario(env_config.scenario_config)` 生成 `environment_model_config`。

5. `use_preset_defaults` 已从 ESR 配置链路移除：
  - 不再通过隐式总开关控制 preset 与详细字段覆盖。

6. runtime patch 已进一步收敛到“最小可变面”：
  - 禁止 `environment preset` 运行时热更新；
  - `environment::EsrEnvironmentRuntimeConfigPatch` 仅允许模型叶子字段（`atmospheric_physics` / `atmospheric_context`）生效；
  - 若提交 `has_preset=true`，resolver 显式 reject。

7. runtime patch reject 已提供结构化可观察反馈：
  - `EsrSession` 新增 `ApplyRuntimeConfigWithResult(...) -> EsrRuntimeConfigApplyResult`；
  - `TryApplyRuntimeConfig(...)` 与 `ApplyRuntimeConfig(...)` 继续保留，分别作为布尔与 void 语义入口。

8. ESR 相关测试已补齐并通过：
  - 新增合同测试 `esr_environment_config_contract_test.cpp`（alias 约束 + Scenario->Model 映射 + Default 持有 Scenario）。
  - 既有 resolver/integration/convenience 测试已迁移到 `environment.scenario_config` 路径并通过。

### 15.2 复核关注点（建议）

1. 是否需要进一步统一 AR/EOS/ESR 的 runtime reject 状态码命名与跨模块语义。

2. 是否将 `EsrRuntimeConfigApplyResult` 扩展为携带字段级 reject 详情（例如参数名/值），用于回放诊断。

### 15.3 变更判定原则

若后续复核发现实现与本节记录不一致，以“代码 + 单测行为”为准，并在本节追加修订记录（不覆盖历史差异项）。
