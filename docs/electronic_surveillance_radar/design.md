# Electronic Surveillance Radar 当前设计

Status: active
Last-reviewed: 2026-07-18
Authority: current electronic_surveillance_radar module design

本文是 `electronic_surveillance_radar` 当前设计权威。它描述 ESR 的会话门面、拦截 pipeline、观测预处理、聚类、辐射源假设关联、输出三通道和运行期状态边界。

## 1. 架构设计说明

### 1.1 模块定位

ESR 模块模拟电子侦察接收机对辐射源的观测和估计。它的核心不是“直接输出 truth emitter”，而是把输入辐射源场景、平台姿态、接收机配置和电磁环境转化为：

- observation output：设备观测记录。
- emitter output：系统估计的辐射源假设。
- truth evaluation output：仿真评估辅助。

当前设计目标：

- 对外提供稳定 `EsrSession` 门面和四域配置。
- runtime patch 经 resolver 校验后立即提交；配置不提供 session 层回滚。
- 保持真实侦察输出与仿真 truth evaluation 分离。
- 不暴露用户自定义 pipeline/controller/environment service。

### 1.2 Public API 与内部实现边界

公共头位于 `include/1q/electronic_surveillance_radar/`：

| 区域 | 职责 |
|---|---|
| `electronic_surveillance_radar.hpp` | 模块聚合入口 |
| `config/` | `EsrSessionConfig`、runtime patch、semantic builder、config validation |
| `session/` | `EsrSession`、cycle input/result、scene emitter、observation/hypothesis、adapter、trace/replay、debug/lifecycle |

内部实现位于 `src/electronic_surveillance_radar/`：

| 目录 | 职责 |
|---|---|
| `config/` | 内部执行配置 | `EsrInternalExecutionConfig` |
| `environment/` | `EsrEnvironmentService` 和 suppression/deception/atmospheric 环境采样 |
| `pipeline/` | 拦截 gate、检测执行、预处理、特征编码、Kd-tree 聚类、假设关联、后处理 |
| `runtime/` | `EsrController`、`EsrOutputManager` 和执行状态管理 |
| `session/` | session 组合根、配置解析、runtime patch、输入/输出适配、trace/replay |

### 1.3 新开发者视角的分层图

```mermaid
flowchart TB
  subgraph Public["Public API\n公共 API：include/1q/electronic_surveillance_radar"]
    Entry["electronic_surveillance_radar.hpp\n模块聚合入口"]
    Config["config/*\n四域配置 / RuntimePatch\nSemanticBuilder / Validation"]
    SessionApi["session/*\nEsrSession / CycleInput / CycleResult"]
    Types["EmitterObservation / EmitterHypothesis\n设备观测 / 辐射源假设"]
    Tools["Trace / Replay\nDebugView / Lifecycle"]
  end

  subgraph Session["Session orchestration\n会话编排层：src/electronic_surveillance_radar/session"]
    EsrSession["EsrSession\n电子侦察会话门面"]
    Composition["EsrSessionCompositionRoot\n默认内部装配"]
    Resolver["Session/Runtime resolvers\n配置解析与运行期 patch"]
    Snapshots["Runtime snapshots\npipeline/controller 累积状态快照"]
  end

  subgraph Runtime["Runtime layer\n运行期层"]
    Controller["EsrController\n单周期执行控制"]
    OutputManager["EsrOutputManager\n输出帧管理"]
    Environment["EsrEnvironmentService\n电磁环境采样"]
  end

  subgraph Pipeline["Intercept pipeline\n拦截流水线：src/electronic_surveillance_radar/pipeline"]
    Detect["InterceptDetectionExecutor\n检测执行"]
    Preprocess["ObservationPreprocessor\n排序 / 过滤 / 去重"]
    Cluster["KdTreeClusterer\n特征空间聚类"]
    Associate["HypothesisAssociator\n假设关联与轨迹状态"]
    Post["PostProcessingExecutor\n三通道输出组装"]
  end

  Entry --> Config
  Entry --> SessionApi
  Entry --> Types
  Config --> EsrSession
  SessionApi --> EsrSession
  EsrSession --> Composition
  Composition --> Controller
  Composition --> Environment
  Controller --> OutputManager
  Controller --> Pipeline
  EsrSession --> Resolver
  EsrSession --> Snapshots
  Detect --> Preprocess
  Preprocess --> Cluster
  Cluster --> Associate
  Associate --> Post
  Post --> OutputManager
  Tools -. "observe\n观测" .-> EsrSession
```

### 1.4 执行时序图

```mermaid
sequenceDiagram
  participant Caller as Caller 调用方
  participant Session as EsrSession ESR 会话
  participant Controller as EsrController 控制器
  participant Pipeline as InterceptPipeline 拦截流水线
  participant Env as EnvironmentService 环境服务
  participant Assoc as HypothesisAssociator 假设关联
  participant Result as EsrCycleResult 单周期结果

  Caller->>Session: StepWithResult(EsrCycleInput) 提交单周期输入
  Session->>Pipeline: CaptureRuntimeState 捕获 pipeline 状态
  Session->>Controller: CaptureRuntimeState 捕获 controller 状态
  Session->>Controller: RunOnce 执行单周期控制
  alt validation rejected 输入校验失败
    Controller-->>Session: not executed + validation rejection
  else validation accepted 输入校验通过
  Controller->>Pipeline: RunCycle 执行拦截流水线
  Pipeline->>Env: SampleEnvironment 采样电磁环境
  Pipeline->>Pipeline: detection / preprocess / cluster 检测、预处理、聚类
  Pipeline->>Assoc: Update clusters 更新辐射源假设
  Assoc-->>Pipeline: hypotheses 假设列表
  Pipeline-->>Controller: observation / emitter / truth outputs 三通道输出
  note over Pipeline,Controller: InterceptPipelineResult 是纯三通道数据载体；当前无 pipeline 自报失败状态
  end
  Session-->>Result: output frame + validation + abort reason
  Session-->>Caller: EsrCycleResult 返回结构化结果
```

### 1.5 数据流

主链路展示输入如何变成三通道输出：

```mermaid
flowchart LR
  subgraph Input["输入层 Input"]
    Config["EsrSessionConfig\n硬件 / 任务 / 策略 / 环境"]
    Cycle["EsrCycleInput\n平台 / 辐射源 / 外部环境输入"]
    Patch["EsrRuntimeConfigPatch\n运行期变更"]
  end

  subgraph Detect["检测层 Detection"]
    Env["EnvironmentSnapshot\nsuppression / deception / atmospheric"]
    Gate["InterceptGate\n范围 / 频段 / 扫描窗口 / SNR 门"]
    Angle["AngleErrorModel\nAOA 误差采样"]
    Raw["RawObservationRecord\n原始观测记录"]
  end

  subgraph Feature["特征层 Feature"]
    Pre["ObservationPreprocessor\n有限值过滤 / SNR 质量归一 / 去重"]
    Encode["ObservationFeatureEncoder\nRF/PW/AOA/SNR 特征缩放"]
    Cluster["KdTreeClusterer\n半径聚类 / 噪声标记"]
  end

  subgraph Track["关联层 Association"]
    Assoc["HypothesisAssociator\n门限距离匹配 / ID 稳定 / miss 回收"]
    Hyp["EmitterHypothesis\n模式 / 威胁 / 置信度 / candidate classes"]
  end

  subgraph Output["输出层 Output"]
    Obs["observation_output\n设备观测"]
    Emit["emitter_output\n系统估计"]
    Truth["truth_evaluation_output\n仿真评估"]
    Result["EsrCycleResult\n执行状态 / 诊断 / replay"]
  end

  Config --> Gate
  Patch --> Gate
  Cycle --> Gate
  Env --> Gate
  Gate --> Angle
  Angle --> Raw
  Raw --> Pre
  Pre --> Encode
  Encode --> Cluster
  Cluster --> Assoc
  Assoc --> Hyp
  Raw --> Obs
  Hyp --> Emit
  Hyp --> Truth
  Obs --> Result
  Emit --> Result
  Truth --> Result
```

输出三通道必须保持语义分离：

```mermaid
flowchart TB
  InputTruth["Scene emitters\n仿真输入辐射源"] --> Detection["Detection pipeline\n设备观测生成"]
  Detection --> Observation["Observation output\n真实侦察观测"]
  Detection --> Association["Association\n观测到假设"]
  Association --> Emitter["Emitter output\n系统估计假设"]
  InputTruth -. "evaluation only\n仅评估使用" .-> Truth["Truth evaluation output\n仿真评估通道"]
  Association --> Truth
```

## 2. 本模块使用的算法

### 2.1 算法总览

| 算法/部件 | 入口 | 当前角色 | Public 默认 |
|---|---|---|---|
| 环境采样 | `EsrEnvironmentService::SampleEnvironment` | suppression/deception/atmospheric 环境快照 | session 内部 |
| 扫描窗口 | `ScanPatternGenerator` | 根据扫描模式和运行期配置生成接收窗口 | pipeline 内部 |
| 拦截门控 | `InterceptGate` | range、receiver window、dynamic range、SNR 等 joint constraints | pipeline 内部 |
| 边界搜索 | `BoundarySearchSolver` | 单调谓词边界查找 | pipeline 内部 |
| 角误差 | `AngleErrorModel` | 基于 SNR/系数/随机种子的 AOA 扰动 | pipeline 内部 |
| 干扰聚合 | `JammingAggregator` | suppression/deception 通道分离和混合技术 fallback | pipeline 内部 |
| 观测预处理 | `ObservationPreprocessor` | 排序、有限值过滤、质量归一、窗口去重 | pipeline 内部 |
| 特征编码 | `ObservationFeatureEncoder` | RF/PW/AOA/SNR 按尺度编码到特征空间 | pipeline 内部 |
| 聚类 | `KdTreeClusterer` | 半径聚类、min-points、noise/border point 处理 | pipeline 内部 |
| 假设关联 | `HypothesisAssociator` | cluster 到 track 的 gated matching、ID 稳定、miss 回收 | pipeline 内部 |
| 输出管理 | `EsrOutputManager` | 输出帧 stamp、空帧和上一帧复用 | runtime 内部 |

### 2.2 拦截检测

`InterceptDetectionExecutor` 将场景 emitter 转成 raw observation。关键步骤包括：

1. 计算平台到 emitter 的距离和接收机参考系方位/俯仰。
2. 应用天线安装偏置。
3. 计算 emitter beam overlap；历史默认 beam state 退化为全覆盖。
4. 使用自由空间路径损耗和综合接收损耗估计接收功率。
5. 启用大气物理时，以 `platform_altitude_m` 作为接收端大气高度参考；更高高度通过气压、密度与
   Blake 衰减改变传播附加损耗，而不是只进入 trace。
6. 以 `1 + 9 * spectrum_occupancy_ratio` 放大接收机底噪与杂波底噪：0 严格退化，1 对应
   +10 dB 环境噪声；占用率不参与主动 jammer 聚合，也不单独置 `is_jammed`。
7. 结合统计检测参数、pulse count、integration mode、PFA 和 threshold scale 判断是否可检测。
8. 将 suppression/deception 影响写入观测质量或 false/confused observation 可能性。

限制：

- detection 输出是设备观测，不直接等同于 truth emitter。
- 随机量必须受 config seed 和 runtime snapshot 管理；snapshot 只覆盖累积运行态，不覆盖配置。

验证入口：

- `tests/unit/esr_algorithms_test.cpp`
- `tests/integration/esr_session_test.cpp`

[evidence: tests/integration/electronic_surveillance_radar/esr_session_test.cpp::AltitudeAndSpectrumOccupancyAffectReceiverSnr]
[evidence: tests/replay/electronic_surveillance_radar/esr_replay_codec_roundtrip_test.cpp::CycleInputPreservesAllFields]

### 2.3 观测预处理

`ObservationPreprocessor` 对 raw observation 做三件事：

- 按 timestamp 和 observation id 排序。
- 丢弃非有限值、非法 RF、非法 pulse width。
- 在时间、RF、pulse width、azimuth、elevation 窗口内去重，保留 SNR 更高或 observation id 更小的记录。

质量归一规则：

- `snr_db >= 18` → high。
- `snr_db >= 10` → medium。
- 否则 low。

验证入口：

- `tests/unit/esr_kdtree_clusterer_test.cpp`

### 2.4 特征编码与聚类

聚类前会把 observation 转为特征向量，尺度来自 `InterceptClusterConfig`：

- RF scale。
- pulse width scale。
- azimuth/elevation scale。
- SNR scale。

`KdTreeClusterer` 使用半径和 min-points 判断 cluster/noise，并处理 border point。该层目标是把同一 emitter 的多条观测归为候选簇，不直接生成最终 emitter hypothesis。

验证入口：

- `tests/unit/esr_kdtree_clusterer_test.cpp`

### 2.5 假设关联

`HypothesisAssociator` 维护内部 track state。每周期：

1. 计算 cluster centroid feature 到现有 track feature 的距离。
2. 距离小于 `gate_distance` 的 cluster-track pair 进入候选集。
3. 候选按距离、cluster index、track index 排序，固定并列距离顺序，避免边界抖动。
4. 匹配 track 使用 `confidence_alpha` blending 更新 feature、bearing、mode、threat、confidence。
5. 未匹配 cluster 创建新 hypothesis id。
6. 未命中 track 累计 missed cycles，超过阈值后回收。

模式和威胁推断：

- pulse width、PRI 和 SNR 推断 search/tracking/guidance。
- guidance 或高 SNR 提升 threat level。
- deception support 会降低 confidence，并添加 ambiguous/deception candidate class。

验证入口：

- `tests/unit/esr_hypothesis_associator_test.cpp`

### 2.6 运行期配置与状态边界

环境域以 `EsrEnvironmentScenarioConfig` 作为唯一公开 DTO 权威，环境服务直接消费该类型。当前运行路径
没有 execution-only 字段，因此不得维护同型公开 Model 类型或恒等 mapper；只有先证明存在执行态专属
字段时，才可新增内部执行配置和显式映射。
[evidence: tests/contract/electronic_surveillance_radar/esr_environment_config_contract_test.cpp::EsrEnvironmentConfigContractTest.DefaultConfigOwnsScenarioConfig]

`scan_rate_hz` 的单位不是波束更新率或角速度，而是**每秒完成的完整二维 scan pattern 循环数**。
pipeline 持有归一化扫描相位 `[0, 1)`：本周期先用 `floor(phase × pattern_size)` 选择波束，再累加
`scan_rate_hz × dt` 并回绕。因此变步长不会改变物理扫描速度；运行期只改速率会保留当前相位，改变
窗口边界、顺序或起始位置则重置到起始波束。设备关闭时扫描相位冻结。静态配置和 runtime patch 均拒绝
非有限或非正速率。
[evidence: tests/unit/electronic_surveillance_radar/esr_controller_runtime_state_test.cpp]
[evidence: tests/unit/electronic_surveillance_radar/esr_session_config_builder_test.cpp]

该标量 pipeline 每个 `Step` 只判定当前相位对应的一个波束，不在单周期内积分连续扫过的全部驻留。
因此 `scan_rate_hz × dt` 为整数时，相位会按物理周期回到同一点；需要观察完整扫描覆盖的场景必须选择
能够解析扫描相位的步长/速率组合，不能依赖 cycle index 隐式轮转波束。
[evidence: tests/integration/cross_domain/multi_model_scenario_test.cpp::MultiModelScenarioTest.AirToAirHeadOn]

扫描窗口有两种互斥解释方式：

- `use_explicit_scan_bounds=true` 时，四个显式起止角必须全部有限，并分别满足
  `scan_start_az_deg < scan_end_az_deg`、`scan_start_el_deg < scan_end_el_deg`；显式边界生效，中心角字段被忽略。
- `use_explicit_scan_bounds=false` 时，中心角结合硬件扫描范围推导窗口；即使显式字段为 NaN/Inf，也因未被选择而忽略。

静态 session validation 按所选模式验证，不能把非法的显式模式静默退化成中心模式。运行期 patch
提交中心角时关闭显式模式，提交显式起止角时开启显式模式，因此最近一次被明确选择的表达拥有窗口语义。
[evidence: tests/unit/electronic_surveillance_radar/esr_session_config_builder_test.cpp]
[evidence: tests/unit/electronic_surveillance_radar/esr_runtime_config_resolver_test.cpp]

`ApplyRuntimeConfigWithResult()` 通过 `ResolveEsrRuntimeConfigPatch()` 校验 patch。有效 patch 立即写入 `resolved_config`，并同步到 pipeline/environment；无效 patch 拒绝且不污染现有配置。ESR 属于 `docs/common/contract.md` 定义的立即提交类，配置单向落定，不提供 session 层回滚。

`InterceptPipeline::RunCycle()` 返回 `InterceptPipelineResult`，该结果只包含 observation、emitter、truth evaluation 三通道输出，不包含 `executed_this_cycle` 或 `abort_reason`。因此 ESR controller 当前没有 EOS 式的 pipeline 自报失败状态可校验；一旦输入校验通过并进入 pipeline，空输出也是合法数据结果。

pipeline/controller 的 `CaptureRuntimeState()` / `RestoreRuntimeState()` 只描述累积运行态能力：

- pipeline 快照含 RNG、observation/hypothesis id、hypothesis associator tracks 和归一化扫描相位。
- pipeline 快照不含 config、feature scales 或环境配置。
- controller 快照含 latest output、validation issues、batch id 和最近一次执行状态。

设计含义：

- validation rejection 在进入 pipeline 前发生，`EsrCycleResult` 记录 `executed_this_cycle=false` 和 `kValidationRejected`。
- 当前没有“pipeline 执行后自报失败并触发回滚”的路径；不要把 `kOutputContractViolation` 理解为 ESR 输出契约校验的预留占位。
- 新增 ESR pipeline 失败语义前，必须先扩展 `InterceptPipelineResult` 或等价内部结果结构，使失败状态成为显式接口契约。

验证入口：

- `tests/unit/esr_controller_runtime_state_test.cpp`
- `tests/unit/esr_runtime_config_resolver_test.cpp`
- `tests/integration/esr_session_test.cpp`

### 2.7 输出三通道与可观测性

ESR 输出保持三通道：

- `observation_output`：设备观测。
- `emitter_output`：系统估计 hypothesis。
- `truth_evaluation_output`：仿真评估。

名称字段和 truth identity 不进入真实输出通道。需要人读映射时通过 debug view 或 truth evaluation 关联回填。

验证入口：

- `tests/unit/esr_output_boundary_contract_test.cpp`
- `tests/unit/esr_cycle_output_builder_test.cpp`
- `tests/consumer/esr_output_observability_consumer.cpp`
- `tests/unit/esr_replay_codec_roundtrip_test.cpp`

## 3. 非目标与边界

- 不暴露用户自定义 pipeline/controller/environment service。
- 不把 truth evaluation 合并进真实 observation/hypothesis 输出。
- 不把 pipeline internal context、runtime snapshot 或 generated replay header 当成 public API。
- 不通过日志文本判断状态；调用方应使用 `EsrCycleResult`。

## 4. 设计变更规则

1. Observation/hypothesis 字段变化必须同步 replay roundtrip 测试。
2. Gate、preprocess、cluster、association 语义变化必须同步本文和对应 focused tests。
3. Runtime patch、snapshot 或状态边界变化必须同步控制器状态测试。
4. 输出通道边界变化必须同步 output boundary contract 测试。
