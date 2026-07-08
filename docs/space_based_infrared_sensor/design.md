# Space-Based Infrared Sensor (SBIRS-inspired) 目标设计

Status: active
Last-reviewed: 2026-07-07
Authority: target design for the new `sbirs_sensor` module

本文是 `sbirs_sensor`（天基红外预警仿真传感器）模块的设计权威文档。模块已实现并具备单元、
集成与契约测试分层；本文描述目标架构、数据流和算法边界。跨模块 public API、builder、三层输出
等共同规则见 `docs/common/contract.md`。

本文以公开 SBIRS / OPIR 资料中的扫描红外传感器与 step-staring/staring 红外传感器为真实系统校准点，
但不声称复刻真实 SBIRS 设备、保密载荷或地面处理链路。本文中的 WFOV / NFOV 是面向仿真实现的
宽域搜索 / 窄域凝视抽象：WFOV 对应扫描搜索能力，NFOV 对应可任务化凝视与高灵敏度区域覆盖能力。


## 1. 架构设计说明

### 1.1 模块定位

`sbirs_sensor` 提供天基红外预警仿真传感器的配置、单周期输入、环境与大气建模、扫描搜索发现、
凝视捕获/跟踪、搜索→凝视交接（cueing & handover）、多目标状态管理、融合输出、trace/replay、
调试视图和生命周期事件。

与 EOS 的核心差异在于**扫描搜索 + 凝视资源调度 + 状态机驱动交接**：EOS 是单视场扫描探测器，对
FOV 内目标做一次性 SNR 判定；SBIRS-inspired 模型用 WFOV 宽域扫描发现目标，首次进入 NFOV 时用
WFOV 带误差 cue 生成凝视指向并做捕获判定，捕获成功后进入仿真简化的持续跟踪，捕获失败回退 WFOV
继续搜索。

它不是一个通用图像处理或航迹滤波框架。当前模块的稳定外部使用方式是：

1. 使用 `SbirsSessionConfig` 或 semantic builder 描述硬件、任务、策略、环境四域配置。
2. 使用 `SbirsCycleInput` 或 input adapter 提供平台（卫星）姿态、环境和目标场景。
3. 调用 `SbirsSession::Step()` 获取 1q 仿真传感器主输出，或调用 `SbirsSession::StepWithResult()`
   获取结构化执行结果。
4. 通过 runtime patch 调整工作模式、扫描速率、检测阈值、NFOV 资源策略和环境模型等有限运行期参数。

当前模块不提供 public pipeline/controller/environment-service/state-machine 自定义点。外部如果
需要改变物理行为，应通过配置和输入表达；如果需要排查仿真归属，应消费 `SbirsCycleResult`、
debug view、lifecycle 或 replay，而不是把仿真真值塞进 raw output。

### 1.2 Public API 与内部实现边界

公共头位于 `include/1q/sbirs_sensor/`，命名空间 `sbirs_sensor`，public 类型前缀 `Sbirs*`：

| 区域 | 职责 | 设计约束 |
|---|---|---|
| `sbirs_sensor.hpp` | 模块聚合入口 | 只聚合稳定 public API，不暴露 foundation/pipeline/runtime/state-machine 内部类型 |
| `config/` | `SbirsSessionConfig`、runtime patch、semantic builder、config validation | 表达硬件（扫描搜索/凝视传感器，仿真参数名保留 WFOV/NFOV）、任务、策略、环境四域语义 |
| `session/` | `SbirsSession`、cycle input/result、scene target、output types、adapter、trace/replay、debug/lifecycle | 是外部调用方的主要使用面 |

内部实现位于 `src/sbirs_sensor/`：

| 目录 | 职责 | 典型类型/函数 |
|---|---|---|
| `config/` | 内部执行配置 | `SbirsInternalExecutionConfig` |
| `foundation/` | 光学、传播、辐射、噪声、空间频谱基础算法（参照 EOS foundation 复制改名） | `ComputePlanckRadiance`、`EvaluateRadiativeTransfer`、`ComputeBackgroundNoiseStatistics` |
| `environment/` | 环境模型与气象衰减 | `ResolveEnvironmentFactors`、`ResolveWeatherAttenuation` |
| `pipeline/` | WFOV 扫描通道、NFOV 跟踪通道、交接与状态机调度 | `SbirsPipeline`、`SbirsTargetStateMachine`、`SbirsNfovScheduler` |
| `runtime/` | controller、config mapper、runtime config resolver | `SbirsController`、`MapSessionToInternal`、`ResolveSbirsRuntimeConfigPatch` |
| `session/` | public session 装配、输入输出适配、trace/replay、debug/lifecycle | `SbirsSessionCompositionRoot`、`SbirsCycleOutputAdapter` |

SBIRS 不在公开头文件中暴露 `electro_optical_sensor`、`Eos*` 或 `1q/electro_optical_sensor/...`。
EOS 仅作为 foundation 物理算法的迁移参考，不构成 SBIRS 的对外集成依赖。

### 1.3 与 EOS 的关系：派生 + 独立

SBIRS 的 foundation 物理算法（Planck 辐射、Beer-Lambert 传播、光子/热/读出噪声、SNR 合成）
参照 EOS foundation 层复制并改为 `sbirs_sensor` 命名空间和 `Sbirs*` 类型。这些算法是内部可测试
实现，不是 public 契约，也不是 public customization surface。

但 pipeline、controller、session 三层**全部独立实现**，不复用 EOS 代码：

- EOS pipeline 是单视场扫描 + 单次 SNR 判定（`EosPipeline::RunCycle`，
  `src/electro_optical_sensor/pipeline/EosPipeline.cpp:387-426`）。
- SBIRS pipeline 是双视场 + 状态机调度：WFOV 通道发现目标、状态机决策交接、NFOV 通道做首次捕获
  或真值辅助跟踪。状态机是跨周期累积状态，由 controller 做 capture/restore（见 1.5、2.2）。

### 1.4 分层组件图

```mermaid
flowchart TB
  subgraph Public["Public API / 公共调用面"]
    Entry["sbirs_sensor.hpp\n聚合稳定入口"]
    Config["config/*\n扫描搜索/凝视传感器 / 任务 / 策略 / 环境配置\nRuntimePatch / Builder / Validation"]
    SessionApi["session/*\nSbirsSession / CycleInput / CycleResult\nOutputFrame / SceneTarget"]
    Tools["Trace / Replay / Debug / Lifecycle\n追踪 / 回放 / 调试 / 生命周期"]
  end

  subgraph Session["Session orchestration / 会话编排层"]
    SbirsSession["SbirsSession\n外部门面：Step / StepWithResult / RuntimePatch"]
    Composition["SbirsSessionCompositionRoot\n默认依赖图装配"]
    InputAdapters["Input adapters\n外部输入到 SbirsCycleInput"]
    OutputAdapters["Output adapters\nOutputFrame / Result / DebugView"]
    ReplayCodec["Replay codec\nFlatBuffer 追踪与回放"]
  end

  subgraph Runtime["Runtime control / 运行期控制层"]
    Controller["SbirsController\n校验输入 / 执行周期 / 状态机 capture-restore / 缓存输出"]
    Mapper["SbirsPipelineConfigMapper\nSessionConfig 到内部执行配置"]
    Resolver["SbirsRuntimeConfigResolver\nPatch 校验与立即生效"]
  end

  subgraph Pipeline["Detection pipeline / 探测流水线"]
    FrameCtx["FrameContext\n帧级光学 / 环境 / 噪声 / 卫星几何上下文"]
    Occult["Earth-occultation gate\n地球遮挡与大气边界门控"]
    Wfov["WFOV channel\n宽视场扫描发现 + 带误差位置"]
    StateMachine["SbirsTargetStateMachine\n目标级状态机（5 状态）"]
    Handoff["Handoff decision\n首次捕获判定"]
    NfovFirst["NFOV first acquisition\n用 WFOV 带误差位置捕获"]
    NfovTrack["NFOV truth-assisted tracking\n仿真真值辅助持续跟踪"]
    Scheduler["SbirsNfovScheduler\n单目标锁定资源调度"]
  end

  subgraph Foundation["Foundation algorithms / 基础物理算法（参照 EOS）"]
    Env["environment\n气象影响 + 环境因子"]
    Optics["optics\n孔径 / FOV / 衍射 / GSD"]
    Transfer["radiative transfer\n路径透过率 / 路径辐射惩罚"]
    Radiometry["radiometry\nPlanck / Lambertian / contrast"]
    Noise["noise / NEP\n背景噪声 / 等效噪声 / 有效信号"]
  end

  Entry --> Config
  Entry --> SessionApi
  Config --> SbirsSession
  SessionApi --> SbirsSession
  Tools -. "observe / consume\n观测与消费" .-> ReplayCodec
  SbirsSession --> Composition
  Composition --> Controller
  Composition --> Mapper
  Controller --> Resolver
  Controller --> FrameCtx
  Mapper --> FrameCtx
  Resolver --> FrameCtx
  InputAdapters --> SbirsSession
  FrameCtx --> Occult
  Occult --> Wfov
  Wfov --> StateMachine
  StateMachine --> Handoff
  Handoff --> NfovFirst
  Handoff --> NfovTrack
  Scheduler --> Handoff
  NfovFirst --> OutputAdapters
  NfovTrack --> OutputAdapters
  Wfov --> OutputAdapters
  OutputAdapters --> SbirsSession
  ReplayCodec -. "record/replay\n记录与回放" .-> SbirsSession
  FrameCtx --> Env
  FrameCtx --> Optics
  Wfov --> Transfer
  Wfov --> Radiometry
  Wfov --> Noise
  NfovFirst --> Transfer
  NfovTrack --> Radiometry
  NfovTrack --> Noise
```

读图顺序：

1. 外部只从 Public API 进入，不直接构造 `SbirsPipeline`、`SbirsTargetStateMachine` 或 foundation 类型。
2. `SbirsSessionCompositionRoot` 负责默认依赖图；当前没有用户替换 controller、pipeline、状态机或环境模型的 public API。
3. `SbirsController` 处理输入校验、运行期状态（含状态机 capture/restore）、失败输出复用和周期执行。
4. `SbirsPipeline` 把一个周期拆成帧级上下文、地球遮挡门控、WFOV 发现、状态机决策、NFOV 首次捕获或真值辅助跟踪。
5. foundation 算法参照 EOS 复制改名，是内部可测试实现，不是模块间契约。

### 1.5 执行时序图

```mermaid
sequenceDiagram
  participant Caller as Caller / 调用方
  participant Session as SbirsSession / 会话门面
  participant Controller as SbirsController / 周期控制器
  participant Validator as Validation / 输入校验
  participant Pipeline as SbirsPipeline / 探测流水线
  participant SM as TargetStateMachine / 目标状态机
  participant Physics as Foundation / 物理算法
  participant Output as OutputAdapter / 输出适配

  Caller->>Session: StepWithResult(input)\n提交单周期输入
  Session->>Controller: RunOnce(input)\n执行一个周期
  Controller->>Controller: snapshot state-machine\n快照状态机用于失败回滚
  Controller->>Validator: ValidateSbirsCycleInput(input)\n校验平台 / 环境 / 目标
  alt invalid input / 输入无效
    Validator-->>Controller: issues\n错误列表
    Controller->>Output: reuse latest output if available\n复用最近有效输出
    Output-->>Session: SbirsCycleResult with validation status\n携带校验状态的结果
  else valid input / 输入有效
    Controller->>Pipeline: RunCycle(input)\n进入探测流水线
    Pipeline->>Physics: build FrameContext\n构造帧级光学 / 噪声 / 卫星几何上下文
    Pipeline->>Physics: earth-occultation gate\n地球遮挡与大气边界过滤
    loop each target / 每个目标
      Pipeline->>Physics: WFOV scan: range/FOV/radiometry/noise/SNR\n带误差位置
      Pipeline->>SM: update target state\n更新目标状态
      alt first acquisition candidate / 首次捕获候选
        SM-->>Pipeline: AwaitingNfovAcquisition\n等待 NFOV 首次捕获
        Pipeline->>Physics: NFOV acquisition: window + SNR gate\n捕获窗口与门限判定
        alt acquisition success / 捕获成功
          SM-->>Pipeline: TruthAssistedTracking\n真值辅助跟踪
          Pipeline->>Physics: NFOV truth-assisted track\n真值辅助持续跟踪
        else acquisition fail / 捕获失败
          SM-->>Pipeline: back to WideCandidate\n回退 WFOV
        end
      end
    end
    Pipeline-->>Controller: detections + attribution\n检测记录与仿真归属
    Controller->>Output: BuildCycleResult(input)\n生成结构化结果
    Output-->>Session: OutputFrame + diagnostics\n系统输出与诊断
  end
  Session-->>Caller: SbirsCycleResult\n返回结果

  Caller->>Session: TryApplyRuntimeConfig(patch)\n提交运行期变更
  Session->>Controller: ResolveSbirsRuntimeConfigPatch\n校验 patch
  Controller->>Pipeline: ApplyInternalConfig (immediate)\n立即生效，不在 session 层回滚
```

运行期配置采用**立即提交**策略（与 EOS 同类），见 `docs/common/contract.md` 运行期配置提交策略表。
状态机 capture/restore 是 controller 内部失败回滚机制，不上升为 session 层事务契约。

### 1.6 主探测数据流

```mermaid
flowchart LR
  subgraph Input["Input / 输入"]
    Config["SbirsSessionConfig\nSearch/Stare hardware abstraction / Mission / Policy / Environment"]
    Cycle["SbirsCycleInput\n卫星姿态 / 环境快照 / 目标列表"]
    Patch["SbirsRuntimeConfigPatch\n工作模式 / 扫描速率 / 门限 / NFOV 策略"]
  end

  subgraph Runtime["Runtime mapping / 运行期映射"]
    Internal["SbirsInternalExecutionConfig\n内部执行配置"]
    Frame["FrameContext\n帧级上下文（含卫星轨道几何）"]
  end

  subgraph Geo["Geometric gating / 几何门控"]
    Occult["Earth occultation\n地球遮挡角判别"]
    WfovGate["WFOV FOV gate\n宽视场门控"]
    RangeGate["Range gate\nDmin / Dmax"]
  end

  subgraph Wfov["WFOV discovery / 宽视场发现"]
    Wsnr["WFOV IR SNR\nPlanck / 透过率 / 噪声"]
    Werr["Error-bearing position\n带误差方位/俯仰/距离"]
    Weather["Weather attenuation\n气象衰减 A_total"]
  end

  subgraph State["State machine / 状态机决策"]
    SM["Target state\n5 状态机"]
    Handoff["Handoff\n首次捕获判定"]
    Sched["NFOV scheduler\n单目标锁定调度"]
  end

  subgraph Nfov["NFOV channel / 窄视场通道"]
    Acq["First acquisition\nWFOV 带误差位置 + 窗口 + 门限"]
    Track["Truth-assisted tracking\n真值辅助持续跟踪"]
  end

  subgraph Output["Output / 输出"]
    Raw["SbirsOutputFrame\n1q 仿真传感器检测记录"]
    Result["SbirsCycleResult\n状态 / attribution / debug source"]
    Trace["Trace / Replay\n可回放输入输出"]
  end

  Config --> Internal
  Patch --> Internal
  Cycle --> Frame
  Internal --> Frame
  Frame --> Occult
  Occult --> WfovGate
  WfovGate --> RangeGate
  RangeGate --> Weather
  Weather --> Wsnr
  Wsnr --> Werr
  Werr --> SM
  SM --> Handoff
  Sched --> Handoff
  Handoff --> Acq
  Handoff --> Track
  Acq --> Raw
  Track --> Raw
  Wsnr --> Raw
  Raw --> Result
  Result --> Trace
```

## 2. 本模块使用的算法

### 2.1 算法总览

SBIRS 第一版的 public 可调面限定为 config、cycle input、runtime patch 和 debug/replay 消费面。
下表中的算法均为 internal 实现；除 `SbirsSession`、配置、输入输出 DTO、trace/replay/debug/lifecycle
工具外，不形成 public customization surface。

| 算法/部件 | 入口 | 当前角色 | Public 默认 | 主要测试锚点（计划） |
|---|---|---|---|---|
| 配置到内部执行映射 | `MapSessionToInternal` | 将硬件、任务、策略、环境四域配置映射为 WFOV/NFOV 可执行参数 | internal mapper，不暴露 | `sbirs_input_validation_test` |
| runtime patch 立即提交 | `ResolveSbirsRuntimeConfigPatch`、`SbirsSession::TryApplyRuntimeConfig` | 校验工作模式、扫描速率、阈值、NFOV 策略和环境模型变更 | public 只提交 patch，不替换 resolver | `sbirs_runtime_config_resolver_test`、`sbirs_session_test` |
| 环境与气象衰减 | `ResolveEnvironmentFactors`、`ResolveWeatherAttenuation` | 将场景/大气观测映射为透过率衰减因子 | internal 环境模型，不提供环境 service SPI | `sbirs_environment_model_test` |
| 地球遮挡门控 | `EvaluateEarthOccultation` | 用有限 LOS 线段与地球球体相交判别穿地视线 | internal 几何门控，不进入 raw output | `sbirs_earth_occultation_test` |
| WFOV 扫描搜索 | `SbirsPipeline` | 推进扫描相位，执行地球遮挡、FOV、范围和 SNR 门控 | internal pipeline，不可替换 | `sbirs_pipeline_test` |
| WFOV 误差模型 | `ApplyAngularErrorModel` | 对方位/俯仰/距离生成带误差 cue，供首次 NFOV 捕获使用 | internal 随机源可注入，public 不直接采样 | `sbirs_error_model_test` |
| 目标状态机 | `SbirsTargetStateMachine` | 5 状态管理 WFOV 候选、首次捕获和真值辅助跟踪 | internal 状态机，debug view 可观测 | `sbirs_state_machine_test` |
| NFOV 首次捕获 | `EvaluateNfovAcquisition` | 由 WFOV cue 生成凝视指向，真实 LOS 落入窗口且 NFOV SNR 达标时捕获 | internal 判定，不暴露捕获算法 SPI | `sbirs_pipeline_test` |
| NFOV 资源调度 | `SbirsNfovScheduler::Select` | 单目标锁定，按已跟踪、SNR、距离、target id 排序 | internal scheduler，不暴露策略 SPI | `sbirs_scheduler_test` |
| 辐射传输与 SNR | `ComputePlanckRadiance`、`EvaluateRadiativeTransfer`、`ComputeInfraredSnrLinear` | 计算红外辐射、透过率、噪声和可探测性 | internal foundation，可测试但不可定制 | `sbirs_foundation_test`、`sbirs_radiative_transfer_test` |
| 输出构造与仿真归属 | `SbirsCycleOutputAdapter` | 生成 1q 仿真传感器主输出、结构化 result、debug/lifecycle/replay | public 只消费 DTO，不混入 truth | `sbirs_cycle_output_builder_test` |

### 2.2 核心状态机与 WFOV→NFOV 交接

本章是 SBIRS-inspired 模型区别于 EOS 的核心。EOS 对 FOV 内目标做一次性 SNR 判定；本模块用跨周期
状态机管理每个目标的 WFOV 发现、NFOV 首次捕获和真值辅助跟踪全过程。

#### 2.2.1 目标状态机（6 状态版）

每个目标独立维护一个状态机实例，以 `target_id` 为键。状态枚举（解决 `docs/review/sbirs_filter.md` 两套状态机
不一致问题，见第 5 节修正 #2）：

| 状态 | 含义 | 该状态下本周期输出 |
|---|---|---|
| `Undetected` | 初始或目标未被任何视场发现 | 不输出 |
| `WideCandidate` | WFOV 已发现，等待 NFOV 资源调度 | 输出 WFOV 检测记录 |
| `AwaitingNfovAcquisition` | 已被调度器选为首次捕获目标，本周期执行 NFOV 首次捕获 | 视捕获结果 |
| `TruthAssistedTracking` | 首次 NFOV 捕获成功，进入仿真简化的真值辅助持续跟踪（默认跟踪态） | 输出 NFOV 检测记录 |
| `EstimatedTracking` | EKF 滤波测量跟踪（默认启用；由 `SbirsTrackingConfig.enable_estimated_tracking` 控制） | 输出 NFOV 检测记录（滤波估计角度） |
| `Lost` | 目标从输入场景消失或传感器关闭 | 不输出 |

捕获失败不是独立状态；失败转移回 `WideCandidate`，并清除本次交接上下文。

状态转移：

```mermaid
stateDiagram-v2
  [*] --> Undetected
  Undetected --> WideCandidate : WFOV FOV 门控通过\n且 WFOV SNR ≥ 门限
  WideCandidate --> AwaitingNfovAcquisition : 调度器选中\n（优先级最高候选）
  AwaitingNfovAcquisition --> TruthAssistedTracking : 首次捕获成功\n（默认：未启用滤波）\n（真实 LOS 落入 cue 指向窗口\n且 NFOV SNR ≥ 门限）
  AwaitingNfovAcquisition --> EstimatedTracking : 首次捕获成功\n（启用滤波）\n（预留态，当前未接线）
  AwaitingNfovAcquisition --> WideCandidate : 首次捕获失败\n（清除交接状态）
  TruthAssistedTracking --> TruthAssistedTracking : 目标仍存在且传感器开启\n（仿真真值辅助跟踪）
  EstimatedTracking --> EstimatedTracking : 目标仍存在且传感器开启\n（滤波测量跟踪，预留）
  WideCandidate --> WideCandidate : 下一周期仍是 WFOV 候选
  Undetected --> Undetected : 目标在 WFOV 外\n或 SNR 不足
  TruthAssistedTracking --> Lost : 目标从场景消失\n或传感器关闭
  EstimatedTracking --> Lost : 目标从场景消失\n或传感器关闭
  WideCandidate --> Lost : 目标从场景消失
  AwaitingNfovAcquisition --> Lost : 目标从场景消失
  Lost --> [*]
```

转移条件表（补充状态图中的判定细节）：

| 起点 → 终点 | 触发条件 | 周期内副作用 |
|---|---|---|
| `Undetected` → `WideCandidate` | 目标在本周期 WFOV 视场内，且 WFOV IR SNR ≥ WFOV 检测门限 | 记录 WFOV 带误差位置、SNR |
| `WideCandidate` → `AwaitingNfovAcquisition` | NFOV 资源空闲且该目标在优先级排序中胜出（见 2.6） | 标记本周期为首次捕获目标 |
| `AwaitingNfovAcquisition` → `TruthAssistedTracking` | 由 WFOV 带误差 cue 生成的 NFOV 指向窗口覆盖目标真实 LOS，且 NFOV IR SNR ≥ NFOV 捕获门限（默认：未启用滤波） | 记录 NFOV 检测，进入真值辅助跟踪 |
| `AwaitingNfovAcquisition` → `EstimatedTracking` | 同上捕获成功条件，且配置启用滤波测量跟踪（预留态，当前未接线） | 记录 NFOV 检测，进入滤波测量跟踪（初始化滤波状态） |
| `AwaitingNfovAcquisition` → `WideCandidate` | 首次捕获条件不满足（窗口外或 SNR 不足） | 清除交接状态，目标回候选池 |
| `TruthAssistedTracking` → `TruthAssistedTracking` | 目标仍存在于输入场景且传感器开启 | 用仿真真值辅助生成 NFOV 指向与检测输出 |
| `EstimatedTracking` → `EstimatedTracking` | 目标仍存在于输入场景且传感器开启（预留态，当前未接线） | 用滤波估计生成 NFOV 指向与检测输出 |
| 任意 → `Lost` | 目标从输入场景消失，或传感器关闭 | 释放 NFOV 资源 |

设计要点：

- 首次 NFOV 捕获**必须**使用 WFOV 输出的带误差位置，不得直接用真值位置（`docs/review/sbirs_filter.md:121` 假设）。
- 捕获成功后进入哪个跟踪态由配置决定：默认（未启用滤波）进入 `TruthAssistedTracking`，用仿真真值辅助
  生成指向；启用滤波时进入 `EstimatedTracking`，用滤波估计生成指向。两个跟踪态**严格分离**，不复用同一
  状态枚举——这满足原先"后续若引入估计滤波必须先把真值辅助状态拆分"的前置约束。
- 真值辅助跟踪**只**受"目标是否存在"和"传感器是否开启"影响，不受后续测量误差影响。这是第一版的仿真简化，
  不是真实 SBIRS 设备行为：真实传感器只能产生测量、事件和辐射数据，不知道目标真值。
- `EstimatedTracking` 是预留态，当前未接线（不实现 EKF/CKF 滤波调用、R 矩阵构造、滤波状态 snapshot）。
  滤波 facade 骨架已建立于 `sbirs_sensor::tracking` 命名空间（`src/sbirs_sensor/tracking/`），下一步接入 EKF 时
  在此填充实例化别名与角度量测模型。当前捕获成功仍走 `TruthAssistedTracking`，行为零变化。
- 状态机是跨周期累积状态。`SbirsController` 在执行前 snapshot、失败时 restore（见 1.5 时序图）。
  这是 SBIRS controller 的内部失败回滚约束，不是 session 层事务接口，也不要求与其他模块输出或
  controller 形状保持一致。

适用边界：

- 状态机只管理目标级发现、首次捕获、真值辅助跟踪和消失，不负责图像检测、滤波估计或多假设关联。
- 状态机输入来自 pipeline 判定结果；外部调用方不能直接推进状态，也不能替换状态机策略。
- debug view 可以暴露状态机状态，但 raw output 不携带状态枚举。

验证入口：

- `sbirs_state_machine_test`
- `sbirs_scheduler_test`
- `sbirs_session_test`

### 2.3 WFOV 宽视场多目标搜索

每周期对输入场景中的所有目标执行 WFOV 扫描判断：

1. **几何门控**：目标先过地球遮挡门控（2.7）、WFOV FOV 门控、范围门控。WFOV FOV 门控参照 EOS
   `IsTargetInCurrentFov`（`src/electro_optical_sensor/pipeline/EosPipeline.cpp:443-451`）：
   `|az − scan_az| ≤ 0.5 × wide_field_fov_az` 且 `|el − scan_center_el| ≤ 0.5 × wide_field_fov_el`。
2. **扫描相位推进**：每周期按 `scan_rate_deg_per_sec × dt` 推进 WFOV 扫描方位角，对
   `[start_az, end_az]` 取模回绕，参照 EOS `AdvanceScan`
   （`src/electro_optical_sensor/pipeline/EosPipeline.cpp:428-441`）。角度归一化必须用 `std::fmod`
   常数时间实现（`docs/common/contract.md` 实现安全规则 5）。
3. **SNR 计算**：对门控通过的目标计算 WFOV IR SNR（见 2.8）。气象衰减 `A_total`（2.9）作用于
   路径透过率，进入 SNR。
4. **带误差位置**：满足 WFOV 检测门限的目标，输出带误差的方位角、俯仰角、距离（见 2.10 误差模型）。
   带误差位置是后续 NFOV 首次捕获的输入。
5. **多目标**：内部以 `target_id` 建立候选状态表，每个目标独立维护状态。同一周期允许多个 WFOV
   候选目标同时存在。

适用边界：

- WFOV 搜索只处理输入场景中显式给出的目标列表，不从图像像素中生成新目标。
- WFOV 带误差位置是仿真观测/cue，不是目标真值，也不是外部 target identity。
- 扫描相位、FOV 和范围门控属于 pipeline 内部状态；public 只能通过配置和 runtime patch 影响它们。

验证入口：

- `sbirs_pipeline_test`
- `sbirs_error_model_test`

### 2.4 NFOV 首次捕获

对状态机进入 `AwaitingNfovAcquisition` 的目标，本周期执行首次捕获：

1. **输入**：使用该目标 WFOV 输出的**带误差位置**（方位角、俯仰角、距离），不得使用真值位置。
2. **指向生成**：由 WFOV 带误差位置生成 NFOV 命令指向 `u_cmd`。第一版不模拟 ATP 姿态机动过程，
   但保留 `narrow_pointing_settle_error_deg` / `narrow_cue_latency_s` 等配置位，默认可为 0。
3. **窗口判定**：判断目标真实 LOS `u_true` 是否落入以 `u_cmd` 为中心的 NFOV 搜索窗口。`u_true` 在
   `narrow_cue_latency_s > 0` 且目标提供 `velocity_ecef_m_per_s` 时，按延迟时间对真值位置做线性外推
   后重算——即 cue 指向测量瞬间的位置，而目标在延迟期间继续运动，从而建模 cue 延迟与目标运动的
   耦合对首次捕获的影响。无速度时 `u_true` 即当前帧真值，行为不变。这样捕获判定仍受 WFOV 误差、
   目标运动、cue 延迟和 NFOV 视场大小影响，不会因为窗口中心直接取测量值而恒成立。
4. **SNR 门限**：判断 NFOV IR SNR 是否 ≥ NFOV 捕获门限。NFOV 门限通常高于 WFOV（窄视场虚警率
   要求更低，对应 `k=5~6`，见原始需求 `docs/review/sbirs_infrared_model_1205_v3.md:137-142` 的 k 值表）。
5. **成功**：进入 `TruthAssistedTracking`。后续周期不再使用 WFOV 带误差位置重新捕获。
6. **失败**：清除该目标本次交接状态，回退 `WideCandidate`，等待后续周期重新发现和交接。不输出该
   目标本周期 NFOV 成功记录；但产出 `capture_failure_reason = kNfovAcquisitionFailed` 的诊断归属，
   仅进入 `SbirsCycleResult.detection_attributions` 与调试/lifecycle 层，不进入 raw output。

适用边界：

- NFOV 首次捕获只做几何窗口 + SNR 门限判定，不做模板匹配、图像相关或目标运动外推。
- `u_cmd` 由 WFOV cue 生成；真实 LOS 只用于仿真判定捕获是否成功，不进入 raw output。
- cue 延迟外推仅用目标速度对真值位置做线性平移，不做积分轨道传播或 ATP 动力学建模。
- cue 延迟和指向 settle error 是配置参数，不代表完整 ATP 动力学模型。

验证入口：

- `sbirs_pipeline_test`
- `sbirs_error_model_test`

### 2.5 NFOV 真值辅助持续跟踪

对 `TruthAssistedTracking` 状态的目标，后续周期持续跟踪：

1. **指向来源**：使用目标**真实位置**（输入场景中的真值方位角、俯仰角、距离）辅助计算 NFOV 指向和
   检测输出，不再使用 WFOV 带误差位置。这是仿真层稳定性假设，不代表真实传感器知道目标真值。
2. **持续条件**：只要目标仍存在于输入场景，且传感器开启，就认为 NFOV 能持续捕获/跟踪该目标。
   不因测量误差丢失锁定。
3. **输出**：按现有红外传感器检测记录格式生成（角度、距离、SNR、是否探测成功）。可在输出测量值
   上叠加误差，但误差不影响内部真值辅助状态——即输出层的误差是"显示噪声"，不是"状态转移输入"。
4. **释放**：目标从输入场景消失后，状态转为 `Lost`，NFOV 释放资源，回到 WFOV 中选择下一个候选。

适用边界：

- 真值辅助跟踪是第一版仿真稳定性假设，不是 OPIR/SBIRS 真实跟踪算法。
- 该阶段（`TruthAssistedTracking`）不实现 EKF/CKF、波门关联、轨迹平滑或丢锁概率模型。
- 跟踪态已拆分为 `TruthAssistedTracking`（真值辅助，默认）与 `EstimatedTracking`（滤波测量跟踪，预留态
  当前未接线）。原"后续若引入估计滤波，必须先把本状态重命名或拆分"的前置约束现已满足：两个跟踪态
  严格分离。`EstimatedTracking` 的滤波接线（EKF 量测模型、R 矩阵、predict/update、滤波状态 snapshot）在
  后续阶段，当前捕获成功仍走 `TruthAssistedTracking`，行为零变化。

验证入口：

- `sbirs_state_machine_test`
- `sbirs_pipeline_test`

### 2.5a NFOV EKF 滤波测量跟踪

对 `kEstimatedTracking` 状态的目标（`enable_estimated_tracking=true` 时捕获成功进入），后续周期用
扩展卡尔曼滤波（EKF）做测量跟踪。滤波框架消费 `common/estimation`（`oneq::common::estimation`），
SBIRS 侧 facade 位于 `sbirs_sensor::tracking`（`src/sbirs_sensor/tracking/SbirsTrackingTypes.h`）。

**状态空间**：6 维 ECEF 恒速模型 `[x, vx, y, vy, z, vz]`（CV 交错布局），复用 common 的
`KalmanPredictor::BuildTransitionMatrix` / `BuildProcessNoise`。

**量测模型**（非线性，球坐标角度）：2 维 `[az, el]`（弧度），被动红外不测距（design 2.11 不含 range）。
`SbirsAngleMeasurementModel` 实现 `IMeasurementModel<6,2>`：
- `h(x)` = 目标 ECEF 位置（状态偶数索引 0/2/4）相对卫星位置 `satellite_position` 的 LOS →
  `[atan2(dy,dx), asin(dz/r)]`。卫星位置非状态分量，每帧由 pipeline 通过 `SetSatellitePosition` 注入。
- Jacobian `H = ∂[az,el]/∂[x,vx,y,vy,z,vz]` 解析求导，速度列为零。

**初始化**（首次捕获成功时，方案 A）：状态均值用输入场景真值 ECEF 位置 + 速度（无速度时速度置 0），
初始协方差 P0 由 `SbirsTrackingConfig.initial_position_std_m` / `initial_velocity_std_m_per_s` 构造为
对角阵（位置/速度交替）。这是仿真的 track initiation 简化（等效"首次检测无误差"）；后续 predict/update
用带误差测量才是滤波器发挥作用的环节。

**每周期 predict-update**（`kEstimatedTracking` 目标）：
1. `SetSatellitePosition(input.satellite_position_ecef_m)`。
2. EKF predict（CV 转移模型，过程噪声 `process_noise_diff_coeff`）。
3. 测量 = 本帧带误差角度（`ApplyAngularErrorModel` 输出的 az/el，deg → rad）。
4. R 矩阵 = `BuildMeasurementCovariance(error_model, range, elevation, angular_rate)`，从 design 2.10 的
   5 类误差 1-σ（orbit/attitude/fov 高斯 + 折射 + 滞后）RSS 合成，deg² → rad²。R 随距离/俯仰/角速度动态变化。
5. EKF update（3 参数重载，动态 R）。

**输出处理**（design 2.5 第 3 点精神：滤波发散不影响可探测性）：
- **SNR / range / 检测门限**：用真值几何算（与真值辅助一致），**不受滤波估计影响**。滤波器发散不会
  导致"虚假丢失"——目标红外辐射特性是物理事实。
- **输出角度**（`azimuth_deg` / `elevation_deg`）：用滤波估计的 ECEF 位置 → 相对卫星 LOS → az/el。
  比单次带误差测量更平滑。`used_truth_assist = false`。
- **状态转移**：仍只受"目标是否存在"和"传感器是否开启"影响（与真值辅助一致）。

**snapshot / replay**：滤波状态（`filter_states_` map：target_id → `SbirsGaussianState` 均值+协方差）
进 `SbirsPipelineSnapshot`，随 controller capture/restore 同步。EKF 本身确定性（无额外随机源采样），
测量噪声采样复用 `SbirsRandomSource`（已在 snapshot 的 `random_state`），故 replay 确定性保持。

适用边界：

- `kEstimatedTracking` 与 `kTruthAssistedTracking` 严格分离（不同枚举值），不复用同一状态。
  `enable_estimated_tracking=false` 时回退真值辅助，零滤波开销。
- EKF 初始化用真值位置（方案 A），后续 update 用带误差测量；不实现测量初始化（反算 ECEF 需距离假设）。
- SNR / 可探测性用真值链路；滤波器只影响指向与输出角度。滤波发散不丢锁。
- 过程噪声为 CV 模型白噪声加速度（标量 q），不建模机动目标加速度跳变。
- 量测噪声 R 从 design 2.10 误差模型合成，az/el 通道对称（各向同性假设）。

配置（`SbirsTrackingConfig`，挂 `SbirsPolicyConfig.tracking`）：
- `enable_estimated_tracking`（默认 true）：是否启用 EKF 跟踪。
- `process_noise_diff_coeff`（默认 1.0）：CV 过程噪声扩散系数。
- `initial_position_std_m`（默认 1000）、`initial_velocity_std_m_per_s`（默认 100）：P0 构造。

验证入口：

- `sbirs_state_machine_test`（默认走 kEstimatedTracking；显式关闭回退 kTruthAssistedTracking）
- `sbirs_pipeline_test`（捕获后 used_truth_assist=false；持续跟踪输出角度有限）

### 2.6 多目标优先级与 NFOV 资源调度

第一版 NFOV 资源采用**单目标锁定策略**：任一时刻至多一个目标处于 `AwaitingNfovAcquisition`、
`TruthAssistedTracking` 或 `EstimatedTracking`。

优先级默认规则（调度器在多个 WFOV 候选中选目标进入首次捕获）：

1. 已真值辅助跟踪目标优先级最高（持续占用 NFOV 资源，直到目标消失）。
2. 新候选按 WFOV IR SNR 从高到低。
3. SNR 相同按距离从近到远。
4. 仍相同按 `target_id` 从小到大。

已锁定目标消失后，NFOV 释放资源，调度器在剩余 `WideCandidate` 中按上述规则选下一个。

适用边界：

- 第一版调度器只支持单 NFOV 资源；多凝视资源、多区域同时重访和任务化排程不在当前范围。
- 优先级排序必须稳定，避免相同输入在 replay 中产生不同捕获目标。
- 调度器不读取仿真目标名称，只使用状态、SNR、距离和 `target_id`。

验证入口：

- `sbirs_scheduler_test`
- `sbirs_replay_codec_roundtrip_test`

### 2.7 地球遮挡与几何门控

天基传感器视线穿过地球时目标不可观测，这是 `docs/review/sbirs_filter.md` 缺失、但原始需求明确要求的天基必备
几何门控（`docs/review/sbirs_infrared_model_1205_v3.md:864-880`）。

**遮挡角计算**：

- 卫星到地心距离 `R_sat = ||r_sat(t)||`。
- 地球半径 `R_E = 6371 km`。
- 地球遮挡角 `θ_occ = arcsin(R_E / R_sat)`。

**遮挡判定**：对任意目标视线方向 `u_LOS`，计算卫星-地心-视线夹角

```
φ = arccos( (r_sat · (R_sat · u_LOS)) / R_sat² )
```

上述遮挡角公式适合解释地球圆盘半角。实现时使用有限线段射线-地球球体判定，避免方向符号误用：

```
p = r_sat
u = unit(target_position - satellite_position)
range = ||target_position - satellite_position||
s_closest = -dot(p, u)
d_closest² = dot(p, p) - s_closest²
occulted = (0 < s_closest && s_closest < range && d_closest² <= R_E²)
```

其中 `u` 是从卫星指向目标的单位 LOS。只有最近点位于卫星到目标的有限线段内，且该线段穿过地球球体
时，目标才被地球遮挡。若只做无限射线近似，也必须要求 `dot(p, u) < 0`，即视线朝向地球一侧。

**大气边界过滤**：设定大气顶层高度 `H_atm`（如 100 km）。对目标高度 `h < H_atm` 的区域，考虑
大气红外吸收（通过透过率阈值过滤）。第一版的气象衰减模型（2.9）已覆盖大气透过率，大气边界过滤
主要作为几何前置门控：目标位于大气层以下且距离过远时，直接判为不可观测。

地球遮挡判定在 WFOV FOV 门控和范围门控**之前**执行，避免对穿地视线做无意义的 SNR 计算。该门控
是帧级（目标无关）与目标级（目标相关）的混合：遮挡角 `θ_occ` 只依赖卫星位置（帧级），夹角 `φ`
依赖目标视线方向（目标级）。

适用边界：

- 遮挡门控只回答 LOS 是否穿过地球球体，不负责地形、云图、临边散射或三维大气廓线。
- 大气边界过滤是 SNR 前置 gate；更精细的大气吸收仍由 2.9 的透过率链路承担。
- 实现必须使用一致的 ECEF/ECI 坐标输入，不能混用局部 FOV 坐标做地球相交判定。

验证入口：

- `sbirs_earth_occultation_test`
- `sbirs_pipeline_test`

### 2.8 Foundation 物理链路

以下 foundation 算法参照 EOS foundation 层复制并改为 `sbirs_sensor` 命名空间，原始公式见
`docs/review/sbirs_infrared_model_1205_v3.md`：

| 算法 | EOS 参照 | 原始需求公式 | 说明 |
|---|---|---|---|
| Planck 辐射 | `ComputePlanckRadiance`（`src/electro_optical_sensor/foundation/EosRadiometry.cpp`） | `docs/review/sbirs_infrared_model_1205_v3.md:208-214`（斯特藩-玻尔兹曼简化） | 目标谱辐射，Stefan-Boltzmann 常数使用 `σ≈5.670374419e-8 W/(m²·K⁴)`；原始需求中的 `5.76e-8` 视为近似/笔误 |
| 大气衰减（Beer-Lambert） | `EvaluateRadiativeTransfer`（`EosRadiativeTransfer.cpp`） | `docs/review/sbirs_infrared_model_1205_v3.md:216-222` | `Φ_atm = Φ_tar · τ(λ,d)`，`τ` 依赖波段和距离 |
| 探测器接收功率 | `ComputeReceivedPowerW` | `docs/review/sbirs_infrared_model_1205_v3.md:224-230` | `P_sig = Φ_atm · A_det · η_det / d²` |
| 噪声模型 | `ComputeBackgroundNoiseStatistics`（`EosNoiseModel.cpp`） | `docs/review/sbirs_infrared_model_1205_v3.md:232-240` | 光子噪声、热噪声、读出噪声均方根合成。实现见 `SbirsNoiseModel`：`background_radiance_w_sr_m2`/`detector_temperature_k`/`readout_noise_rms_w` 三项 RMS 合成，三项默认全 0 时回退到 `noise_equivalent_power_w` 标量 |
| SNR 与可探测性 | `ComputeInfraredSnrLinear` | `docs/review/sbirs_infrared_model_1205_v3.md:242-246` | `SNR = P_sig · t_int / N_total`，`SNR ≥ SNR_th` 可探测 |
| 方位角/俯仰角计算 | EOS pipeline 内部 | `docs/review/sbirs_infrared_model_1205_v3.md:148-186` | `El = RADTODEG(-arcsin(XLOS_z/LOSRange))`，`Az = RADTODEG(arctan2(XLOS_y, XLOS_x))` |
| 探测阈值调整 | EOS policy 映射 | `docs/review/sbirs_infrared_model_1205_v3.md:121-142` | `T = μ + k·σ`，k 值按虚警率选取（WFOV k≈4，NFOV k≈5~6） |

这些算法是内部可测试实现，不是 public 契约。复制时改命名空间为 `sbirs_sensor`，类型前缀改
`Sbirs*`，并允许为天基红外场景修正物理常数、波段参数、背景项和几何门控。若与 EOS 逻辑不等价，
必须在测试名和本文变更记录中说明差异来源。

适用边界：

- foundation 算法可被单元测试直接覆盖，但不作为 public header、SPI 或 runtime plugin 暴露。
- 第一版只实现波段、透过率、接收功率、噪声和门限的标量链路，不实现图像帧、像元级背景图或多色分类器。
- 与 EOS 复制来的算法允许按天基场景修正常数和几何输入，但必须保持调用面由 `SbirsPipeline` 统一编排。

验证入口：

- `sbirs_foundation_test`
- `sbirs_noise_model_test`
- `sbirs_radiative_transfer_test`

### 2.9 气象衰减模型

气象影响是 SBIRS SNR 链路的必要组成，原始需求 `docs/review/sbirs_infrared_model_1205_v3.md:35-103` 有完整定义。

**气象影响列表**（查表得各参数独立衰减比例 `A_i`）：

| 气象参数 | 独立衰减比例 |
|---|---|
| 海浪等级 | 低 5% / 中 10% / 高 15% |
| 天气类型 | 晴 0% / 多云 5% / 雨 15% / 雾 20% |
| 温度 | 每升高 10℃ 衰减减少 2% |
| 湿度 | 每增加 20% 衰减增加 5% |
| 能见度 | >10km 0% / 5-10km 5% / 1-5km 10% / <1km 20% |

**加权叠加公式**（`docs/review/sbirs_infrared_model_1205_v3.md:87-103`）：

```
A_total = Σ(w_i · A_i) + Σ(k_j · A_p · A_q) + C
```

其中 `w_i` 为参数权重（`Σw_i = 1`），`k_j · A_p · A_q` 为参数交互项（如湿度与能见度联合影响），
`C` 为常数修正项。`A_total ∈ [0, 1]`，1 表示完全衰减。

> 实现状态：第一版固定权重实现独立项 `Σ(w_i · A_i)` 与温度修正；交互项 `k_j · A_p · A_q`
> 通过 `humidity_visibility_interaction_weight`（湿度×能见度）与 `rain_humidity_interaction_weight`
> （雨×湿度，仅雨天）两个可配置系数启用，默认 0 即关闭交互项（向后兼容）。

**进入 SNR 链路的方式**：`A_total` 作用于路径透过率，即有效透过率

```
τ_eff(λ,d) = τ(λ,d) · (1 − A_total)
```

`τ_eff` 替换 2.8 中的 `τ(λ,d)` 进入 `Φ_atm = Φ_tar · τ_eff`，进而降低 `P_sig` 和 SNR。这样气象
衰减与 Beer-Lambert 大气衰减统一在透过率维度合成，避免在多个环节重复扣减。

适用边界：

- 气象模型只输出透过率衰减因子，不直接改写 detection threshold、目标温度或输出记录。
- `A_total` 必须夹紧到 `[0, 1]`，并且只能在透过率维度扣减一次。
- 第一版使用查表和加权叠加，不接入 MODTRAN/LOWTRAN 或三维天气场。

验证入口：

- `sbirs_environment_model_test`
- `sbirs_radiative_transfer_test`

### 2.10 误差模型（WFOV 带误差位置）

WFOV 输出的带误差位置是 NFOV 首次捕获的输入，误差模型直接影响首次捕获成功率。原始需求
`docs/review/sbirs_infrared_model_1205_v3.md:1052-1115` 定义 5 类误差：

| 误差类型 | 物理成因 | 建模方法 |
|---|---|---|
| 卫星轨道误差 | 轨道预报摄动（太阳辐射压、大气阻力） | 高斯分布 + 协方差传播，`Δr_sat ~ N(0, Σ_orb)` |
| 卫星姿态误差 | 姿态传感器噪声（陀螺漂移、星敏误差） | 高斯分布 + 一阶马尔可夫，`Δα,Δβ ~ N(0, σ_att²)`，典型 `σ_att≈0.01°` |
| 探测器视场误差 | 像元错位、光学畸变 | 随机偏移 + 系统偏差，`Δθ_FOV = Δθ_rand + Δθ_sys` |
| 大气折射误差 | 大气密度梯度导致的光线偏折 | 标准大气模型，红外波段 `Δθ_refr = 1.5e-6 / (d·cosβ)`，β 为目标俯仰角 |
| 动态滞后误差 | 探测器响应延迟（高速目标运动快） | 一阶系统滞后，`Δθ_lag = ω_tar / (2π·f_det)`，`ω_tar` 为目标角速度，`f_det` 为探测器带宽 |

**加法合成**（角度误差）到真值方位角/俯仰角（`docs/review/sbirs_infrared_model_1205_v3.md:1111-1113`）：

```
α_meas = α_true + Δα_orb + Δα_att + Δα_refr + Δα_lag
β_meas = β_true + Δβ_orb + Δβ_att + Δβ_refr + Δβ_lag
```

**乘法合成**（距离误差）：`d_meas = d_true · (1 + Δd_rand)`，典型 `Δd_rand ≈ 0.1%`。

误差叠加作用于 WFOV 输出层、NFOV cue 指向生成和 NFOV 首次捕获判定层；真值辅助跟踪阶段（2.5）
不受后续测量误差影响。高斯随机误差的采样应使用可注入的随机数源，保证 replay 可复现。

> 实现状态：`SbirsErrorModel` 实现 5 类误差的加法/乘法合成。轨道/姿态/视场为高斯随机
> （`orbit_sigma_deg`/`attitude_sigma_deg`/`fov_sigma_deg`，Box-Muller），折射与滞后为确定性公式
> （`RefractionErrorDeg`/`DynamicLagErrorDeg`）。随机源 `SbirsRandomSource` 为 xorshift32 + Box-Muller，
> 由 `random_seed` 初始化，状态经 `SbirsPipelineSnapshot::random_state` 随 capture/restore 持久化，
> 保证 replay 可复现。当三项高斯 sigma 均为 0 时回退到合并 `angular_sigma_deg`（向后兼容）。
> 目标角速度由 `SbirsSceneTarget.velocity_ecef_m_per_s`（ECEF 速度真值，可选）推导：pipeline 调用
> `ComputeRelativeAngularRateDegPerSec(los, velocity)` 得到视线角速度 `ω_tar`，接入动态滞后项。
> 未提供速度（`has_velocity_ecef_m_per_s=false`）时 `ω_tar=0`，动态滞后项为 0，保持旧行为。当前无
> 卫星速度输入，相对速度按目标速度处理；后续接入卫星运动估计时再扩展 `SbirsCycleInput`。

适用边界：

- 误差模型生成的是观测/cue 误差，不改变输入目标真值。
- 随机源必须可注入、可 snapshot 或可由 replay 固定，避免同一 trace 回放产生不同捕获结果。
- 距离误差只用于内部 cue/诊断链路；不能由此推导真实单星被动红外具备直接测距能力，也不得进入
  `SbirsOutputFrame` raw output。

验证入口：

- `sbirs_error_model_test`
- `sbirs_replay_codec_roundtrip_test`

### 2.11 输出与仿真归属

SBIRS 遵守三层输出模型（`docs/common/contract.md` 三层输出模型表）：

| 层级 | 入口 | 责任 |
|---|---|---|
| 原始系统输出层 | `Step()` 返回的 `SbirsOutputFrame` | 1q 仿真传感器主输出 |
| 结构化执行结果层 | `StepWithResult()` 返回的 `SbirsCycleResult` | 输出帧、执行状态、校验、abort reason、诊断摘要 |
| 开发调试视图层 | `SbirsOutputDebugViewBuilder` / `SbirsDetectionLifecycleRecorder` | 人读状态、生命周期事件、输入实体回填 |

**`SbirsOutputFrame` 字段**（第一版使用原生 SBIRS-inspired 观测契约，不继承
EOS 检测记录形状）：

| 字段 | 类型 | 说明 |
|---|---|---|
| `detection_id` | `std::uint64_t` | 本输出帧内的探测记录标识 |
| `azimuth_deg` | `float` | 方位角（deg） |
| `elevation_deg` | `float` | 仰角（deg） |
| `infrared_snr_linear` | `float` | 红外通道线性 SNR |
| `observation_stage` | enum | 原生观测阶段：WFOV 搜索、NFOV 首次捕获或 NFOV 真值辅助跟踪 |
| `detected` | `bool` | 是否通过探测门限判决 |

输出规则（WFOV/NFOV 状态仅决定当前周期哪些目标输出检测记录，不进 raw output 字段）。这里的 raw
output 指 1q 仿真传感器主输出层，不等同于真实 SBIRS 下传的未处理辐射图像或事件消息：

- WFOV 阶段（`WideCandidate`）：输出 WFOV 检测成功目标的检测记录，位置为带误差值。
- NFOV 首次捕获成功周期（`AwaitingNfovAcquisition → TruthAssistedTracking`）：输出 NFOV 捕获后的检测记录。
- NFOV 真值辅助周期（`TruthAssistedTracking`）：持续输出锁定目标的检测记录，位置由真值辅助生成（可叠加显示误差）。
- NFOV 首次捕获失败周期（`AwaitingNfovAcquisition → WideCandidate`）：不输出该目标 NFOV 成功记录，目标回 WFOV 流程。

仿真归属（detection id → 输入 target id/name）、debug view、lifecycle（found/lost）、replay 仅进
`SbirsCycleResult` 和调试视图层，不得混入 `SbirsOutputFrame`。WFOV/NFOV 状态机内部状态如需调试，
通过 debug view 暴露，不影响正式输出接口。

适用边界：

- `SbirsOutputFrame` 是 1q 仿真传感器主输出层，不是 debug view，也不是真实 SBIRS 下传辐射图像。
- `target_id`、输入目标名称、状态机枚举、capture attribution 和生命周期事件只能进入 result/debug/replay 层。
- `range_m`、visible/fused SNR 不属于首批 raw output；距离估计只进入 `SbirsCycleResult` 的 attribution/诊断层。
- 如果后续新增真实 OPIR 风格事件消息或辐射帧，应新增独立 DTO，不得把字段塞回其他模块的输出形状。

验证入口：

- `sbirs_cycle_output_builder_test`
- `sbirs_output_boundary_contract_test`
- `sbirs_replay_codec_roundtrip_test`
- `sbirs_replay_session_test`

## 3. 非目标与边界

- **图像级 TBD（Track-Before-Detect）**——管道滤波能量累积（`:268-294`）、动态规划 TBD
  （`:296-310`）。第一版用 WFOV 单帧 SNR 门控判定可探测性，不做帧间能量累积。理由：TBD 需要
  多帧图像缓存和速度空间搜索，是独立的图像处理子系统；第一版聚焦视场协同与状态机交接。

- **模板匹配 NCC 窄视场捕获**——归一化互相关模板匹配（`:442-472`）。第一版用 WFOV cue 生成
  NFOV 命令指向，再用真实 LOS 是否落入搜索窗口 + SNR 门限判捕获。理由：NCC 需要宽视场目标模板和
  窄视场当前帧图像，依赖图像级数据；第一版的几何 + SNR 判定已能覆盖捕获语义。

- **EKF 已实现；CKF 与波门关联仍为非目标**——扩展卡尔曼滤波（EKF）已接入 `kEstimatedTracking`
  状态（见 2.5a）：6 维 CV 状态 / 2 维角度量测，消费 `common/estimation` 模板化框架。仍不做：
  容积卡尔曼滤波（CKF，sigma-point 路径，需新写 predictor/updater）、波门关联（多假设航迹关联）。
  理由：EKF 已覆盖 SBIRS 红外角度量测的非线性估计需求；CKF 在大俯仰角/近距离机动场景的精度优势
  需先有 EKF 基线对比数据，波门关联需多目标同时跟踪场景。

- **Otsu/DBSCAN 多目标聚类**——自适应阈值分割（`:1174-1182`）、聚类分析（`:1184-1186`）。
  第一版按 `target_id` 独立维护状态机，不做像素级聚类。理由：聚类针对图像级检测点，第一版的
  目标来自输入场景的显式目标列表。

- **多 NFOV 通道同时锁定**——第一版 NFOV 资源采用单目标锁定策略（2.6）。理由：多通道同时跟踪
  需要独立的 NFOV 资源模型和调度器，第一版先用单目标验证状态机和交接语义。

- **Cueing 运动预测与 ATP 姿态机动建模**——CV/CA 运动模型状态预测（`:370-432`）、ATP 快速姿态
  机动（`:434-440`）。第一版不做 CV/CA 滤波运动预测和姿态机动过程建模；但接入 `narrow_cue_latency_s`
  驱动的**线性位置外推**（用 `velocity_ecef_m_per_s` 对真值位置平移），以建模 cue 延迟与目标运动对
  首次捕获窗口的耦合影响。理由：线性外推是已有边界内的能力接线（速度作为可选输入、缺省退化为旧行为），
  而 CV/CA 状态空间搜索与 ATP 闭环稳定仍是后续优化项。

- **不暴露用户自定义 pipeline、controller、状态机、环境模型或 foundation algorithm 类型。**

- **不把仿真目标 ID/name 混入 `SbirsOutputFrame` 的 raw detection。**

- **不把 debug view、lifecycle 或 replay 当作 1q 仿真传感器主输出。**

- **不为测试 mock 便利新增 public 扩展点。**

已实现但仍受边界约束的辅助面：

- `SbirsOutputDebugViewBuilder` / `SbirsDetectionLifecycleRecorder` 只消费输入与 `SbirsCycleResult`，
  用于人读诊断和生命周期事件，不改变 raw output。
- `SbirsTraceSession` / `ReplaySbirsTrace` / `SbirsReplayFlatbufferCodec` 记录和回放的是 1q SBIRS 仿真
  DTO。schema 位于 `schemas/replay/sbirs_replay.fbs` 与
  `schemas/replay/sbirs_session_replay.fbs`，payload type 使用 `Sbirs*`，不复用 EOS schema。
- 交接诊断字段（受三层分离约束，不进 `SbirsOutputFrame` raw output）：
  - `SbirsSceneTarget.velocity_ecef_m_per_s` / `has_velocity_ecef_m_per_s`：目标速度真值，驱动 cue
    延迟外推与动态滞后误差；缺省时行为不变。velocity 进 replay（`sbirs_replay.fbs`）。
  - `SbirsCaptureFailureReason`（枚举）与 `SbirsDetectionAttributionRecord.capture_failure_reason`：
    首次捕获失败/调度跳过诊断，进入 `SbirsCycleResult.detection_attributions` 与 lifecycle reason
    （`kNfovAcquisitionFailed`/`kSchedulerSkipped`），不进 raw output。进 replay（`sbirs_replay.fbs`）。

## 4. 设计变更规则

1. `SbirsOutputFrame`、检测记录字段、`SbirsCycleResult` 或 attribution/debug/lifecycle 语义变化，
   必须同步本文和输出边界测试；不得为复用 EOS consumer 而把 range、visible/fused SNR 塞进 raw output。
2. 状态机状态集合、转移条件、优先级规则变化，必须同步本文 2.2 状态转移图、转移条件表和
   `sbirs_state_machine_test`、`sbirs_scheduler_test`。
3. 气象影响列表、加权叠加公式 `A_total`、衰减进入 SNR 链路的方式变化，必须同步本文 2.9 和
   `sbirs_environment_model_test`、`sbirs_radiative_transfer_test`。
4. 误差模型（5 类误差、加法/乘法合成、折射角与滞后公式）变化，必须同步本文 2.10 和
   `sbirs_error_model_test`；同时检查对 NFOV cue 指向、首次捕获成功率的影响是否需要更新测试期望。
5. runtime patch 的可变字段、立即提交策略或状态机 capture/restore 规则变化，必须同步本文 1.5、
   `docs/common/contract.md` 运行期配置提交策略表和 runtime resolver 测试。
6. foundation 算法如果从 internal 变成 public API，必须在本文 `[evidence: ...]` 标注中记录扩展
   理由、稳定性约束和迁移影响，并检查与 EOS 对应算法的偏离是否有意。
7. 新增 debug/replay 字段时，必须保持真实输出、结构化结果和仿真辅助视图三层分离。

## 5. `docs/review/sbirs_filter.md` 审查记录

`docs/review/sbirs_filter.md` 是历史实现计划草案，不再作为当前设计权威。它经与原始需求、`docs/common/contract.md`
和现有 `electro_optical_sensor`（EOS）代码核对后，保留以下结论：

通过项：

- `docs/review/sbirs_filter.md` 曾提议输出字段与现有红外传感器一致；本文已废弃该方案，首批 raw output 使用原生
  SBIRS-inspired 观测契约。
- 三层输出模型：内部 WFOV/NFOV 状态不进 raw output，符合 `docs/common/contract.md`
  三层输出规则。
- public API 边界：不在公开头暴露 `Eos*` / `electro_optical_sensor`。
- 第一版范围裁剪清晰，非目标边界明确。

修正项：

| # | 主题 | 历史草案问题 | 本文结论 | 理由 |
|---|---|---|---|---|
| 1 | 迁移源头 | 草案称从当前 `space_based_infrared_sensor` 迁移，但仓库中不存在该实现模块 | 真实源头是 `electro_optical_sensor`；参照复制 EOS foundation 层算法，pipeline/controller/session 独立实现 | 全仓库搜索 `sbirs`/`space_based_infrared` 仅命中草案和本文 |
| 2 | 内部状态机 | 草案同时给出两套状态，数量和语义不对齐 | 统一为本文 2.2 的 5 状态；捕获失败回退是转移，不是独立状态 | 同一对象不能有两套状态机 |
| 3 | 地球遮挡门控 | 草案未覆盖天基几何遮挡 | 本文 2.7 要求地球遮挡与大气边界过滤，并以有限 LOS 线段与地球球体相交作为实现判定 | 天基传感器视线穿地不可观测 |
| 4 | 契约注册 | 草案提议 `sbirs_sensor` 命名空间和 `Sbirs*` 前缀，但未同步公共契约 | `docs/common/contract.md` 需同步文档目录、模块前缀、运行期提交策略和模块关系图 | 公共契约是模块边界权威 |
| 5 | 气象衰减 | 草案只笼统提环境 | 本文 2.9 明确气象影响列表查表 + 加权叠加公式 `A_total`，作用于路径透过率并进入 SNR 链路 | 原始需求 `docs/review/sbirs_infrared_model_1205_v3.md:35-103` 有完整定义 |
| 6 | 输出字段 | 草案要求复用 EOS 检测记录形状 | 本文改为原生 SBIRS 输出，不包含必填 range、visible SNR 或 fused SNR | 新模块没有复用 EOS 输出形状的约束，被动红外 raw output 不应伪装成可见光/融合/直接测距输出 |
