---
Status: active
Last-reviewed: 2026-08-03
Authority: EOS 数据流、Public API 边界、时序与状态所有权
Answers: EOS 的分层架构、数据如何流动、Public API 边界在哪、跨周期状态归谁所有
---

# EOS 数据流

本文承载 EOS 的架构图、Public API 边界、时序、主数据流和状态所有权。算法逐步逻辑读代码；
本文只回答"组件如何分层、数据如何流动、状态归谁"。

## Public API 与内部实现边界

公共头位于 `include/1q/electro_optical_sensor/`：

| 区域 | 职责 |
|---|---|
| `electro_optical_sensor.hpp` | 模块聚合入口；只聚合稳定 public API，不暴露 foundation/pipeline/runtime 内部类型 |
| `config/` | `EosSessionConfig` 四域配置、runtime patch、语义常量表（`EosProfileConstants.h`）、薄封装 builder、config validation |
| `session/` | `EosSession`、cycle input/result、scene target、output types、adapter、trace/replay、debug/lifecycle |

`electro_optical_sensor.hpp` 不是 EOS 全量 public header 汇总。trace/replay、debug view、lifecycle
recorder 等工具头按需单独包含；foundation 算法、pipeline、controller 不通过聚合入口暴露。内部实现位于 `src/electro_optical_sensor/`。新增生产源必须通过 EOS C++11/冻结源/contract guard：

| 目录 | 职责 |
|---|---|
| `config/` | `EosInternalExecutionConfig`、`EosPipelineConfigMapper`（SessionConfig 到内部执行配置） |
| `foundation/` | 光学、传播、辐射、噪声、空间频谱、杂散光基础算法 |
| `environment/` | `ResolveEnvironmentFactors`（preset + 大气观测到环境因子） |
| `pipeline/` | `EosPipeline`、`FrameContext`、`DetectionComputationContext` |
| `runtime/` | `EosController`（单周期调度、输入校验 gate、runtime state capture/restore）、`EosRuntimeConfigResolver` |
| `session/` | `EosSession`（对外门面）、`EosSessionCompositionRoot`（统一装配）、输入输出适配、trace/replay、debug/lifecycle |

## 分层组件图

```mermaid
flowchart TB
  subgraph Public["Public API / 公共调用面"]
    Entry["electro_optical_sensor.hpp\n聚合稳定入口"]
    Config["config/*\n硬件 / 任务 / 策略 / 环境配置\nRuntimePatch / ProfileConstants / Builder / Validation"]
    SessionApi["session/*\nEosSession / CycleInput / CycleResult\nOutputFrame / SceneTarget"]
    Tools["Trace / Replay / Debug / Lifecycle\n追踪 / 回放 / 调试 / 生命周期"]
  end

  subgraph Session["Session orchestration / 会话编排层"]
    EosSession["EosSession\n外部门面：Step / StepWithResult / RuntimePatch"]
    Composition["EosSessionCompositionRoot\n默认依赖图装配"]
    InputAdapters["Input adapters\n外部输入到 EosCycleInput"]
    CoordinateAdapter["EosCycleOutputAdapter\n外部坐标输出转换"]
    ReplayCodec["Replay codec\nFlatBuffer 追踪与回放"]
  end

  subgraph Runtime["Runtime control / 运行期控制层"]
    Controller["EosController\n校验输入 / 执行周期 / 缓存最新输出"]
    Mapper["EosPipelineConfigMapper\nSessionConfig 到内部执行配置"]
    Resolver["EosRuntimeConfigResolver\nPatch 校验与原子更新"]
  end

  subgraph Pipeline["Detection pipeline / 探测流水线"]
    FrameCtx["FrameContext\n帧级光学 / 环境 / 噪声上下文"]
    TargetCtx["DetectionComputationContext\n目标级传播 / 分辨 / 杂散光上下文"]
    Ir["Infrared channel\n红外 SNR"]
    Vis["Visible channel\n可见光 SNR"]
    Fusion["Fused decision\n通道融合与门限"]
  end

  subgraph Foundation["Foundation algorithms / 基础物理算法"]
    Env["environment\npreset / 大气观测到环境因子"]
    Optics["optics\n孔径 / FOV / 衍射 / GSD"]
    Transfer["radiative transfer\n路径透过率 / 路径辐射惩罚"]
    Radiometry["radiometry\nPlanck / Lambertian / contrast"]
    Noise["noise / NEP\n背景噪声 / 等效噪声 / 有效信号"]
    Spatial["spatial spectrum\n空间可分辨性"]
    Stray["stray light\n太阳夹角 / 遮光罩抑制"]
  end

  Entry --> Config
  Entry --> SessionApi
  Config --> EosSession
  SessionApi --> EosSession
  Tools -. "observe / consume\n观测与消费" .-> ReplayCodec
  EosSession --> Composition
  Composition --> Controller
  Composition --> Mapper
  EosSession --> Resolver
  Controller --> FrameCtx
  Mapper --> FrameCtx
  Resolver --> EosSession
  EosSession --> FrameCtx
  InputAdapters --> EosSession
  FrameCtx --> TargetCtx
  TargetCtx --> Ir
  TargetCtx --> Vis
  Ir --> Fusion
  Vis --> Fusion
  Fusion --> Controller
  Controller --> EosSession
  SessionApi --> CoordinateAdapter
  ReplayCodec -. "record/replay\n记录与回放" .-> EosSession
  FrameCtx --> Env
  FrameCtx --> Optics
  TargetCtx --> Transfer
  TargetCtx --> Radiometry
  TargetCtx --> Noise
  TargetCtx --> Spatial
  TargetCtx --> Stray
```

读图顺序：

1. 外部只从 Public API 进入，不直接构造 `EosPipeline` 或 foundation 类型。
2. `EosSessionCompositionRoot` 负责默认依赖图；没有用户替换 controller、pipeline 或环境模型的 public API。
3. `EosPipeline` 把一个周期拆成帧级上下文和目标级上下文，再运行红外、可见光和融合逻辑；foundation
   算法是内部可测试实现，不是模块间契约。

## 执行时序

```mermaid
sequenceDiagram
  participant Caller as Caller / 调用方
  participant Session as EosSession / 会话门面
  participant Controller as EosController / 周期控制器
  participant Validator as Validation / 输入校验
  participant Pipeline as EosPipeline / 探测流水线
  participant Env as Environment / 环境模型
  participant Physics as Foundation / 物理算法
  participant Resolver as RuntimeResolver / 补丁解析器

  Caller->>Session: StepWithResult(input)\n提交单周期输入
  Session->>Controller: RunOnce(input)\n执行一个周期
  Controller->>Validator: ValidateEosCycleInput(input, frame_rate_hz)\n校验步长 / 平台 / 环境 / 目标
  alt invalid input / 输入无效
    Validator-->>Controller: issues\n错误列表
    Controller-->>Session: EosCycleResult（默认空帧，不复用）\n组装校验状态与默认输出帧
  else valid input / 输入有效
    Controller->>Pipeline: Execute(input)\n进入探测流水线
    Pipeline->>Env: ResolveFactors(environment input)\n解析环境因子
    Pipeline->>Physics: build FrameContext\n构造帧级光学 / 噪声上下文
    loop each target / 每个目标
      Pipeline->>Physics: FOV membership / radiometry / range eligibility\n视场成员 / 辐射噪声 / 距离检测资格
      Physics-->>Pipeline: IR SNR + visible SNR + quality\n通道 SNR 与成像质量
    end
    Pipeline-->>Controller: detections + attribution\n检测记录与仿真归属
    Controller-->>Session: OutputFrame + attribution + diagnostics\n直接组装系统输出与结构化结果
  end
  Session-->>Caller: EosCycleResult\n返回结果

  Caller->>Session: TryApplyRuntimeConfig(patch)\n提交运行期变更
  Session->>Resolver: ResolveEosRuntimeConfigPatch(current, patch)\n原子解析 patch
  Resolver-->>Session: next config + reset_scan_phase\n返回解析结果
  Session->>Pipeline: ApplyInternalConfig(reset_scan_phase)\n更新内部配置并按需重置扫描相位
```

## 主探测数据流

```mermaid
flowchart LR
  subgraph Input["Input / 输入"]
    Config["EosSessionConfig\nHardware / Mission / Policy / Environment"]
    Cycle["EosCycleInput\n平台姿态 / 环境快照 / 目标列表"]
    Patch["EosRuntimeConfigPatch\n工作模式 / 扫描 / 门限 / 环境模型"]
  end

  subgraph Runtime["Runtime mapping / 运行期映射"]
    Internal["EosInternalExecutionConfig\n内部执行配置"]
    Frame["FrameContext\n帧级上下文"]
    Scan["scan phase + FOV gate\n扫描相位与视场门控"]
  end

  subgraph Target["Per-target context / 目标级上下文"]
    Range["range eligibility\nDmin / Dmax；保留检测记录"]
    Transfer["radiative transfer\n路径透过率与路径辐射惩罚"]
    Quality["GSD + spatial spectrum\n地面采样距离与空间可分辨性"]
    Stray["stray-light filter\n太阳夹角与遮光罩抑制"]
  end

  subgraph Channels["Channel evaluation / 通道计算"]
    IR["Infrared SNR\nPlanck 辐射 / 红外对比 / NEP"]
    VIS["Visible SNR\nLambertian 反射 / 光子噪声"]
    Fuse["Fusion\n昼夜权重 / 阈值 / 检测判定"]
  end

  subgraph Output["Output / 输出"]
    Raw["EosOutputFrame\n真实传感器侧检测记录"]
    Result["EosCycleResult\n状态 / attribution / debug source"]
    Trace["Trace / Replay\n可回放输入输出"]
  end

  Config --> Internal
  Patch --> Internal
  Cycle --> Frame
  Internal --> Frame
  Frame --> Scan
  Scan --> Transfer
  Transfer --> Quality
  Quality --> Stray
  Stray --> IR
  Stray --> VIS
  IR --> Fuse
  VIS --> Fuse
  Fuse --> Range
  Range --> Raw
  Raw --> Result
  Result --> Trace
```

## 输出与仿真归属数据流

```mermaid
flowchart TB
  subgraph Pipeline["Pipeline result / 流水线结果"]
    Detection["EosDetectionRecord\n检测 ID / 角度 / 距离 / SNR / 通道"]
    Attribution["Attribution\n检测 ID 到输入目标的仿真归属"]
    Issues["Validation/runtime status\n校验与运行期状态"]
  end

  subgraph RealOutput["Real sensor output / 真实系统输出"]
    Frame["EosOutputFrame\n外部可视作传感器输出"]
  end

  subgraph SimulationAid["Simulation aid / 仿真辅助"]
    Result["EosCycleResult\n归属 / 诊断 / 最近输出"]
    Debug["DebugView\n检测记录与输入目标合并显示"]
    Lifecycle["LifecycleRecorder\nfound / lost / optional not-detected"]
    Replay["ReplayTrace\n输入输出与失败标记"]
  end

  Detection --> Frame
  Detection --> Result
  Attribution --> Result
  Issues --> Result
  Result --> Debug
  Result --> Lifecycle
  Result --> Replay
```

设计要点：

- `EosOutputFrame` 是系统输出，不携带仿真目标 ID/name；raw detection 只保留真实传感器侧字段
  （detection id、方位/俯仰、距离、SNR、通道）。
- 仿真归属由 `EosCycleResult` 和 debug view 承接；lifecycle 是跨周期辅助视图，不替代 raw output；
  replay 用于复现输入输出和失败标记，不作为 public 算法扩展点。
- **replay double-precision 契约（反直觉）**：replay cycle-input 中的平台位置、速度与欧拉角必须保持
  public `PoseState` 的 double 精度；不允许 schema/codec 静默降为 float。即使当前检测 pipeline 不消费
  平台位姿，trace 仍必须能精确重组调用方输入。

[evidence: tests/replay/electro_optical_sensor/eos_replay_codec_roundtrip_test]

## 生命周期与状态所有权

`EosSession` 的内部状态包括：

1. 当前 runtime config（经 `EosRuntimeConfigResolver` 原子更新）。
2. `previous_output`：输入或运行期配置失败时复用上一有效输出（首个周期无效时不合成虚假输出）。
3. 扫描相位：runtime patch 改变扫描速率/工作模式时由 resolver 显式决定是否重置。
4. lifecycle 视图：found/lost/optional not-detected 跨周期事件，是辅助视图不是 raw output 状态。

`Step()` 只返回 `EosOutputFrame`；`StepWithResult()` 返回结构化执行状态、abort reason、attribution、
diagnostics 和 debug source。日志不作为状态判断依据。controller runtime state 支持 capture/restore，
但必须拒绝不兼容的 pipeline snapshot 或其他 controller 实例的 snapshot。

### 会话创建入口

`EosSession::Create()` 是信任构造路径，不隐式调用初始化校验。`CreateWithDiagnostics(config, issues)`
会报告 config 校验问题但仍构造会话（非阻断，见 contract.md §会话创建入口的非阻断语义）；真正阻断
执行的是每周期输入 gate 和 runtime patch 的原子校验。

[evidence: tests/unit/electro_optical_sensor/eos_session_create_test]
[evidence: tests/unit/electro_optical_sensor/eos_controller_runtime_state_test]
