---
Status: active
Last-reviewed: 2026-08-07
Authority: AR 数据流、Public API 边界、时序与状态所有权
Answers: AR 的分层架构、数据如何流动、Public API 边界在哪、输出/调试/归属边界、跨周期状态归谁所有
---

# Airborne Radar 数据流

本文承载 AR 的架构图、Public API 边界、时序、主数据流、输出/调试/归属边界和状态所有权。算法逐步逻辑
读代码；本文只回答"组件如何分层、数据如何流动、状态归谁"。

## Public API 与内部实现边界

公共头位于 `include/1q/airborne_radar/`：

| 区域 | 职责 |
|---|---|
| `airborne_radar.hpp` | 模块聚合入口；聚合稳定 public API，不暴露内部 signal/environment/runtime 类型 |
| `config/` | `ArSessionConfig` 四域配置、runtime patch、语义常量表（`ArProfileConstants.h`）、薄封装 builder、validation |
| `session/` | `ArSession`、cycle input/result、scene target、output types、trace/replay、debug/lifecycle、decision DTO |

Public 决策 DTO 只包含 `DecisionObservation`、`ExternalDecisionOverride`、`ExternalDecisionSubmitStatus` 和
`DecisionControlSource`。默认算法使用的 `TargetCategory`、`TacticalMode`、`TacticalStateStore` 和
`TacticalDecisionResult` 位于 `src/airborne_radar/decision/`，不属于安装边界。

内部实现位于 `src/airborne_radar/`（`config/mapping`、`environment`、`signal/{detection,pipeline,association,tracking}`、
`decision`、`runtime`、`session`、`output`）；逐目录职责读代码，本文只标注边界——`signal/tracking` 生产路径
使用 KF，IMM 按策略配置启用，其它滤波原语位于 `src/common/estimation/`，不构成 AR 可选生产后端。

## 分层组件图

```mermaid
flowchart TB
  subgraph Public["Public API / 公共调用面"]
    Entry["airborne_radar.hpp\n稳定聚合入口"]
    Config["config/*\nHardware / Mission / Policy / Environment\nRuntimePatch / Builder / Validation"]
    SessionApi["session/*\nArSession / ArCycleInput / ArCycleResult\nTrackOutputFrame / SceneTarget"]
    DecisionSeam["DecisionObservation / ExternalDecisionOverride\n步间外部决策 seam"]
    Tools["Trace / Replay / Debug / Lifecycle\n追踪 / 回放 / 调试 / 生命周期"]
  end

  subgraph Session["Session orchestration / 会话编排层"]
    ArSession["ArSession\nStep / StepWithResult / RuntimePatch"]
    Composition["ArSessionCompositionRoot\n默认内部依赖图"]
    Context["MutableArContext\n周期输入 / 命令 / 最新控制配置"]
    Rollback["runtime snapshots\nContext / Pipeline / Environment / Controller 快照回滚"]
    Adapters["Input/Output adapters\n外部输入输出适配"]
  end

  subgraph Runtime["Runtime control / 运行期控制层"]
    Controller["ArController\n校验 / 冻结环境 / 执行 pipeline / 调用决策"]
    Mapper["SessionToExecutionMapper\n公开配置到内部工程配置"]
    Patch["RuntimePatchMapper\n运行期变更解析"]
    Command["ControlCommandMapper\n决策 proposal 到控制配置"]
  end

  subgraph Signal["Signal pipeline / 信号与航迹流水线"]
    Env["EnvironmentService\n冻结环境快照 / 内部派生干扰判定"]
    Schedule["ScanScheduleResolver\n扫描调度与驻留中心"]
    Detect["DetectionExecution\n统一物理探测链"]
    Assoc["DataAssociation\nMahalanobis 代价 / LAPJV 分配"]
    Track["TrackLifecycle + Filters\nKF / IMM(KF) 生产链"]
    DecisionFrame["DecisionFrameBuilders\n航迹 / 感知质量 / ECCM 来源"]
  end

  subgraph Decision["Decision algorithms / 战术决策层"]
    Threat["ThreatAssessmentEvaluator\n威胁分类与 LPI 输入"]
    Lpi["LpiEvaluator\n低截获概率发射控制"]
    Eccm["EccmEvaluator\n抗干扰措施"]
    Tactical["TacticalCoordinator\n默认决策协调"]
    Reducer["ControlReducer\n冲突归约 / 可配置保持与冷却（默认 0）"]
  end

  Entry --> Config
  Entry --> SessionApi
  SessionApi --> ArSession
  DecisionSeam --> ArSession
  Config --> ArSession
  ArSession --> Composition
  Composition --> Context
  Composition --> Controller
  ArSession --> Rollback
  ArSession --> Patch
  Patch --> Mapper
  Mapper --> ArSession
  Controller --> Env
  Controller --> Detect
  Env --> Schedule
  Schedule --> Detect
  Detect --> Assoc
  Assoc --> Track
  Track --> DecisionFrame
  DecisionFrame --> Tactical
  DecisionSeam -. "next-cycle proposals / 下一周期建议" .-> Reducer
  Tactical --> Threat
  Tactical --> Lpi
  Tactical --> Eccm
  Tactical --> Reducer
  Reducer --> Command
  Command --> Context
  Adapters --> ArSession
  Tools -. "observe / consume\n观测与消费" .-> ArSession
```

读图要点（边界语义，图本身不传达）：外部只从 observation/response seam 进入；`ArSession` 在运行期配置提交前
捕获 context/pipeline/environment/controller 四类快照，提交或执行失败时回滚避免部分生效；`ArController` 每周期
冻结环境快照让 pipeline 和 decision engine 看同一份事实；决策 proposal 经 `ControlReducer`/
`ControlCommandMapper` 形成下一周期控制配置，不直接改 signal pipeline。

## 执行时序图

`StepWithResult()` 在内部先确定实际 AR 发射，再合并调用方提供的 `RfEmissionFrame` 并完成接收、探测与
跟踪；调用方不管理中间阶段。

```mermaid
sequenceDiagram
  participant Caller as Caller / 调用方
  participant Session as ArSession / 会话门面
  participant Context as MutableArContext / 周期上下文
  participant Env as EnvironmentService / 环境服务
  participant Controller as ArController / 控制器
  participant Pipe as SignalPipeline / 信号流水线
  participant Decision as DecisionEngine / 战术决策
  participant Mapper as CommandMapper / 控制映射

  Caller->>Session: TryApplyRuntimeConfig(patch)\n提交运行期变更
  Session->>Session: stage pending runtime state\n暂存新运行期状态

  Caller->>Session: StepWithResult(input)\n提交单周期输入
  Session->>Session: ValidateArCycleInput\n校验输入
  alt validation error / 输入校验失败
    Session-->>Caller: ArCycleResult(rejected)\n返回校验状态，不复用历史输出
  else valid input / 输入有效
    Session->>Context: CaptureRuntimeState\n捕获上下文快照
    Session->>Pipe: CaptureRuntimeState\n捕获流水线快照
    Session->>Env: CaptureRuntimeState\n捕获环境快照
    Session->>Controller: CaptureRuntimeState\n捕获控制器快照
    Session->>Pipe: Commit pending config\n提交待生效配置
    Session->>Env: Commit pending environment config\n提交待生效环境配置
    alt commit failed / 提交失败
      Session->>Context: RestoreRuntimeState\n回滚上下文
      Session->>Pipe: RestoreRuntimeState\n回滚流水线
      Session->>Env: RestoreRuntimeState\n回滚环境
      Session->>Controller: RestoreRuntimeState\n回滚控制器
      Session-->>Caller: abort result\n返回执行中止
    else commit succeeded / 提交成功
      Session->>Context: BeginCycle(input)\n写入周期输入
      Session->>Controller: RunOnce()\n执行一个周期
      Controller->>Env: BeginCycle + SampleEnvironment\n冻结并采样环境
      Controller->>Mapper: Reduce previous external response or internal baseline\n归约上一成功周期控制
      Mapper->>Pipe: SetControlProfile\n写入本周期唯一控制真值
      Controller->>Pipe: RunCycle(SignalCycleInput, environment)\nscene targets + RF context + interference obs + deception candidates
      Pipe-->>Controller: SignalCycleResult + DecisionInputFrame\n信号结果与决策帧
      Controller->>Decision: Evaluate(frame, state_store)\n评估战术决策
      Decision-->>Controller: TacticalDecisionResult\n当前分类与下一周期 internal baseline
      Controller->>Controller: stage baseline + observation\n暂存 baseline 与观测
      Session->>Session: assemble ArCycleResult from controller/context/pipeline\n组装周期结果
      Session-->>Caller: ArCycleResult + DecisionObservation\n输出结果和决策观测
      Caller->>Caller: external Evaluate(observation)\n同进程外部评估
      Caller->>Session: SubmitExternalDecision(override)\n在下一次 Step 前提交
    end
  end
```

## 主数据流

自然环境与外部 RF 干扰分开输入。

```mermaid
flowchart LR
  subgraph Input["Input / 输入"]
    Config["ArSessionConfig\n硬件 / 任务 / 策略 / 环境"]
    Cycle["ArCycleInput\n平台姿态 / 高度 / 目标 / 干扰"]
    Patch["ArRuntimeConfigPatch\n运行期工程参数 / 自然环境"]
    Rf["RfEmissionFrame\n外部 RF 干扰"]
    Observation["DecisionObservation\n本周期输入帧 + 实际 profile"]
    Response["ExternalDecisionOverride\n下一周期完整 profile 覆盖值"]
  end

  subgraph Environment["Environment / 自然环境"]
    Scene["SceneManager\npending scene 到 active scene"]
    Snapshot["EnvironmentSnapshot\n冻结传播 / 杂波事实"]
  end

  subgraph Signal["Signal and tracking / 信号与航迹"]
    Scan["Scan schedule\n扫描中心 / 波束指向"]
    Detect["Detection pass\n雷达方程 / RCS / 大气 / RF 检测单元 / Monte Carlo"]
    Assoc["Association\nMahalanobis cost / LAPJV / quality metrics"]
    Track["Lifecycle + filters\n确认 / 丢失 / 回收 / Kalman/IMM"]
    Frame["DecisionInputFrame\ntracks / perception / ECCM source"]
  end

  subgraph Decision["Decision and control / 决策与控制"]
    Default["TacticalCoordinator\n默认威胁 / LPI / ECCM"]
    External["External decision module\n同进程外部模块"]
    Reduce["ControlReducer\n优先级 / 冲突 / 保持 / 冷却"]
    Profile["ArControlProfile\n下一周期控制配置"]
  end

  subgraph Output["Output / 输出"]
    TrackOut["TrackOutputFrame\n系统侧航迹输出"]
    Result["ArCycleResult\n执行状态 / commands / metrics / decision provenance"]
    Debug["Debug / Lifecycle / Replay\nArReplayCycleRecord / 调试 / 生命周期 / 回放"]
  end

  Config --> Detect
  Patch --> Detect
  Patch --> Environment
  Config --> Scene
  Rf --> Detect
  Cycle --> Detect
  Scene --> Snapshot
  Snapshot --> Detect
  Scan --> Detect
  Detect --> Assoc
  Assoc --> Track
  Track --> Frame
  Frame --> Default
  Frame --> Observation
  Observation --> External
  External --> Response
  Default --> Reduce
  Response --> Reduce
  Reduce --> Profile
  Profile --> Scan
  Track --> TrackOut
  TrackOut --> Result
  Reduce --> Result
  Assoc --> Result
  Result --> Debug
```

### 远程识别链路（kLrr）

识别是纯并行输出：执行点位于 `DecisionInputFrame` 生成之后（controller 内部），结果仅回填
`TrackOutputFrame`，不进入决策帧与威胁评估。

```text
ArSceneTarget 特征真值（aspect / polarization / range_rcs 样本）
  → RecognitionObservationBuilder（SNR / 带宽 / 驻留 / 视角覆盖约束）
  → Rcs / Motion / Polarization / RangeProfile FeatureExtractor
  → RecognitionTrackState 多周期积累（每 association_key 一份）
  → RecognitionMatcher × RecognitionFeatureDatabase（只读内存基线）
  → ArRecognitionResult 回填 TrackOutputFrame
```

状态所有权：`RecognitionFeatureDatabase` 归 `ArController`（构造加载/析构释放，加载期只读
连接，运行期无连接）；每航迹 `RecognitionTrackState` 随航迹创建、随 `kRecycled`/键重分配
清理；识别快照纳入 `ArControllerRuntimeState` 四类回滚矩阵；`database_version` 入
`ArSessionReplayState`。

## 输出、调试与归属边界

```mermaid
flowchart TB
  subgraph Pipeline["Signal pipeline result / 信号流水线结果"]
    Tracks["TrackStateSnapshotList\n航迹状态快照"]
    Metrics["AssociationQualityMetrics\n匹配率 / 新建率 / 丢失率 / 成本"]
    DecisionFrame["DecisionInputFrame\n决策输入帧"]
  end

  subgraph RealOutput["Real system output / 系统输出"]
    Frame["TrackOutputFrame\n航迹输出帧"]
    Queries["TrackOutputQueries\n按 ID / 状态 / 干扰条件查询"]
  end

  subgraph Diagnostics["Diagnostics / 诊断辅助"]
    Result["ArCycleResult\n执行状态 / 控制配置 / 提交命令"]
    Debug["ArTrackOutputDebugView\n人读排查视图"]
    Lifecycle["ArTrackLifecycleRecorder\nconfirmed / lost / recycled"]
    Replay["ArTraceSession / ArReplaySession\n回放输入输出、决策状态和失败标记"]
  end

  Tracks --> Frame
  Frame --> Queries
  Tracks --> DecisionFrame
  Metrics --> DecisionFrame
  Metrics --> Result
  Frame --> Result
  DecisionFrame --> Result
  Result --> Debug
  Result --> Lifecycle
  Result --> Replay
```

归属边界（图传达分层，以下传达归属禁令）：`TrackOutputFrame` 是唯一系统输出，debug/lifecycle/replay 是
仿真辅助视图，output query 不改变输出语义；决策 SPI 不拥有输出结构，不能绕过内部 output adapter 写系统输出。

**反直觉点（emission_frame 是 base 发射身份）**：公开发布的 `emission_frame`（`RfSceneEmission.antenna`）的
发射功率、载波频率捷变、rejitter 等效果直接由控制 profile 作用到发射，但天线方向图字段读取自未经
`ControlProfileEffects` 处理的 base detection 工程配置。旁瓣对消/自适应波束的效果**只**作用于
`receiver_state.antenna`（接收态），不进公开发射方向图。两个消费者（对外 RF 场景 vs 对内 detector 工程配置）
刻意分属两个物理面，常量有意保持差异。

[evidence: tests/unit/airborne_radar/ar_rf_session_test.cpp]
[evidence: tests/unit/airborne_radar/ar_core_controller_test.cpp]
[evidence: tests/unit/airborne_radar/ar_output_boundary_contract_test.cpp]

## 生命周期与状态所有权

AR 内部按"准备实际发射、冻结接收状态、求解外部 RF、探测与跟踪"分层，但这些步骤不形成 public token
或外部状态机。

```mermaid
flowchart LR
  Input["ArCycleInput\nplatform / targets / interference"] --> Prepare["internal prepare\n解析实际频率/功率/PRF/波束/驻留"]
  Prior["上一成功周期的 ArControlProfile"] --> Prepare
  Prepare --> Tx["本周期实际 AR emission"]
  Tx --> World["internal RF frame\nAR emission + interference"]
  World --> Receive["internal receive\n固定接收状态"]
  Receive --> Echo["双程 target echo resolver"]
  Receive --> Incident["单程 external RF resolver"]
  Echo --> Cells["range/Doppler/beam detection cells"]
  Incident --> Cells
  Cells --> Detect["SINR / Pfa / Pd / measurement covariance"]
  Detect --> Track["association / filter / lifecycle"]
  Detect --> JamObs["interference observation\nJ/N gate + estimated AoA/RF"]
  Track --> Decision["N+1 internal/external decision observation"]
  JamObs --> Decision
  Track --> Result["ArCycleResult\ntracks / impairment / AR emission"]
```

状态所有权固定如下（谁拥有什么、什么消费什么——代码不传达的边界语义）：

1. **transmitter state** 由 AR session 内部发射准备子状态唯一拥有（waveform、frequency-hop/PRI/rejitter
   phase、emission ID、发射随机流）。输入校验、关机或实际发射发布前的配置拒绝不消费这些状态；实际
   emission 一经发布即提交，后续接收/探测执行拒绝只恢复接收侧候选状态，`ArCycleResult::emission_frame`
   仍返回该不可撤销的发射事实。
2. **receiver operating state** 由 signal pipeline 在本周期冻结（接收波束/自适应零陷、调谐、预选器、检测
   窗口、系统损耗、噪声参数、最大线性输入功率），对本周期所有目标和外部发射相同，禁止逐目标临时重指向。
3. **detection/association/tracking state** 由 pipeline/lifecycle/filter 各自拥有；单周期拒绝时恢复全部本周期
   候选状态，饱和则是成功物理周期并推进 missed-detection。
4. **internal/external LPI/ECCM proposal** 在下一次**成功发布实际 emission**时消费；输入拒绝、发射前配置拒绝
   或关机不消费，发射后的接收拒绝不允许再次消费同一 proposal。
5. 旧 `PrepareCycle` / `CompleteCycle` / `AbandonCycle`、opaque token 和 scene freeze 不进入公共头、示例、
   trace 门面或安装消费者。

trace 因果顺序 `cycle_output(N) → decision_input(override profile) → cycle_input(N+1) → cycle_output(N+1)`。
`cycle_output` 使用内部 `ArReplayCycleRecord`（public result + `ArDecisionReplayState`）；内部决策按 live 路径
重新计算并逐字段比较 pending internal baseline、实际采用 proposal、来源 cycle/batch、reducer 计数、
observation 和最终 profile，不支持旧 `ArCycleResult` replay 输出格式。

[evidence: tests/unit/airborne_radar/ar_rf_session_test.cpp]
[evidence: tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp]
[evidence: tests/replay/airborne_radar/ar_rf_trace_session_test.cpp]
