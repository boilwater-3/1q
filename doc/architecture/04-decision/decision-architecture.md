# Decision 层架构说明

## 1. 简介

Decision 层负责把单周期的目标快照、环境干扰事实和跨周期战术状态，归并为下一周期生效的统一控制真值 `common::RadarControlProfile`。

当前代码中的正式链路是：

```text
DecisionInputFrame
  -> TacticalCoordinator
     -> ThreatAssessmentEvaluator
     -> EmissionControlEvaluator
     -> SurvivabilityEvaluator
  -> ControlReducer
  -> RadarControlProfile
```

其中 ECCM 并不是直接驱动信号处理实现细节，而是输出“应该启用哪些抗干扰策略”的控制意图；具体如何作用到探测、关联和跟踪，由 `SignalPipeline` 在下一周期执行时统一映射。

## 2. 目录结构

```text
include/1q/airborne_radar/decision/
├── ITacticalDecisionEngine.h              # 决策引擎接口、proposal/result/state 契约
├── TacticalCoordinator.h                  # 默认协调器
├── ControlReducer.h                       # proposal -> RadarControlProfile
├── classifier/ThreatAssessmentEvaluator.h # 威胁评估
├── lpi/EmissionControlEvaluator.h         # LPI 评估
└── eccm/SurvivabilityEvaluator.h          # ECCM 生存性评估

src/airborne_radar/decision/
├── ControlReducer.cpp
├── classifier/ThreatAssessmentEvaluator.cpp
├── lpi/EmissionControlEvaluator.cpp
└── eccm/SurvivabilityEvaluator.cpp
```

## 3. 主处理链路

```text
EnvironmentService.SampleEnvironment()
  -> EnvironmentSnapshot
  -> RadarController 构造 DecisionInputFrame(environment_jamming_detected, tracks)
  -> TacticalCoordinator 顺序执行三个 evaluator
  -> TacticalProposal 列表
  -> ControlReducer.Reduce()
  -> RadarControlProfile
  -> SignalPipeline.SetControlProfile()
  -> 下一周期 RunCycle() 按 profile 调整探测/关联/跟踪
```

这个链路有三个关键边界：

1. 环境层只提供“发生了什么”的事实，不做“怎么对抗”的策略选择。
2. 决策层只决定“启用哪些策略组合”，不直接修改探测器内部参数。
3. 信号层只负责执行控制真值，不反向决定是否启用 ECCM。

## 4. ECCM 与环境层的职责边界

### 4.1 环境层职责

环境层的职责是生成可复用的干扰事实。当前正式接口是 `environment::EnvironmentSnapshot`：

| 字段 | 当前含义 | 决策/ECCM 如何使用 |
|------|----------|--------------------|
| `propagation_loss_db` | 传播损耗 | 供信号层探测使用 |
| `clutter_power_db` | 杂波强度 | 供信号层探测使用 |
| `jamming_detected` | 是否检测到干扰 | 决策层 ECCM 触发条件 |

因此，环境层当前已经承担了“干扰存在性判定”的事实生产者角色。

建议的进一步职责边界如下：

| 子职责 | 是否属于环境层 |
|--------|----------------|
| 干扰是否存在、强度多大、来自哪个角域 | 是 |
| 干扰是压制式、欺骗式还是转发式 | 是 |
| 干扰与当前工作频率/PRF 的重叠程度 | 是 |
| 是否启用频率捷变、重频抖动、旁瓣对消 | 否 |
| 是否允许烧穿增益覆盖 LPI 功率压低 | 否，由决策层决定 |

结论：环境层拥有“物理事实”的解释权，不拥有“战术动作”的解释权。

### 4.2 决策层 ECCM 职责

`eccm::SurvivabilityEvaluator` 的职责是把干扰事实转换为控制意图集合。当前实现中，只要 `evaluation_state.eccm_source_info.has_jamming_signal` 或 `input_frame.environment_jamming_detected` 为真，就会输出以下 proposal：

- `REQUEST_ENABLE_SIDELOBE_CANCELLER`
- `REQUEST_ENABLE_ADAPTIVE_BEAMFORMING`
- `REQUEST_AGILITY_FREQUENCY`
- `REQUEST_ECCM_REJITTER`
- `REQUEST_ECCM_BURNTHROUGH_GAIN`

这说明当前 ECCM 设计已经是“策略可叠加”的组合式输出，而不是“多选一”的互斥策略。

另外，当前决策链已经引入 `AssociationQualityInfo` 作为补充证据，但边界被限定为：

- `TacticalCoordinator` 只把“高关联压力 + 欺骗/转发语义”当成 ECCM 触发补位，不把它伪装成完整环境事实。
- `SurvivabilityEvaluator` 再根据 `association_stress`、`jamming_severity` 和语义类型，抬高频率捷变 / 重频抖动 / 自适应波束形成的 proposal 优先级。
- 因此，仅由关联压力触发的 ECCM 不应默认推出旁瓣对消或烧穿增益这类需要更强环境事实支撑的动作。
- 纯粹的匹配率下降、随机丢检或无类型语义的关联变差，不应直接触发 ECCM。

同时，控制器还会补充 `PerceptionQualityInfo`，用于把“探测阶段已经明显掉量”与“探测还在、但关联在抖”分开表达。当前版本里它主要用于决策摘要与模式解释，不直接驱动新的控制动作。

ECCM 在决策层的正式职责应该限定为：

| 决策层 ECCM 要做 | 决策层 ECCM 不做 |
|------------------|------------------|
| 根据干扰事实决定启用哪些策略 | 直接修改 `SignalDetector` |
| 决定策略优先级、保持周期和冲突裁决 | 计算具体噪声功率 |
| 把多个策略压缩成统一 `RadarControlProfile` | 解释传播损耗和杂波来源 |

### 4.3 信号层职责

信号层读取 `RadarControlProfile`，把策略映射到运行时配置：

| 策略 | 当前信号层落点 |
|------|----------------|
| 旁瓣对消 | 降低 `jam_noise_w` / `clutter_noise_w`，压低方向图旁瓣 |
| 自适应波束形成 | 缩窄波束宽度，提高主瓣增益，减小量测噪声 |
| 频率捷变 | 修改 `frequency_hz`，同步提高关联与跟踪保守度 |
| 重频抖动 | 修改 `prf_hz`，同步提高关联与跟踪保守度 |
| 烧穿增益 | 通过 `eccm_burnthrough_gain` 提升有效探测能力并减小量测噪声 |

因此信号层是“战术执行器”，不是策略选择器。

## 5. 叠加策略设计

当前 `RadarControlProfile` 已经天然支持叠加：

| 控制真值字段 | 语义域 | 是否可叠加 |
|--------------|--------|------------|
| `enable_sidelobe_canceller` | 空域抗干扰 | 可与其他策略叠加 |
| `enable_adaptive_beamforming` | 空域抗干扰 | 可与其他策略叠加 |
| `enable_agility_frequency` | 频域抗干扰 | 可与其他策略叠加 |
| `enable_eccm_rejitter` | 时域抗干扰 | 可与其他策略叠加 |
| `eccm_burnthrough_gain` | 能量域抗干扰 | 可与其他策略叠加 |

推荐把 ECCM 策略看成四个正交维度：

1. 空域：旁瓣对消、自适应波束形成。
2. 频域：频率捷变。
3. 时域：重频抖动。
4. 能量域：烧穿增益。

这样设计的好处是：

- 可以清晰表达组合策略，而不是在 evaluator 内部硬编码某一种“大招模式”。
- `ControlReducer` 只需要处理冲突和限幅，不需要理解干扰物理。
- `SignalPipeline` 可以按域做参数映射，避免相互覆盖。

## 6. 建议的数据流分层

### 6.1 当前已落地的数据流

```text
EnvironmentSnapshot.jamming_detected
  -> DecisionInputFrame.environment_jamming_detected
  -> TacticalEvaluationState.should_enable_eccm
  -> TacticalProposal[]
  -> RadarControlProfile
  -> SignalPipeline::ApplyControlProfileToConfig()
```

### 6.2 建议扩展的数据流

当前只有一个布尔量，足以触发 ECCM，但不足以支撑“按干扰类型选择最优组合”。建议保持单向数据流不变，只扩展事实颗粒度：

```text
环境层干扰事实
  -> EccmSourceInfo
  -> TacticalCoordinator / SurvivabilityEvaluator
  -> TacticalProposal[]
  -> RadarControlProfile
  -> SignalPipeline 运行时配置
```

```text
信号层关联质量摘要
  -> AssociationQualityInfo
  -> TacticalCoordinator(仅做 ECCM 触发补位)
  -> SurvivabilityEvaluator(只做 proposal 优先级修正)
```

```text
控制器探测质量摘要
  -> PerceptionQualityInfo(detection_rate / detection_stress)
  -> TacticalCoordinator(只做模式说明与原因摘要)
```

建议扩展的是“事实”，而不是绕过 reducer 直接给信号层下参数。

最小可用的扩展事实建议：

| 事实维度 | 为什么需要 |
|----------|------------|
| 干扰强度或 J/S 估计 | 决定是否需要烧穿增益 |
| 干扰入射方向/角域 | 决定旁瓣对消与自适应波束形成是否有效 |
| 与当前载频的重叠程度 | 决定频率捷变价值 |
| 与当前 PRF 的锁定/相干程度 | 决定重频抖动价值 |
| 干扰类型标签（压制/欺骗/转发） | 决定组合策略优先级 |

注意：这些字段是设计建议，不是当前已实现 API。

## 7. 各抗干扰技术如何落到信号层

### 7.1 旁瓣对消

适用条件：

- 干扰主要通过旁瓣进入。
- 已知或可估计干扰角域。

信号层作用点：

- 降低干扰噪声注入 `jam_noise_w`。
- 降低杂波/旁瓣泄漏。
- 修改天线方向图旁瓣电平。

当前代码对应：

- `enable_sidelobe_canceller` 会压低 `jam_noise_w`、`clutter_noise_w`
- 并降低 `antenna.pattern.max_sidelobe_level_db`

### 7.2 自适应波束形成

适用条件：

- 需要通过空域零陷/主瓣增强提升抗干扰能力。

信号层作用点：

- 提高主瓣增益。
- 缩小有效波束宽度。
- 通过 `MeasurementErrorModel` 间接改善角度测量精度。

当前代码对应：

- `enable_adaptive_beamforming` 会提高 `main_beam_gain_db`
- 并通过 `ResolveBeamwidthScale()` 缩窄波束
- 同时降低 `kalman_measurement_noise_std`

### 7.3 频率捷变

适用条件：

- 干扰与当前载频高度重叠，且跳频可打破干扰锁定。

信号层作用点：

- 修改发射频率 `frequency_hz`
- 重新影响探测链回波预算
- 由于量测统计特性发生变化，适度提高关联和跟踪保守度

当前代码对应：

- `enable_agility_frequency` 修改 `transmitter.frequency_hz`
- 同时上调 `association.unassigned_cost` 与 `kalman_noise_diff_coeff`

### 7.4 重频抖动

适用条件：

- 需要打破转发式/相干式干扰对时序的锁定。

信号层作用点：

- 修改 `prf_hz`
- 让虚假目标时序更难稳定对齐
- 对跟踪层增加适度模型不确定性

当前代码对应：

- `enable_eccm_rejitter` 修改 `transmitter.prf_hz`
- 同时上调 `association.unassigned_cost` 与 `kalman_noise_diff_coeff`

### 7.5 烧穿增益

适用条件：

- 干扰强但目标高价值，需要用更高能量/更强接收能力强行保持探测链闭合。

信号层作用点：

- 提高有效探测能力
- 改善量测质量
- 提升跟踪连续性保护

当前代码对应：

- `eccm_burnthrough_gain` 会降低有效 `noise_figure_db`
- 放宽关联保留能力
- 降低 `kalman_measurement_noise_std`

## 8. 冲突与优先级建议

当前实现允许 LPI 与 ECCM 同时打开，这对“可叠加”是正确的。当前 reducer 已经具备域级 release / hold / cooldown 与显式冲突裁决骨架，建议保持以下原则：

1. 安全/生存性优先于隐身性。
2. ECCM 能量域动作可以覆盖 LPI 的功率压低，但应保留版本化痕迹。
3. 空域、频域、时域策略默认可并行；只有作用到同一物理参数时才需要 reducer 限幅。

因此推荐的 reducer 规则是：

- `eccm_burnthrough_gain > 1.0f` 时，允许覆盖部分 `lpi_power_scale` 带来的负面影响。
- 旁瓣对消与自适应波束形成可同时打开。
- 频率捷变与重频抖动可同时打开。

当前已落地的 reducer 规则包括：

- 当某一域没有新 proposal 且保持周期已结束时，自动回落到基线 profile，而不是无限继承旧状态。
- LPI / ECCM 可以分别配置域级 hold 与 release cooldown。
- `eccm_burnthrough_gain > 1.0f` 时，仍可通过功率保护下限覆盖部分 `lpi_power_scale`。
- `REQUEST_LPI_BEAMFORMING` 与 `REQUEST_ENABLE_ADAPTIVE_BEAMFORMING` 同时出现时，默认优先生存性，保留自适应波束形成。

后续如果策略种类继续增加，仍应优先扩展 reducer，而不是让 evaluator 直接写信号参数。

## 9. 当前状态与设计缺口

### 已落地

- `SurvivabilityEvaluator` 已实现 ECCM proposal 输出
- `ControlReducer` 已把 proposal 归并为 `RadarControlProfile`
- `RadarController` 已把 profile 注入 `SignalPipeline`
- `SignalPipeline` 已把多种 ECCM 控制真值映射到探测/关联/跟踪运行时配置

### 仍需补强

- 多源干扰事实仍是单层列表，尚未形成更稳定的主源/群组/角域聚类抽象
- `ControlReducer` 已具备域级状态管理，但还没有更细粒度的策略表与来源追踪
- 信号层尚未把 reducer 的域级状态元数据转化为更明确的统计稳定性策略

## 10. 推荐阅读

- [environment-architecture.md](/Users/aurora/Code/1q/doc/architecture/environment/environment-architecture.md)
- [signal-architecture.md](/Users/aurora/Code/1q/doc/architecture/signal/signal-architecture.md)
