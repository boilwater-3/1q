# 对外公开模块输出说明手册

本文说明 AR、EOS、ESR 三个模块对外公开的单周期输出结构，覆盖 `include/1q/.../session/*CycleResult.h`、输出记录类型、`Step()` / `StepWithResult()` 语义，以及辅助查询接口。

## 1. 通用输出入口

三个模块都提供两个单周期执行入口：

| 接口 | 返回值 | 适用场景 |
| --- | --- | --- |
| `Step(input)` | 只返回输出帧。 | 调用方只关心业务输出，不关心本周期是否实际执行、是否复用上一帧、校验问题或 abort 原因。 |
| `StepWithResult(input)` | 返回聚合结果。 | 调用方需要区分正常执行、输入校验失败、下游 abort、复用上一有效输出、校验问题列表等状态。 |

建议外部系统默认使用 `StepWithResult()` 做集成，因为它能明确区分“本周期有输出帧”和“本周期实际执行成功”。`Step()` 是输出便捷入口。

聚合结果中的常见状态字段：

| 字段 | 说明 |
| --- | --- |
| `input_cycle_index` | 本次调用输入周期号。即使执行失败，也可用于 trace、日志和故障归属。 |
| `validation_issues` | 本周期输入校验结果列表。 |
| `has_validation_error` | 是否存在 error 级输入问题。 |
| `executed_this_cycle` | 本次调用是否真正执行了核心处理链路。 |
| `reused_previous_*` | 当前输出帧是否复用了上一有效周期输出。不同模块字段名略有差异。 |
| `abort_reason` | 下游或会话层中止原因。AR 字段名是 `signal_cycle_abort_reason`。 |

## 2. AR 机载雷达输出

### 2.1 输出入口

来源：

- `include/1q/airborne_radar/session/RadarSession.h`
- `include/1q/airborne_radar/session/RadarCycleResult.h`
- `include/1q/airborne_radar/model/TrackStateSnapshot.h`

| 接口 | 返回值 | 说明 |
| --- | --- | --- |
| `RadarSession::Step(input)` | `TrackOutputFrame` | 只返回轨迹输出帧。非法输入时不抛异常，可能返回上一有效输出或空轨迹帧。 |
| `RadarSession::StepWithResult(input)` | `RadarCycleResult` | 返回轨迹输出帧、控制指令、校验问题、执行状态、控制真值和关联质量指标。 |

### 2.2 `TrackOutputFrame`

`TrackOutputFrame` 是 AR 对外发布的单周期轨迹帧。

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `cycle_index` | 当前输出帧周期号。 | 用于和输入周期、trace、外部时序系统对齐。 |
| `batch_id` | 当前批号。 | 用于区分会话内连续输出批次，失败/复用场景下也可用于排查输出来源。 |
| `tracks` | `TrackStateSnapshotList`。 | 当前周期发布的轨迹快照列表。 |

### 2.3 `TrackStateSnapshot`

`TrackStateSnapshot` 是单条稳定轨迹状态快照。

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `association_key` | 当前快照对应的关联键。 | 用于跨周期跟踪同一内部轨迹。 |
| `external_target_id` | 输入目标的外部 ID，`0` 表示未知。 | 用于把输出轨迹映射回外部场景目标。 |
| `status` | 生命周期状态：`kTentative`、`kConfirmed`、`kLost`。 | 区分候选、确认、丢失轨迹。 |
| `position_x/y/z` | 雷达局部笛卡尔位置，单位 m。 | 表示当前轨迹位置估计。 |
| `velocity_x/y/z` | 速度向量分量，单位 m/s。 | 表示当前轨迹速度估计。 |
| `speed` | 速度模长，单位 m/s。 | 便于外部直接使用目标速度大小。 |
| `acceleration_x/y/z` | 加速度向量分量，单位 m/s^2。 | 表示当前轨迹加速度估计。 |
| `acceleration` | 加速度模长，单位 m/s^2。 | 便于外部直接使用机动强度。 |
| `rcs` | 目标估计 RCS，单位 m^2。 | 可用于目标类型推断、威胁评估或显示。 |
| `jamming_detected` | 该轨迹是否携带干扰观测标记。 | 用于抗干扰态势显示和策略判断。 |
| `hit_count` | 命中累计计数。 | 反映轨迹稳定程度。 |
| `miss_count` | 连续失配计数。 | 反映目标丢失风险。 |
| `target_type` | 决策层目标分类字符串。 | 例如 `UNKNOWN`、低威胁目标、高威胁目标等。 |
| `target_probability` | 分类置信度，范围 `[0, 1]`。 | 表示 `target_type` 的可信程度。 |

### 2.4 AR 查询辅助接口

来源：`include/1q/airborne_radar/session/RadarCycleResult.h`

| 接口 | 说明 |
| --- | --- |
| `BuildTrackMapByExternalTargetId(frame)` | 按外部目标 ID 构造轨迹映射。适合把轨迹回填到外部目标对象。 |
| `BuildTrackMapByAssociationKey(frame)` | 按内部关联键构造轨迹映射。适合跨周期跟踪内部轨迹。 |
| `CollectTracksByExternalTargetId(frame, id)` | 收集指定外部目标 ID 对应的全部轨迹。 |
| `CollectConfirmedTracks(frame)` | 收集所有 confirmed 轨迹。 |
| `CollectLostTracks(frame)` | 收集所有 lost 轨迹。 |
| `CollectJammingTracks(frame)` | 收集携带干扰标记的轨迹。 |
| `ContainsExternalTargetId(frame, id)` | 判断输出帧是否包含指定外部目标 ID。 |
| `CountJammingTracks(frame)` | 统计干扰标记轨迹数量。 |
| `CountTracksByStatus(frame, status)` | 按生命周期状态统计轨迹数量。 |

### 2.5 `RadarCycleResult`

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `input_cycle_index` | 本次调用输入周期号。 | 失败结果与 trace 归属。 |
| `track_output_frame` | 当前调用返回的轨迹输出帧。 | 业务输出主体。 |
| `submitted_commands` | 当前周期已提交的控制指令列表。 | 供外部观察雷达控制/战术指令输出。未执行时为空。 |
| `validation_issues` | 当前周期输入校验结果。 | 调试输入问题。 |
| `has_validation_error` | 是否存在 error 级输入问题。 | 为 true 时本周期通常不会执行主链路。 |
| `executed_this_cycle` | 是否真正执行 signal/decision/control 链路。 | 判断输出是否来自本周期计算。 |
| `signal_cycle_abort_reason` | 下游主链路 abort 原因。 | 区分运行准备失败、输入非法、恢复失败等结构化故障。 |
| `reused_previous_track_output` | `track_output_frame` 是否复用上一有效周期输出。 | 判断输出是否为回退结果。 |
| `has_control_profile` | 当前周期是否产出控制真值。 | 为 false 时 `control_profile` 保持默认值。 |
| `control_profile` | 当前周期控制真值。 | 供调试/回放/策略验证使用。 |
| `association_quality_metrics` | 当前周期关联质量观测指标。 | 供外部评估关联压力、匹配质量和干扰影响。未执行时保持默认值。 |

## 3. EOS 光电传感器输出

### 3.1 输出入口

来源：

- `include/1q/electro_optical_sensor/session/EosSession.h`
- `include/1q/electro_optical_sensor/session/EosCycleResult.h`
- `include/1q/electro_optical_sensor/extension/EosPipelineTypes.h`

| 接口 | 返回值 | 说明 |
| --- | --- | --- |
| `EosSession::Step(input)` | `EosOutputFrame` | 只返回单周期探测输出帧。 |
| `EosSession::StepWithResult(input)` | `EosCycleResult` | 返回探测输出、校验问题、执行状态、复用状态和 abort 原因。 |

### 3.2 `EosOutputFrame`

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `cycle_index` | 当前周期号。 | 与输入、trace、外部时间线对齐。 |
| `scan_azimuth_deg` | 当前周期扫描中心方位角，单位 deg。 | 表示本周期光电扫描相位，可用于画面/扫描态显示。 |
| `detections` | `EosDetectionRecordList`。 | 当前周期单目标探测输出列表。 |

### 3.3 `EosDetectionRecord`

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `target_id` | 目标标识。 | 对应输入 `EosSceneTarget::target_id`，用于外部回填。 |
| `range_m` | 斜距，单位 m。 | 输出检测目标距离。 |
| `azimuth_deg` | 方位角，单位 deg。 | 输出检测目标方位。 |
| `elevation_deg` | 仰角，单位 deg。 | 输出检测目标仰角。 |
| `infrared_snr_linear` | 红外通道线性 SNR。 | 表示红外通道检测质量。 |
| `visible_snr_linear` | 可见光通道线性 SNR。 | 表示可见光通道检测质量。 |
| `fused_snr_linear` | 融合线性 SNR。 | 表示融合模式下综合检测质量。 |
| `fused_snr_db` | 融合 dB SNR。 | 便于和 dB 门限、日志或显示系统对齐。 |
| `detected` | 是否通过探测门限判决。 | 外部应以此判断该记录是否为有效检测。 |

### 3.4 `EosCycleResult`

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `input_cycle_index` | 本次调用输入周期号。 | 失败结果与 trace 归属。 |
| `output_frame` | 当前周期输出帧。 | 业务输出主体。 |
| `validation_issues` | 当前周期输入校验结果。 | 调试输入问题。 |
| `has_validation_error` | 是否存在 error 级输入问题。 | 为 true 时本周期通常不会执行 pipeline。 |
| `executed_this_cycle` | 当前周期是否实际执行核心 pipeline。 | 判断 `output_frame` 是否来自本周期计算。 |
| `reused_previous_output` | 当前周期是否复用上一有效输出。 | 输入失败且已有上一帧时可为 true。 |
| `abort_reason` | 当前周期终止原因。 | 取值包括 `kNone`、`kValidationRejected`、`kOutputContractViolation`、`kRuntimeStateRestoreRejected`。 |

## 4. ESR 电子侦察输出

### 4.1 输出入口

来源：

- `include/1q/electronic_surveillance_radar/session/EsrSession.h`
- `include/1q/electronic_surveillance_radar/session/EsrCycleResult.h`
- `include/1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h`
- `include/1q/electronic_surveillance_radar/model/EmitterObservation.h`
- `include/1q/electronic_surveillance_radar/model/EmitterHypothesis.h`

| 接口 | 返回值 | 说明 |
| --- | --- | --- |
| `EsrSession::Step(input)` | `EsrOutputFrame` | 只返回三通道输出帧。 |
| `EsrSession::StepWithResult(input)` | `EsrCycleResult` | 返回三通道输出、校验问题、执行状态、复用状态和 abort 原因。 |

### 4.2 `EsrOutputFrame`

ESR 输出分为三个通道：观测、侦察假设、真值评估。`cycle_index` 和 `batch_id` 只在顶层输出帧中作为规范周期头。

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `cycle_index` | 当前周期号。 | 与输入、trace、外部时间线对齐。 |
| `batch_id` | 当前批次号。 | 区分会话内连续输出批次。 |
| `observation_output` | 传感器观测输出通道。 | 输出原始/预处理后的接收机观测记录和聚类数量。 |
| `emitter_output` | 侦察假设输出通道。 | 输出辐射源假设、候选类别、工作模式和威胁等级。 |
| `truth_evaluation_output` | 真值评估输出通道。 | 输出观测与输入真值辐射源之间的评估关联，仅用于评估/测试/回放。 |

### 4.3 `ObservationOutputFrame`

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `raw_observation_count` | 原始观测数量。 | 表示本周期截获/预处理前后的观测规模。 |
| `cluster_count` | 聚类数量。 | 表示观测被聚类后的簇数量。 |
| `observations` | `EmitterObservationList`。 | 当前周期观测记录列表。 |

### 4.4 `EmitterObservation`

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `observation_id` | 观测记录唯一标识。 | 用于关联、回放和评估。 |
| `timestamp_s` | 观测时间戳，单位 s。 | 表示观测发生时间。 |
| `aoa_az_deg` | 测得方位角，单位 deg。 | 表示到达角方位估计。 |
| `aoa_el_deg` | 测得俯仰角，单位 deg。 | 表示到达角俯仰估计。 |
| `rf_hz` | 测得载频，单位 Hz。 | 表示截获信号频率特征。 |
| `pulse_width_s` | 测得脉宽，单位 s。 | 表示脉冲时域特征。 |
| `amplitude_db` | 接收幅度，单位 dB。 | 表示接收信号强度。 |
| `snr_db` | 观测信噪比，单位 dB。 | 表示观测质量和检测裕量。 |
| `quality` | 观测质量等级：低/中/高。 | 供外部筛选或显示观测可信度。 |
| `is_jammed` | 该观测是否受显著干扰影响。 | 用于干扰态势显示和后续处理。 |

### 4.5 `EmitterOutputFrame` 与 `EmitterHypothesis`

`EmitterOutputFrame` 只有一个字段：

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `hypotheses` | `EmitterHypothesisList`。 | 当前周期侦察假设列表。 |

`EmitterHypothesis` 字段：

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `hypothesis_id` | 假设记录唯一标识。 | 用于外部跟踪同一假设。 |
| `candidate_classes` | 候选类别列表，按置信度降序。 | 表示可能的辐射源类别。 |
| `mode` | 工作模式假设：未知、搜索、跟踪、制导。 | 供战术态势和威胁判断使用。 |
| `threat_level` | 威胁等级：低/中/高。 | 供外部告警或优先级排序使用。 |
| `bearing_az_deg` | 方位线方位角，单位 deg。 | 表示辐射源方向估计。 |
| `bearing_el_deg` | 方位线俯仰角，单位 deg。 | 表示辐射源俯仰方向估计。 |
| `bearing_std_deg` | 方位测量标准差，单位 deg。 | 表示方向估计不确定性。 |
| `confidence` | 假设置信度，范围 `[0, 1]`。 | 表示该假设可信程度。 |
| `last_seen_cycle` | 最近命中周期号。 | 表示该假设最近一次被观测支持的周期。 |

### 4.6 `TruthEvaluationFrame`

该通道用于真值评估，不应作为正式侦察输出的业务真值来源。

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `associations` | `TruthAssociationRecordList`。 | 观测记录与输入真值辐射源之间的评估关联。 |

`TruthAssociationRecord` 字段：

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `observation_id` | 观测记录 ID。 | 指向 `observation_output.observations` 中的观测。 |
| `truth_emitter_id` | 真值辐射源 ID。 | 对应输入 `EsrSceneEmitter::emitter_id`。 |
| `matched` | 是否匹配成功。 | 表示该观测是否关联到真值辐射源。 |
| `confidence` | 匹配置信度。 | 表示评估关联可信程度。 |

### 4.7 `EsrCycleResult`

| 字段 | 说明 | 影响/用途 |
| --- | --- | --- |
| `input_cycle_index` | 本次调用输入周期号。 | 失败结果与 trace 归属。 |
| `output_frame` | 当前周期输出帧。 | 三通道业务输出主体。 |
| `validation_issues` | 当前周期输入校验结果。 | 调试输入问题。 |
| `has_validation_error` | 是否存在 error 级输入问题。 | 为 true 时本周期通常不会执行 pipeline。 |
| `executed_this_cycle` | 当前调用是否真正执行 pipeline。 | 判断输出是否来自本周期计算。 |
| `reused_previous_output` | 当前 `output_frame` 是否复用上一有效周期输出。 | 输入校验失败且已有上一帧时可为 true。 |
| `abort_reason` | 下游链路 abort 原因。 | 取值包括 `kNone`、`kValidationRejected`、`kRuntimeStateRestoreRejected`、`kOutputContractViolation`。 |

## 5. 三模块输出对照

| 模块 | 便捷输出帧 | 聚合结果 | 输出主体 | 主要用途 |
| --- | --- | --- | --- | --- |
| AR | `TrackOutputFrame` | `RadarCycleResult` | `TrackStateSnapshotList` | 目标航迹、生命周期、位置速度、干扰标记、控制/关联质量观测。 |
| EOS | `EosOutputFrame` | `EosCycleResult` | `EosDetectionRecordList` | 光电目标探测、通道 SNR、扫描相位和门限判决。 |
| ESR | `EsrOutputFrame` | `EsrCycleResult` | 观测/假设/真值评估三通道 | 电子侦察观测、辐射源假设、威胁等级和评估关联。 |

## 6. 使用建议

- 若输出需要进入业务系统，优先消费各模块的便捷输出帧主体：AR 的 `tracks`、EOS 的 `detections`、ESR 的 `observation_output` 与 `emitter_output`。
- 若输出需要进入诊断、回放、测试或在线健康监控，优先使用 `StepWithResult()`，并记录 `validation_issues`、`executed_this_cycle`、`reused_previous_*` 和 `abort_reason`。
- AR 可用查询辅助接口按外部目标 ID、关联键、生命周期状态和干扰标记过滤轨迹。
- ESR 的 `truth_evaluation_output` 是评估通道，不应混入正式侦察假设输出。
- 当 `executed_this_cycle == false` 且 `reused_previous_* == true` 时，输出帧可用于连续显示，但不代表本周期计算成功。
