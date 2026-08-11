---
Status: active
Last-reviewed: 2026-08-07
Authority: sbirs_sensor 模块级边界、非目标、能力决策与变更规则
Answers: SBIRS 有哪些模块级边界、哪些能力刻意不实现及为什么、输出归属规则、变更规则
---

# SBIRS 模块边界

本文承载 SBIRS 的模块级边界、输出归属、能力决策（刻意不实现什么及为什么）、非目标和变更规则。
算法级边界（状态机转移、EKF 参数、ATP 速率等）见 [algorithms.md](algorithms.md)。

## 电源状态单源边界

电源状态单源（COMMON-OQ-4 字段提升）：`SbirsSessionConfig::sensor_enabled` 是唯一权威（mission 域无
电源字段）；运行时唯一入口为 `SbirsRuntimeConfigPatch::has_sensor_enabled`，builder 为 `WithSensorEnabled`，
replay schema 的 session config 表承载 `sensor_enabled`。旧名 `power_on`/`has_power_on`/`WithPowerOn`
不得回流（见 `tests/contract/check_cross_domain_naming.cmake` 阻断 7 与 contract.md §电源状态单源契约）。

[evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test]
[evidence: tests/unit/sbirs_sensor/sbirs_runtime_config_resolver_test]
[evidence: tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test]

## 输出与仿真归属（三层模型）

SBIRS 遵守三层输出模型（contract.md §三层输出模型）：

| 层级 | 入口 | 责任 |
|---|---|---|
| 原始系统输出层 | `Step()` 返回的 `SbirsOutputFrame` | 1q 仿真传感器主输出 |
| 结构化执行结果层 | `StepWithResult()` 返回的 `SbirsCycleResult` | 输出帧、执行状态、校验、abort reason、诊断摘要 |
| 开发调试视图层 | `SbirsOutputDebugViewBuilder` / `SbirsDetectionLifecycleRecorder` / `SbirsExclusionCauseRecorder` | 人读状态、生命周期事件、排除原因差分、输入实体回填 |

非执行周期（`status != kCompleted`）表示本周期没有产生新的目标观测事实。所有 recorder（
`SbirsDetectionLifecycleRecorder` 与 `SbirsExclusionCauseRecorder`）在该边界返回空事件
列表并保持全部累积状态；validation rejection 不得虚构 `Lost`、`NotDetected` 或 `TargetMissingFromInput`。
下一合法检测继续按拒绝前状态产生 `Updated`。

### 非执行周期统一不复用（五模块统一规则）

SBIRS 非执行周期（校验失败/执行 abort）的 `Step()` 与 `SbirsCycleResult.output_frame` 一律返回**默认空帧**
（`cycle_index=0`、空检测），**永不复用**上一有效输出。调用方用 `StepWithResult().status` /
`abort_reason` 判断周期状态。`reused_previous_output` 字段已删除。

[evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]

### 三写约束（abort_reason + issues + 日志）

SBIRS 所有中止路径遵守 `session_contract.md` 规则 9 的三写模式与规则 14 的统一问题列表模型：

1. **结构化信号**：`SbirsCycleResult.abort_reason`（粗粒度枚举：`kValidationRejected`、`kSensorPoweredOff`）。
2. **结构化诊断**：`SbirsCycleResult.issues`（`SbirsIssueList`，细粒度 code 如
   `"sbirs.sensor_powered_off"`、`"sbirs.validation.invalid_satellite_position"`；
   条目携带 `phase` 来源标签与可选定位）。
   本模块 code 全集单一事实来源：`include/1q/sbirs_sensor/session/SbirsIssueCodes.h`（规则 14c）。
3. **人读日志**：`PROJECT_LOG_ERROR`。

`SbirsCycleResult` 只承载单一问题列表 `issues`：输入校验问题（`phase=kInputValidation`）与执行诊断
（`phase=kExecution`）同列表承载，不设 `validation_issues`/`has_validation_error` 平行字段。
校验拒绝时校验问题本身就是 error 级诊断（规则 9 写二），不再附加粗粒度条目（COMMON-OQ-9
收敛：校验拒绝路径不再调用 `RecordAbort`，显式补齐 `abort_reason` 与日志）。

三写由 `SbirsDiagnosticUtils::RecordAbort` 统一执行（phase 由中止原因推导）；校验拒绝路径
不调用 RecordAbort（校验问题本身即写二）。

正常执行周期（`status == kCompleted`）的按目标排除诊断（kInfo，code 如
`"sbirs.target_out_of_wfov"`）**不属于三写**（session_contract.md 规则 13b）：仅承载排查信息，
由 pipeline 内联写入、controller 层并入 `SbirsCycleResult.issues`，调用方按规则 12
落盘 DebugView 时自然携带；不改变周期状态与 DebugView 状态语义。周期级 `PROJECT_LOG_INFO`
执行摘要（规则 13a，格式 `[SbirsPipeline] cycle_index=… scan_az=… detections=… excluded=…`）
在 pipeline `RunCycle` 每次实际执行后输出，仅人读，不用于状态判断（规则 3）。
**实体机器可读关联（规则 14e/13b）**：排除诊断结构化携带 `location = {kSceneEntity, target_index}`
（`MakeExclusionIssue` 由 pipeline 主循环索引赋值），供 `SbirsExclusionCauseRecorder` 按实体
关联消费。SBIRS 排除诊断涵盖 4 个 code：遮挡/距离带（具体门，cause 恒 kNone）、视场/SNR
（聚合门，有细分 cause）。
**排除原因跨周期差分（规则 13e）**：`SbirsExclusionCauseRecorder` 对持续被排除目标做
`(code, cause)` 对差分，产出 A2 进入/A3 原因变化/A4 退出事件。差分键为组合对（非纯 cause），
正确捕获遮挡↔距离带切换（同为 kNone、code 不同）的 A3 变化。纯观测只读 `result.issues`
（按 `location.kind == kSceneEntity` 过滤），与 `SbirsDetectionLifecycleRecorder` 并列
（独立 Attach/驱动/GetLastEvents），注册与否不影响执行语义（规则 11c）。**消失目标边界**：
recorder 只遍历当前周期 `input.scene`，目标从输入消失时其排除状态条目保留（不会被 A4 清除，
与既有 `SbirsDetectionLifecycleRecorder` 的"消失目标状态保留"行为一致）；重现为 A3 而非 A2。

### 输出规则（WFOV/NFOV 状态仅决定当前周期哪些目标输出检测记录，不进 raw output 字段）

1. WFOV 阶段（`WideCandidate`）：输出 WFOV 检测成功目标的检测记录，位置为带误差值。
2. NFOV 指向等待周期（ATP 未 settled）：继续输出 WFOV 检测记录；attribution 携带已预留的
   `nfov_channel_id`。
3. NFOV 首次捕获成功：Estimated 输出带误差角度，Strict 输出真值，Sensor-like 用独立子流产生带误差角度
   和诊断距离；三者均记录明确 `tracking_source`。
4. NFOV 持续跟踪且门通过：Estimated 输出滤波后验，Strict 输出真值，Sensor-like 输出带误差观测。
5. NFOV 单周期门失败但未达丢锁阈值：raw output 无记录；attribution/debug/lifecycle 标记 `Coasting`
   并保留通道。
6. NFOV 门连续失败达阈值：raw output 无记录；result attribution 携带 `kNfovTrackingGateLost` 并释放锁定。
7. NFOV 首次捕获失败或 pointing timeout：raw output 不含失败记录；result attribution 携带
   `kNfovAcquisitionFailed` 或 `kNfovPointingTimeout`。
8. WFOV 级几何/SNR 排除（地球遮挡、距离门、视场外、WFOV SNR 低于 `wide_min_snr_linear`）：
   raw output 无记录、无 attribution；`SbirsCycleResult.issues` 携带 kInfo 排除码
   （`sbirs.target_occulted` / `sbirs.target_out_of_range` / `sbirs.target_out_of_wfov` /
   `sbirs.target_snr_below_threshold`，message 含 `target_id` 与关键量值）；DebugView 状态仍为
   `kNotInOutput`（规则 13b/c）。
9. **门内归因（规则 13b 归因条款）**：`SbirsIssueCause` 给出机器可读主因——视场门按越界轴细分
   （`kAzOutside` / `kElOutside` / `kBothAxesOutside`，与 `InRectangularFov` 同基准）；
   SNR 门为聚合门（距离²/大气透过率/目标签名折入单一门限），反事实判定主因（距离参考
   1000 km、大气全透过、目标签名取"使 SNR 恰达门限的签名"），损失最大者为
   `kDistanceLimited` / `kAttenuationLimited` / `kSignatureLimited`；遮挡与距离门为具体门
   （cause 恒 `kNone`），message 分别补遮挡余量（`ComputeEarthOccultationMarginM`，负值 =
   遮挡深度）与距带边余量。

仿真归属（detection id → 输入 target id/name）、debug view、lifecycle（found/lost）、replay 仅进
`SbirsCycleResult` 和调试视图层，不得混入 `SbirsOutputFrame`。状态机内部状态如需调试，通过稳定的
status/stage/reason/coasting/gate 语义派生，不直接公开 internal `SbirsTargetState` 枚举。

[evidence: tests/unit/sbirs_sensor/sbirs_cycle_output_builder_test]

### 归属字段边界（不进 raw output）

1. `SbirsSceneTarget.velocity_ecef_m_per_s`：目标速度真值，驱动 cue 延迟外推与动态滞后误差；进 replay。
2. `SbirsCaptureFailureReason` + `capture_failure_reason`：捕获失败/调度跳过/NIS 丢锁诊断；进 attribution
   与 lifecycle reason，进 replay。
3. 闭环跟踪诊断（实际光轴误差、几何/SNR 门状态、连续失败计数、coasting 标志）：进 attribution/debug/
   lifecycle/replay。
4. `nfov_channel_id`：NFOV 通道编号（-1 表示 WFOV/未占用）；进 attribution/lifecycle/debug，进 replay。

`SbirsCycleResult.abort_reason` 解码只接受当前枚举中的 `kNone` 与 `kValidationRejected`；未知数值在修改
输出前拒绝，已删除的 reason 不提供 replay 数值兼容路径。

[evidence: tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test]

## 专项序列验证边界

`batch_validation::sbirs_sensor` 覆盖双目标双锁、三目标单锁交接、持续机动引发 NIS 丢锁与重捕获、
带横向速度的 cue latency、地球遮挡再现、standby 任务重定向和无效输入恢复。

影响退出码的硬检查：
1. 预期未执行周期数。
2. 单周期 NFOV 通道唯一性。
3. 场景特定的通道/目标数量与中断-恢复结果。
4. NIS 超门/丢锁/重捕获事件。
5. `replay_complete` 和 `failure_marker_count`。

batch 未直接证明通道跨周期稳定映射、无效输入前零 mutation、恢复后滤波/通道连续性或与 clean session
等价；这些性质只能由对应的 unit/integration/replay 测试作为证据。红外链路物理趋势仍为 warning。

## 能力决策与重新进入门

当前定位是**系统级、可解释、可确定性 replay 的 SBIRS-inspired 仿真**。下表不是 backlog 或优先级；
没有可复现失败、误差预算和验收门的候选不计入当前架构债务，也不得仅凭"真实性可能提高"进入生产。

| 能力 | 当前决策 | 证据或重新进入 Stage A 的必要条件 |
|---|---|---|
| 捕获后闭环 ATP 跟踪 | implemented | 已按逐通道状态接线；predict→advance→gate→correct、coasting、丢锁、snapshot/replay 均有测试证据 |
| 时间相关姿态抖动与指向误差 | implemented | 已接入共模 WFOV/NFOV 与逐通道 NFOV；零幅默认，不等同完整整星控制器 |
| CA cue predictor | reject for wiring | 标称噪声和较长 latency 下放大误差、降低捕获率（73.91%→41.30%），未通过零回退门；不得接入 config/schema/pipeline |
| 简化整星姿态动力学与执行机构约束 | defer | 必须先给出当前角度域模型无法满足的可复现失败和误差预算，再冻结共享平台姿态与逐通道光轴所有权 |
| 多通道机械耦合与共享姿态资源 | defer | 必须证明独立 LOS 假设导致可观测错误，并具备确定性仲裁、失败归属和 snapshot/replay 验收矩阵 |
| 探测器像元、背景杂波与图像帧 | defer outside current product boundary | 仅在产品目标转为图像检测/TBD/NCC 且具备 PSF/MTF、焦距、像元几何、背景和独立物理真值时重开；归属独立 imaging 子系统 |
| 高精度轨道传播 | reject in sensor ownership | cycle input 已提供同一时标下的平台/目标状态；默认归属场景或平台动力学模块 |
| 地面任务规划、区域重访与星座协同 | reject in sensor ownership | 属于任务规划/星座资源域，应通过 session config/input 驱动传感器 |
| 复刻真实 SBIRS 保密参数或处理链 | reject | 不可审计、不可验证；只使用可追溯公开资料、仓库内模型假设和独立测试证据 |

[evidence: tests/unit/sbirs_sensor/sbirs_cue_predictor_test]
[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

## 非目标

1. **图像级 TBD（Track-Before-Detect）**：第一版用 WFOV 单帧 SNR 门控判定可探测性，不做管道滤波、
   帧间能量累积或动态规划 TBD。理由：TBD 需要多帧图像缓存和速度空间搜索，是独立的图像处理子系统。
2. **模板匹配 NCC 窄视场捕获**：用 WFOV cue + 几何窗口 + SNR 门判捕获，不实现归一化互相关。理由：
   NCC 依赖图像级数据，当前几何+SNR 判定已覆盖捕获语义。
3. **在线残差驱动的自动滤波后端切换**：后端选择由显式配置决定，保证 replay 可复现。可接受的"智能"
   形态为只读 NIS 诊断 + 人工看报告改配置。
4. **Otsu/DBSCAN 多目标聚类**：按 `target_id` 独立维护状态机，不做自适应阈值分割或聚类。理由：
   聚类针对图像级检测点，当前目标来自输入场景的显式目标列表。
5. **完整整星姿态系统 / CA / 6D-9D 搜索 / 轨道预测 / 通道机械耦合**：捕获后 tracking 已使用实际
   actuator LOS 门控，但不扩展为完整整星姿态动力学（见能力决策表 defer 项）。
6. 不暴露用户自定义 pipeline、controller、状态机、环境模型或 foundation algorithm 类型。
7. 不把仿真目标 ID/name 混入 `SbirsOutputFrame` 的 raw detection。
8. 不把 debug view、lifecycle 或 replay 当作 1q 仿真传感器主输出。
9. 不为测试 mock 便利新增 public 扩展点。

## 设计变更规则

1. `SbirsOutputFrame`、检测记录字段、`SbirsCycleResult` 或 attribution/debug/lifecycle 语义变化，必须同步
   本文和输出边界测试；不得为复用 EOS consumer 而把 range、visible/fused SNR 塞进 raw output。
2. 状态机状态集合、转移条件、优先级规则变化，必须同步 algorithms.md 的状态转移图/转移条件表和
   `sbirs_state_machine_test`、`sbirs_scheduler_test`。
3. 气象影响列表、加权叠加公式 `A_total`、衰减进入 SNR 链路的方式变化，必须同步 algorithms.md 和
   `sbirs_environment_model_test`、`sbirs_radiative_transfer_test`。
4. 误差模型（5 类误差、加法/乘法合成、折射角与滞后公式）变化，必须同步 algorithms.md 和
   `sbirs_error_model_test`；同时检查对 NFOV cue 指向、首次捕获成功率的影响。
5. runtime patch 的可变字段、立即提交策略或状态机 capture/restore 规则变化，必须同步 data-flow.md、
   contract.md 运行期配置提交策略表和 runtime resolver 测试。
6. foundation 算法如果从 internal 变成 public API，必须在 `[evidence: ...]` 标注中记录扩展理由、稳定性
   约束和迁移影响，并检查与 EOS 对应算法的偏离是否有意。
7. 新增 debug/replay 字段时，必须保持真实输出、结构化结果和仿真辅助视图三层分离。
