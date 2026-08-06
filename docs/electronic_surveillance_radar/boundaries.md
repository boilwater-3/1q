---
Status: active
Last-reviewed: 2026-08-03
Authority: ESR 模块级边界、非目标与设计变更规则
Answers: ESR 有哪些模块级禁令与边界、哪些非目标、配置/扫描/输出/校验的特殊语义、文档变更规则
---

# ESR 模块边界

本文承载 ESR 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（拦截检测链、聚类、
假设关联）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

ESR 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/trace/replay 门面；`EsrSession` 是对外门面，
   pipeline/controller/environment service、runtime snapshot 和 generated replay header 不通过 public header 暴露。
2. runtime patch 经 `ApplyRuntimeConfigWithResult()` → `ResolveEsrRuntimeConfigPatch()` 解析：整域值先合并，
   leaf override 后应用，再对最终 mission 枚举、scan policy、基础 detection policy 和 environment 统一做
   一次领域校验。通过校验的 patch 立即写入 `resolved_config` 并同步到 pipeline/environment；被拒绝的 patch
   原子无污染。ESR 属立即提交类，配置单向落定，不提供 session 层回滚。
3. 输出遵守三层模型：`EsrOutputFrame` 发布两个去真值化通道（observation_output、emitter_output）与一个
   设备状态标量 `scan_azimuth_deg`（本周期波束中心方位，平台参考系，见"输出与可观测性边界"），
   `EsrCycleResult` 以 `status` 承载本周期执行真相。

## scan_rate_hz 校验边界（反直觉，勿按"波束更新率"理解）

`scan_rate_hz` 的单位不是波束更新率或角速度，而是**每秒完成的完整二维 scan pattern 循环数**。pipeline
持有归一化扫描相位 `[0, 1)`：本周期先用 `floor(phase × pattern_size)` 选择波束，再累加
`scan_rate_hz × dt` 并回绕。因此变步长不会改变物理扫描速度；运行期只改速率会保留当前相位，改变
窗口边界、顺序或起始位置则重置到起始波束。设备关闭时扫描相位冻结。静态配置和 runtime patch 均拒绝
非有限或非正速率。

该标量 pipeline 每 `Step` 只判定当前相位对应的一个波束，不在单周期内积分连续扫过的全部驻留。
`scan_rate_hz × dt` 为整数时，相位会按物理周期回到同一点；需要观察完整扫描覆盖的场景必须选择能够
解析扫描相位的步长/速率组合，不能依赖 cycle index 隐式轮转波束。

[evidence: tests/unit/electronic_surveillance_radar/esr_controller_runtime_state_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_session_config_builder_test]
[evidence: tests/integration/cross_domain/multi_model_scenario_test]

## dt_sec 校验边界（反直觉，勿按"四模块一致"补齐）

`ValidateEsrCycleInput` 对 `dt_sec` 仅校验有限性 + 正值，**故意不含** EOS/SBIRS 的
`dt_sec ≤ 10/frame_rate_hz` 上界。ESR 是被动侦察接收机，配置中没有 `frame_rate_hz` 概念——其节拍由
`scan_rate_hz × dt` 决定（见上文扫描相位模型），变步长不改变物理扫描速度。dt 的合理性由
`scan_rate_hz × dt` 能否解析扫描相位、RF emission 帧窗口一致性（`window_duration_s == dt_sec`）等
ESR 域量约束，不适用一个全局 frame_rate 上界。该差异已由校验链实测、`EsrCycleInput` 无 frame_rate
字段、本节扫描相位文档三方共同固化。

不得为"四模块一致"给 ESR 加 frame_rate 上界。

[evidence: src/electronic_surveillance_radar/session/EsrInputValidation]

## 扫描窗口与坐标系语义（反直觉）

### 扫描窗口两种互斥解释

- `use_explicit_scan_bounds=true` 时，四个显式起止角必须全部有限，并分别满足
  `scan_start_az_deg < scan_end_az_deg`、`scan_start_el_deg < scan_end_el_deg`；显式边界生效，中心角字段被忽略。
- `use_explicit_scan_bounds=false` 时，中心角结合硬件扫描范围推导窗口；即使显式字段为 NaN/Inf，也因未被选择而忽略。

静态 session validation 按所选模式验证，不能把非法的显式模式静默退化成中心模式。运行期 resolver 先合并
full-domain mission，再应用 leaf override，最后只对合并后的 scan policy 做一次统一校验和解析；因此
full-domain 中的非法中间值可被合法 leaf 覆盖，但任何留在最终策略中的非法值都会原子拒绝整份 patch。
提交中心角时关闭显式模式，提交显式起止角时开启显式模式；显式提交 `enabled=false` 时忽略该 inactive
payload 中的四个边界字段，并按中心角、硬件扫描范围和天线安装角重建窗口；将被启用的中心角若非有限
则原子拒绝，不能保留旧显式执行态窗口。runtime 开启显式模式与静态 validation 一致，严格要求两轴
`start < end`，不接受 equal/swapped 输入。因此最近一次被明确选择的合法表达拥有窗口语义。

[evidence: tests/unit/electronic_surveillance_radar/esr_session_config_builder_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_runtime_config_resolver_test]

### 扫描配置的坐标系语义（天线坐标系）

`EsrScanPolicyConfig`（mission 域）中的 `scan_center_az_deg` / `scan_center_el_deg`，以及显式模式下的
`scan_start_az_deg` / `scan_end_az_deg` / `scan_start_el_deg` / `scan_end_el_deg`，均定义在**天线坐标系**中，
即默认假定天线 boresight 与接收机参考轴对齐。`EsrHardwareConfig::antenna_mount_az_deg` /
`antenna_mount_el_deg`（hardware 域）描述天线相对平台参考轴的安装偏置；`ApplyScanPolicy` 在解算实际接收
波束指向时，会从 mission 域扫描角中**减去** hardware 域安装偏置。因此 mission 配置给出的扫描角与最终
平台系指向之间相差一个 hardware mount 偏移：消费方只看 mission 配置无法推断实际平台系扫描方向。这是
当前固化语义，不接受在 mission 域直接填写平台系角度。

## 电源状态单源（COMMON-OQ-4 字段提升）

`EsrSessionConfig::sensor_enabled` 是电源唯一来源（mission 域无电源字段），`has_sensor_enabled` 叶子是
运行时电源唯一入口。运行期关闭传感器不重建会话，只通过 patch 立即生效。

[evidence: tests/integration/electronic_surveillance_radar/esr_session_test]

## 输出与可观测性边界（模块级）

`EsrController` 是输出帧装配和最近有效帧缓存的唯一 runtime owner；它直接写入 cycle/batch header，
移动 pipeline 的观测/假设结果，并只在成功执行后推进 batch。模块不维护第二个 output-manager 状态或
装配路径。

`batch_id` 在 public `EsrOutputFrame`、controller 累积状态、FlatBuffers replay schema、codec 和 comparator
中统一为 64 位无符号值；codec 不得把它缩窄到 32 位，大于 `UINT32_MAX` 的值必须无损 roundtrip。

`scan_azimuth_deg` 是输出帧的设备状态标量：pipeline 在检测阶段按当前扫描相位选中波束后，以
"波束中心方位 + 天线安装偏置"（即 mission 域扫描角所在参考系的实际指向，与 RF 前端接收求解同算式）
写入检测输出，经 `InterceptPipelineResult` 由 controller 装配进 `EsrOutputFrame`。该字段已纳入 FlatBuffers
replay schema、codec 与 comparator；非执行周期（校验失败/关机）随默认空帧输出 0，消费方须以
`status == kCompleted` 守卫读取，不能把 0 当成真实方位。

truth identity、外部坐标适配输出与 debug view 不属于 ESR 公共输出合同；消费者只使用观测和 hypothesis
的估计字段。不通过日志文本判断状态；调用方应使用 `EsrCycleResult`。

### 非执行周期统一不复用（五模块统一规则）

ESR 非执行周期（校验失败/设备关机）的 `Step()` 与 `EsrCycleResult.output_frame` 一律返回**默认空帧**
（`cycle_index=0`、`scan_azimuth_deg=0`、空 observation/emitter 输出），**永不复用**上一有效输出。
调用方仅凭 `Step()` 返回值即可判定本轮无新观测；执行真相（rejected vs powered-off）须走
`StepWithResult().status` / `abort_reason`。

注意 controller 内部在 validation reject 时确实会保留 `GetLatestInterceptOutputFrame()`（旧帧），
但 `EsrSession::BuildCycleResult` 只在 `status == kCompleted` 时把它写入 public `EsrCycleResult.output_frame`，
因此 controller 的旧帧缓存是内部状态机行为，不构成 public `Step()` 的输出回退路径。

[evidence: tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailure]
[evidence: tests/unit/electronic_surveillance_radar/esr_controller_runtime_state_test]

### 三写约束（abort_reason + diagnostics + 日志）

ESR 所有中止路径遵守 `session_contract.md` 规则 9 的三写模式：

1. **结构化信号**：`EsrCycleResult.abort_reason`（粗粒度枚举）。
2. **结构化诊断**：`EsrCycleResult.diagnostics`（`EsrDiagnosticIssueList`，细粒度 code 如 `"esr.rf_receiver_rejected"`）。
3. **人读日志**：`PROJECT_LOG_ERROR`。

三写由 `EsrDiagnosticUtils::RecordAbort` 统一执行，在 `EsrSession::BuildCycleResult` 和 `RunCycle` 中调用。

## 专项序列验证边界

`batch_validation::electronic_surveillance_radar` 覆盖近同频辐射源角度交叉、密集辐射源静默、
ESM/RWR/HGESM 切换、显式扫描边界重定向、关机恢复和无效输入恢复。所有场景的 trace replay 失败、输出
分叉或比较数量不一致都会使批量验证失败。

影响退出码的硬检查：各场景预期的非执行周期数；无效输入场景的 failure marker 数；无效显式边界 patch
的原子拒绝；角度交叉场景的建立/最终 hypothesis id 集合连续性；关机恢复和无效输入恢复场景在第一个
重新执行周期立即保持建立阶段 hypothesis id 集合。

batch 不含 truth matching、legacy lifecycle recorder 或旧输入适配器；每个场景显式设置与载频匹配的窄带
tuning window，因此 sweep 与 sequence 都必须产生真实观测——零观测不再被当作可接受的空验证。场景 ID、
结构化 check 和运行方式由 `examples/batch_validation/README.md` 维护。

性能验收分为两个不可互相替代的 Release 场景：稀疏检测场景以 64 个外部 RF 发射、1000 个 AR 目标和
1000 个 ESR 发射验证 RF 前端/分辨账本；密集检测场景要求每周期 1000 条 raw observation，并至少保留
90% 的预处理观测、cluster 和 hypothesis，同时限制活跃 hypothesis 不超过输入规模的 5 倍。两者都在
20 个 lifecycle 预热周期后连续测量 100 周期并要求 P95 `< 100 ms`，拒绝周期不能计入性能样本。密集场景
还逐周期采样活跃堆分配量，以最初和最后 20 个测量周期的中位数比较稳态增长，允许上限为 4 MiB；不得用
进程历史峰值 RSS 或 hypothesis 数量替代活跃模型状态的内存验收。

[evidence: tests/performance/cross_domain/rf_interference_performance_test]

## 非目标

1. 不暴露用户自定义 pipeline/controller/environment service。
2. 不把 truth identity 或预计算受扰结论合并进真实 observation/hypothesis 输出。
3. 不把 pipeline internal context、runtime snapshot 或 generated replay header 当成 public API。
4. 不通过日志文本判断状态；调用方应使用 `EsrCycleResult`。
5. 不以 `legacy / engineering` 标签改变接收物理链；旧 public jammer 摘要和 adapter 已删除，不属于公共合同。

上述边界由文档结构守护和 public API 契约测试守护。

[evidence: tests/contract/check_public_api_boundary]

## 设计变更规则

1. Observation/hypothesis 或 cycle-input 位姿字段变化必须同步精确 replay roundtrip 测试。
2. Gate、preprocess、cluster、association 语义变化必须同步 algorithms.md 和对应 focused tests。
3. Runtime patch、snapshot 或状态边界变化必须同步控制器状态测试与 data-flow.md 状态所有权。
4. 输出通道边界变化必须同步 output boundary contract 测试。
5. 验证优先使用 `unit::electronic_surveillance_radar`、`integration::electronic_surveillance_radar`、
   `replay::electronic_surveillance_radar`、`batch_validation::electronic_surveillance_radar` 与 ESR guards。
