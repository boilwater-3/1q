Status: draft

# ESR RF v2 接收机重构 — 物理语义冻结合同

**Freeze-Date:** 2026-07-24

**Freeze-Baseline:** `codex/esr-rf-v2-receiver`（head `d09ad6eb refactor(env): remove dead SpaceWeatherContext inputs across AR/ESR`）

**Authority:** 非规范性证据记录；不得替代 `docs/common/contract.md`、
`docs/electronic_surveillance_radar/design.md` 或 `docs/electronic_countermeasure/design.md`。
本文件是 `evidence-first-freeze-contract` 工作流的 Stage A 输出，作为后续 Stage B 分批实现的依据。

**Scope:** 仅 ESR 接收机物理语义、输入合同与观测模型。SAR/EOS/SBIRS、复数 IQ、新欺骗/转发算法、
flight_dynamic 生产代码不在本轮范围。

## 0. 前提：已有 commit 已完成的项（不再进冻结矩阵）

`codex/esr-rf-v2-receiver` 上的 commit 已执行了原七步计划的大部分。下列项在新合同里被表述为
"待重构"，但代码审计确认**已是 DONE**，本轮不重复冻结，只作为背景：

| 项 | 当前证据 | 状态 |
|---|---|---|
| 前端 / 调谐通道 / 分辨单元三层账本 | `src/electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.cpp:113-132`（`TryAggregateRfIncidentPower` 宽带功率聚合 + `receiver_saturated`，独立于当前调谐窗口） | DONE |
| `interference` 字段类型 = `RfEmissionFrame` | `include/1q/electronic_surveillance_radar/session/EsrCycleInput.h:29` | DONE（字段**名**仍叫 `interference`，见 F1） |
| `is_jammed` / truth identity / `scene_emitters` / `EsrJammerSource` / `JammingAggregator` / deception false observation | 全仓 grep 在 ESR `src/`+`include/`+`tests/` 零命中（`RfInterferenceMode`/`RfEmissionSegment` 仍存于共享 `RfLinkBudget`，服务其他模块，与本轮无关） | DONE（ESR 范围） |
| 单一状态枚举；拒绝/关机不复用历史输出 | `EsrCycleResult.h`：`kCompleted/kRejected/kPoweredOff`；`reused_previous_output` 不存在于 ESR | DONE |
| RNG 按语义域拆分；snapshot 单一所有权 | `InterceptDetectionExecutor.cpp:24-25`（detection/AoA 两域）；`EsrController.h:27-39` 嵌套 `InterceptPipelineRuntimeState`（非重复） | DONE |
| replay schema 已 v2-clean；codec 无损 round-trip | `schemas/replay/esr_replay.fbs`、`EsrReplayFlatbufferCodec.cpp:25-130,150,181` | DONE |
| ESR-RX-01（逐候选重指向天线） | `BuildReceiverSite` / `ProcessSingleEmitter` 已不存在；每周期单一冻结 boresight | DONE |

因此真正需要冻结的是**剩余 9 项**（F1–F9）。

## 1. Stage A 证据矩阵

| 冻结项 | 假设 | 证据源 | 探针/测试 | 通过判据 | 拒绝/收窄判据 | 决策 |
|---|---|---|---|---|---|---|
| **F1 字段改名 `interference`→`rf_emissions`** | 字段名 `interference` 预判发射为"干扰"，违反意图中立；改名让合同字面中立 | `EsrCycleInput.h:29`（字段名 `interference`，类型已是 v2）；`MutableEsrContext.h:55,73` getter；`EsrInputValidation.cpp:82,89`；`InterceptDetectionExecutor.cpp:250`；trace JSON 已用 `rf_emission_count`（`EsrTraceSession.cpp:22`） | grep `\binterference\b` ESR 闭包；构建 + ESR 全测试 | 纯机械改名；类型不变；项目未上线无 wire 兼容约束 | 若改名破坏公共 ABI 且下游不可控 → 无（项目未上线） | **pass（narrow）— 延后 Stage B** |
| **F2 RF/带宽/PRI/PW 测量模型替代真值复制** | ESR 输出必须是接收估计而非真值副本 | `InterceptDetectionExecutor.cpp:352-357`：`rf_hz=center_hz`（=waveform center + Doppler）、`bandwidth_hz=occupied_bandwidth_hz`、`pri_s`/`pulse_width_s` 直抄 waveform；仅 `*_std` 走 `1/sqrt(SNR)`（:336-337,358-362） | 特征测试：固定 SNR/驻留，断言 hypothesis 中心 ≠ truth 中心、误差落在发布 σ 内、随 SNR 单调下降 | 均值由测量模型生成（SNR/驻留/脉冲数/带宽/波束宽度驱动），真值参数不进入均值；发布协方差与采样同模型 | 若无可标定估计器且输入缺标定数据 → 收窄到 RF+带宽，PRI/PW 保持复制并标注 known-limit | **pass — 核心物理语义** |
| **F3 观测类型拆分 pulse/energy** | 单一 `EmitterObservation` 无法干净表达 CW/noise/sweep 的 energy-only 语义（PRI/PW 留 0 占位） | `InterceptDetectionExecutor.cpp:354-357`（仅 kPulseTrain 填 PRI/PW，余者默认 0）；`EmitterObservation.h` 单结构 | 契约测试：energy 观测无 PRI/PW 字段；pulse 观测必有；聚类按 waveform class 分流 | 两类型 + waveform-class-appropriate 字段；聚类按 class 分流 | 若拆分使公共面翻倍却无可测性收益 → 收窄为内部 variant，公共保持单一 | **narrow — 依赖 F2** |
| **F4 O(N²) → 分辨单元账本** | `:289-310` 外层逐信号、内层逐 other 求 angular-cell 重叠 + channel power，是 O(N²) 且逐对重算 | `InterceptDetectionExecutor.cpp:289-310` | 性能门：64 外部 + 1000 场景发射、100 周期 P95 < 100 ms | 按(到达时间,频率,角度)一次分组；单元内最强=候选，余=干扰；输入顺序无关 | 若分组改变可测 SINR 相对当前逐对结果 → 收窄为稳定排序 + 频率索引预建，保留逐对语义 | **pass — 性能+正确性** |
| **F5 linear-sweep 瞬时频率驻留** | `ResolveCenterFrequencyHz` 把扫频塌缩为 `(start+stop)/2`，丢失时间相关瞬时频率 | `InterceptDetectionExecutor.cpp:121-128` | 特征测试：扫频跨越通道边缘应部分驻留，而非被指派中点 | 按时间 bin 的瞬时频率驱动通道占用与 RF 估计 | 若需 IQ 而输入无 IQ → 收窄为时间分段中心频率，不引入 IQ | **narrow** |
| **F6 ECEF 可定位性前置校验** | `EsrInputValidation.cpp:33-44` 仅查 finite + 非零 id；finite-but-不可定位 ECEF 通过校验后在 `TryResolveLookAngles` 失败 → 运行期拒绝，留下"校验通过、运行期才拒"裂缝 | `EsrInputValidation.cpp:33-44`；`InterceptDetectionExecutor.cpp:159,285` | 单测：地心/超地球半径 ECEF 被判 `kRejectedInvalidInput` 而非 `kRejectedRfLink`，状态不推进 | 不可定位 ECEF 在输入校验即拒，`status=kRejected`，无状态推进 | 无 | **pass** |
| **F7 batch 指标清理** | `steady_truth_match_rate_mean`（恒 0 死指标）与 `jammed`（实为 `receiver_saturated` 别名）与去真值化、无 jamming 布尔合同冲突 | `examples/batch_validation/esr_batch_validation.cpp:219,249,263,395,464-501,583-598`；`examples/batch_validation/README.md:141,184-188` | batch run 断言无 truth_match_rate/jammed 列；有 SNR/协方差/obs-count/饱和趋势 | 指标集 = 执行状态/观测数/SNR+协方差趋势/饱和/replay/结构化恢复 | 无 | **pass** |
| **F8 跨域 v2→legacy 转换删除** | `ConvertRfV2ForLegacyEsr`（:209-243）与 `engineering_emissions` 赋值（:1520-1522）引用已删除字段，仅在被禁用的 flight-dynamic gate 下编译；直传（:1396）才是活路径 | `tests/integration/cross_domain/multi_model_scenario_test.cpp:209-243,1396,1514,1520-1522` | 开 `ONEQ_TEST_FLIGHT_DYNAMIC_ENABLED` 构建，legacy 块无法编译 | 删除 `ConvertRfV2ForLegacyEsr`；AR/ESR 都直传 `interference=emission_frame` | 无 | **pass** |
| **F9 `TryComposeRfEmissionFrame`** | AR + ECM + 外部源并发需原子合帧 | 今日唯一消费者是单源直传（ECM → ESR/AR `interference`） | 出现真实多源消费者 | — | 无真实消费者 | **defer** |

## 2. 决策汇总

- **pass（本轮冻结，进 Stage B 合同）**：F2, F4, F6, F7, F8
- **narrow（依赖 F2，进 Stage B 合同受限子集）**：F1（本轮延后，用户裁定）、F3, F5
- **defer（无证据，记录下一步探针）**：F9

## 3. Stage B 冻结合同

**已证需求**：ESR 输出必须是接收机估计（均值由 SNR/驻留/脉冲数/带宽/波束宽度驱动，发布协方差与
采样使用同一模型）；真值参数不进入 observation/hypothesis 均值。

**允许范围**：
- `src/electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.{h,cpp}`（F2/F4/F5/F6 核心）
- `include/1q/electronic_surveillance_radar/session/EmitterObservation.h` 及其拆分（F3）
- `src/electronic_surveillance_radar/pipeline/MutableEsrContext.{h,cpp}`、`EsrRfV2FrontEnd.{h,cpp}`（F4/F5 配套）
- `src/electronic_surveillance_radar/session/EsrInputValidation.cpp`（F6 前置）
- `examples/batch_validation/esr_batch_validation.cpp` + `README.md`（F7）
- `tests/integration/cross_domain/multi_model_scenario_test.cpp`（F8）
- 配套 `tests/unit|contract|replay/electronic_surveillance_radar/*`、trace/replay schema 字段同步

**明确出范围**（本轮与后续 Stage B 都不动，除非新证据）：
- 公共 `include/1q` ABI（`RfEmissionFrame`/`RfSceneFrame` 定义保持不变）
- `TryComposeRfEmissionFrame` / `TryMergeRfEmissionFrames`（F9 defer）
- 共享 v1 `RfLinkBudget`（`RfEmissionSegment`/`RfInterferenceMode` 仍服务 SAR/EOS/SBIRS/AR）
- flight_dynamic 生产代码（仅测试有 legacy 残留，见 F8）
- ESR 主门面 `Step()`/`StepWithResult()` 签名与返回类型

**行为边界**：
- **输入**：非法/不可定位 ECEF、帧窗口不匹配、重复 emission identity → `kRejectedInvalidInput` /
  `kRejectedRfFrame`，不推进扫描相位、调谐索引、observation/hypothesis ID 或随机子流。
- **输出**：饱和 = `kCompleted + receiver_saturated`，空观测，但按正常物理周期推进扫描、调谐和
  missed-detection 状态。
- **意图中立**：`interference` 字段名本身预判发射为"干扰"，应改名为 `rf_emissions`（F1）；同一帧中的
  发射均为实际 RF 事实，只有在某分辨单元内不可分辨时另一发射才成为该候选的干扰功率。
- **去真值化**：truth identity 永不进入 observation/hypothesis；参数值只以带误差的接收机估计形式出现。

**验收门（Stage C）**：
- 单程距离翻倍接收功率下降 `6.0206 dB`；功率翻倍增加 `3.0103 dB`。
- active receive beam 与链路接收增益使用同一方向事实。
- 带外强信号可导致前端饱和，但不应被当前调谐通道错误当作可测 emitter。
- 两个发射只有在时间、频率、角度均不可分辨时才互相降低 SINR；pulse 错时、扫频错频、角度可分离均
  不得虚构干扰。
- 所有测量误差随 SNR、驻留和波束宽度单调变化。
- 输入拒绝不推进 batch、扫描相位、调谐相位、observation ID 或随机状态。
- 64 个外部 RF 发射、1000 个场景发射、100 周期 Release P95 `< 100 ms`，预热后无持续内存增长。
- ESR unit/integration/contract/replay/batch、cross-domain、public/install/C++11、docs guards 全部通过。

## 4. 依赖闭包（实现前已枚举）

**核心接收机**：
- `src/electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.{h,cpp}`（F2/F4/F5）
- `src/electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.{h,cpp}`（F4/F5 配套，已实现前端账本）
- `src/electronic_surveillance_radar/pipeline/MutableEsrContext.{h,cpp}`（F1 getter 改名）
- `src/electronic_surveillance_radar/session/EsrInputValidation.cpp`（F6 前置校验）

**观测/假设**：
- `include/1q/electronic_surveillance_radar/session/EmitterObservation.h`（F3 拆分）
- `include/1q/electronic_surveillance_radar/session/EmitterHypothesis.h`（F3 配套字段）
- `src/electronic_surveillance_radar/pipeline/InterceptPostProcessingExecutor.cpp`（聚类按 waveform class 分流）

**输入字段（F1，本轮延后）**：
- `include/1q/electronic_surveillance_radar/session/EsrCycleInput.h:29`（字段名）
- ~15 个生产/测试/示例赋值点（见下"测试需同步"）
- `schemas/replay/esr_replay.fbs:101`（wire key `interference`）
- `src/electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.cpp:150,181`（codec key）

**batch**：`examples/batch_validation/esr_batch_validation.cpp` + `examples/batch_validation/README.md`（F7）

**跨域**：
- `tests/integration/cross_domain/multi_model_scenario_test.cpp:209-243,1520-1522`（删 `ConvertRfV2ForLegacyEsr` + legacy 赋值）
- 同文件 `:1396,1514`（直传参考路径，保留）

**测试需同步**：
- `tests/unit/electronic_surveillance_radar/esr_rf_v2_front_end_test.cpp`
- `tests/unit/electronic_surveillance_radar/esr_rf_v2_detection_test.cpp`
- `tests/unit/electronic_surveillance_radar/esr_input_validation_test.cpp`
- `tests/unit/electronic_surveillance_radar/esr_controller_runtime_state_test.cpp`
- `tests/unit/electronic_surveillance_radar/esr_intercept_post_processing_test.cpp`
- `tests/contract/electronic_surveillance_radar/esr_public_api_convenience_test.cpp`
- `tests/contract/public_api/public_headers_smoke_test.cpp`
- `tests/integration/electronic_surveillance_radar/esr_session_test.cpp`
- `tests/replay/electronic_surveillance_radar/esr_replay_codec_roundtrip_test.cpp`
- `tests/replay/electronic_surveillance_radar/esr_replay_session_test.cpp`
- `tests/replay/electronic_surveillance_radar/esr_trace_session_adapter_test.cpp`
- `tests/consumer/esr_session_consumer.cpp`

**明确不动**：
- `include/1q/electromagnetics/RfScene.h`（`RfEmissionFrame`/`RfSceneFrame` 定义）
- 两份 `schemas/replay/esr*.fbs` schema（已 v2-clean，仅 F1 改名时动 wire key）
- `src/airborne_radar/`、`src/electronic_countermeasure/` 的 `emission_frame` 生产者
- `examples/electronic_warfare/*`（已用 v2 直传）
- `src/flight_dynamic/` 生产代码（仅测试有 legacy 残留，见 F8）

## 5. 建议提交边界（Stage B，待逐项放行）

按最小可构建依赖闭包顺序：

1. `refactor(esr): front-load ECEF geolocatability validation`（F6）— 独立，无前置。
2. `refactor(esr): rename interference to rf_emissions`（F1）— 机械闭包，与 F6 独立。
3. `refactor(esr): replace truth-copy measurements with receiver estimates`（F2）— 核心物理语义。
4. `refactor(esr): split pulse/energy observations and class-aware clustering`（F3，依赖 F2）。
5. `refactor(esr): time-resolved linear-sweep channel dwell`（F5）。
6. `refactor(esr): resolution-cell ledger replacing O(N^2) sweep`（F4）。
7. `refactor(esr): drop legacy batch metrics for SNR/covariance trends`（F7）— 独立。
8. `refactor(test): remove v2->legacy ESR cross-domain conversion`（F8）— 独立。

手工语义修改按最小可构建依赖闭包提交，不受五文件限制；仅 F1/F8 这类机械改动执行 1–2 文件试验
和每批最多五文件规则。

## 6. Residual / Follow-up freeze items

- **F9**：当出现 AR + ECM + 外部源真实多源并发消费者时，立项 `TryComposeRfEmissionFrame`（原子合帧、
  校验时间窗口、拒绝重复 identity）。本轮无证据，defer。
- **F2 收窄预案**：若 PRI/PW 无可标定估计器，先冻结 RF + 带宽测量模型，PRI/PW 保持真值复制并以
  `known_limit` 标注，留待后续证据。

## 7. Stage B 进度（2026-07-24 更新）

| 项 | 状态 | 提交 | 说明 |
|---|---|---|---|
| F6 | ✅ done | `ee2c2a54` | ECEF 可定位性前置校验；新增 `kUnlocatablePlatformEcef`。 |
| F8 | ✅ done | `1617a45c` | 删 `ConvertRfV2ForLegacyEsr` + 死赋值；gated 测试改直传 RF 帧。 |
| F1 | ✅ done | `94e803e9` | `interference`→`rf_emissions` 全 ESR 闭包改名（含 `.fbs` wire key）。 |
| F7 | ✅ done | `cf26935a` | 删 batch 死指标 `truth_match_rate`/`jammed`；修 `scenarios.csv` 列错位。 |
| F2 | 🔜 next | — | 测量模型替代真值复制。F3/F5 前置。 |
| F3 | pending | — | pulse/energy 观测拆分；依赖 F2。 |
| F4 | pending | — | O(N²)→分辨单元账本。 |
| F5 | pending | — | linear-sweep 瞬时频率驻留。 |

## 8. Stage B 实施中发现的新 follow-up（未在原冻结矩阵）

下列两项在 F1/F7 实施中由数据暴露，超出已冻结项范围，记录为独立排查项：

- **FU-1：sequence 场景既有结构化检查失败**。F7 收口时对比 F7 前后 `checks.csv` 字节一致，确认
  F7 未引入失败；但发现 5 个 sequence 场景的结构化检查既有失败：
  - `esr_seq_invalid_input_recovery`：`expected_nonexecuted_cycles`、`failure_marker_count`、
    `hypothesis_identity_continuity`。
  - `esr_seq_power_cycle`、`esr_seq_two_emitter_angular_crossing`：`hypothesis_identity_continuity`。
  - `esr_seq_dense_emitters_with_silence`、`esr_seq_mode_switch`：同类 recovery/replay 检查。
  
  这些失败与去真值化/RF 指标无关，属于 sequence recovery 逻辑或检查期望值本身，需独立排查。
  注意：batch 进程 `exit code` 仍为非零（`total_err>0 || checks.FailureCount()>0`），但这些失败在
  本轮和上一轮均稳定存在、未被任何 commit 触发变化。

- **FU-2：sweep 场景几何在稳态不产生观测**。F7 数据分析发现，所有 sweep 场景（r010–r100km /
  fc02–fc18 / occ0.10–0.95）在全部 40 周期内 `raw_observation_count`、`hypothesis_count`、
  `receiver_saturated` 恒为 0——ESR 完成执行却从不检测到辐射源。这是场景几何（辐射源与扫描波束
  时空关系）问题，不是接收机缺陷。直接后果：距离/占用率对观测/估计的趋势软断言无从建立
  （F7 据此删除了两个空趋势块）。要验证 F2 测量模型的真实误差行为，需要先让 sweep 场景在稳态产生
  可分辨观测；这会影响 F2 的验证策略（见 §9）。

## 9. F2 验证策略前提（由 FU-2 引出）

F2 的验收门要求"测量误差随 SNR/驻留/波束宽度单调变化"——但当前 sweep 场景在稳态不产生任何观测，
无法在 batch 层验证该单调性。因此 F2 的验证分两层：

- **单元/特征测试层**：直接构造固定 SNR/驻留/脉冲数/带宽/波束宽度的 incident link，断言观测中心 ≠
  truth 中心、误差落在发布 σ 内、随 SNR 单调下降。这是 F2 的主验证路径，不依赖 batch 场景几何。
- **batch 层**：F2 落地后 batch 仍以执行状态、replay、sequence 结构化恢复检查为主；观测/估计趋势
  软断言待 FU-2 解决后再恢复（与 F7 删除趋势块的注释一致）。

若 FU-2 在 F2 之前解决（场景几何调整为稳态可观测），则 F2 可直接在 batch 层补回观测/SNR 趋势软断言。
