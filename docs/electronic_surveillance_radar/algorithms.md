---
Status: active
Last-reviewed: 2026-08-03
Authority: ESR 算法登记与实现边界
Answers: ESR 用了哪些算法、各自实现到什么地步、边界在哪、哪些刻意不实现或预留死字段
---

# ESR 算法登记

本文是 ESR 算法清单与边界的权威。算法本身的逐步逻辑读代码（`src/electronic_surveillance_radar/`）；本文只回答
"用没用/到哪步/为什么不做"。模块级边界（scan_rate_hz、dt_sec、扫描窗口与坐标系、输出边界）见
[boundaries.md](boundaries.md)。

## 算法登记表

| 算法/部件 | 意图（一句话） | 实现状态 | 证据 |
|---|---|---|---|
| 环境采样 | 传播附加损耗与杂波噪声快照 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_environment_service_test] |
| 扫描窗口 | 根据扫描模式和运行期配置生成接收窗口 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_controller_runtime_state_test] |
| 拦截门控 | range、receiver window、dynamic range、SNR 等 joint constraints | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_intercept_gate_test] |
| 边界搜索 | 单调谓词边界查找 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_boundary_search_test] |
| 角误差 | 基于 SNR/系数/随机种子的 AOA 扰动 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_angle_error_test] |
| RF 接收与干扰影响 | 双 receiver state、宽带入射账本、饱和、到达时频角分辨单元 SINR | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_rf_v2_front_end_test] |
| 分辨单元投影 | 到达活动投影到固定接收时间单元并按角单元/重叠频带归并 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_resolution_cell_ledger_test] |
| 观测预处理 | 排序、有限值过滤、质量归一、窗口去重 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_kdtree_clusterer_test] |
| 特征编码 | RF/PW/AOA/SNR 按尺度编码到特征空间 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_kdtree_clusterer_test] |
| 聚类 | 半径聚类、min-points、noise/border point 处理 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_kdtree_clusterer_test] |
| 假设关联 | cluster 到 track 的 gated matching、ID 稳定、miss 回收 | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_hypothesis_associator_test] |
| 欺骗标注（Strategy A） | 脉冲观测角度聚类标注 likely-false-target | session-wired | [evidence: tests/unit/electronic_surveillance_radar/esr_deception_detection_test] |

实现状态取值：
- **session-wired**：已接入 `EsrController` / `InterceptPipeline`，覆盖 config、输出/abort、replay 与 session 集成。

ESR 当前所有登记算法均为 session-wired；不存在 characterized/experimental 候选（无 Omega-K 那样的晋级门）。

## 拦截检测链（核心）

冻结的 ESR 拦截链是"宽带前端 → 调谐频率-角度单元 → 观测提取 → 分选/假设"。本节是 ESR 的核心边界，
几乎全部由"不得/不能/不"禁令构成。

1. **单程入射事实。** 对冻结 scene 中每个实际 emission 计算到 ESR equipment 的单程 incident link。
   exact emission ID 只用于避免同一候选重复计入，platform/equipment ID 用于 co-site 路径；**不得**因同平台
   有多个发射设备而排除整个平台，**也不得**把 truth role 带入 detection。
2. **宽带前端账本。** 用固定 receive beam、预选器和设备损耗聚合所有进入前端的功率。该账本独立于当前
   调谐通道，用于最大线性输入、同平台泄漏和强带外 blocking 边界。超过标定上限时输出结构化
   `receiver_saturated` impairment，本周期仍是 executed，但 observation/hypothesis 不生成新记录；**不使用**
   未标定压缩曲线。
3. **通道化与可分辨性。** `EsrRfV2FrontEnd` 同时冻结两个接收状态：硬件频段的 `front_end_receiver` 只服务
   blocking/饱和，当前 tuning window 的 `channel_receiver` 只服务候选检测。`EsrResolutionCellLedger` 将调谐
   通道内的到达活动投影到固定接收时间单元；脉冲使用确定性 jitter 后的实际绝对脉冲时刻累计，线性扫频
   在每个到达时间单元重新求瞬时频率，再按角单元和重叠频带排序归并。单元内功率最强的外部源成为候选，
   其余功率只进入该候选的干扰账本；落入不同接收时间单元的错时脉冲、错频扫频和可分角源**不**互相降低
   SINR。构建复杂度是固定时间单元投影加单元内排序，**不再**执行 candidate×all-emissions 全对扫描。
   候选 signal/interference power 统一按候选实际活动时间归一化，空白接收窗口**不得**稀释短脉冲或扫频驻留
   的 SNR；跨越多个时间单元的同一物理脉冲以 pulse index 去重，只贡献一次统计截获机会。天顶/天底入射的
   方位角在数学上不可观测，但这**不是**非法 RF 帧：该源仍参与前端饱和和对应极区分辨单元的功率账本，
   却**不能**成为会发布伪造 AoA 的候选；同帧其他可观测源继续正常处理。
4. **波形化观测。** 脉冲列填写 PRI/脉宽估计；连续、宽带噪声和扫频仅发布适用的频率/带宽估计，**不**伪造
   PRI/pulse width。
5. **截获判决。** 每个候选使用通道输出 signal power、热噪声、未分辨 interference、有效驻留和脉冲截获机会
   计算 post-channel SINR/intercept probability，再按固定随机子流采样 detection。测量噪声**只能**在 detection
   成功后施加，**不能**反向改变接收波束、gate 或候选归并。
6. **接收机影响。** 当前结构化 impairment 为 `receiver_saturated`；它**不**表达发射方意图。压制效应只通过
   同一分辨单元中的 SINR 与检测结果影响输出；**没有** `is_jammed` 或二次布尔置信度惩罚。
7. **分选与 hypothesis。** preprocess、cluster、deinterleave 和 associator **只能**消费实际生成的 observation。
   center frequency、bandwidth、PRI、pulse width、bearing 及不确定度来自观测统计；**不得**从 scene emitter
   原样复制真值。platform/equipment/emission identity **不**进入 observation 或 hypothesis。
8. **欺骗标注（Strategy A）。** 检测完成后，对已发布的所有观测执行角度聚类：脉冲观测
   （`EsrWaveformClass::kPulse`）中，与其他脉冲观测在同一天线波束宽度内出现 ≥2 个时，该观测被标记为
   `kLikelyFalseTarget`。标注是纯观测层分类，**不**改变检测门限，**不**触发信号级反制；供下游消费者区分
   真实与可疑观测。

### 随机流与顺序无关

检测门与 AoA 误差各使用由 session seed、world cycle、发射 identity 和固定 domain tag 派生的独立流；
稳定 identity 排序保证输入发射顺序**不**改变其它发射的随机结果。相同输入、snapshot 和配置必须
continuation/replay 一致。

### replay 几何精度边界

Replay 的 cycle-input ECEF 位置、速度和独立姿态均为 double 精度；schema/codec **不允许**把它们降为 float。
输出比较继续使用严格判等，输入必须先做到可精确重组，**不能**用比较容差掩盖几何量化引起的观测角漂移。

[evidence: tests/unit/electronic_surveillance_radar/esr_rf_v2_front_end_test]
[evidence: tests/unit/electronic_surveillance_radar/esr_resolution_cell_ledger_test]
[evidence: tests/integration/electronic_surveillance_radar/esr_session_test]

### 模块范围（刻意不实现的接收能力）

复数 IQ、亚时间单元的器件瞬态、未标定压缩和欺骗/转发**不在**本模块范围。RF v2 characterization、前端、
检测、饱和、调谐、顺序无关和 replay 测试共同证明当前接收合同。

## 观测预处理

- **意图**：对 raw observation 排序、丢弃非有限/非法值、在时间/RF/pulse width/azimuth/elevation 窗口内去重，
  并按 SNR 做质量归一（high/medium/low）。
- **实现边界**：
  1. 去重时保留 SNR 更高或 observation id 更小的记录——确定性裁决，不接受随机顺序输入。
  2. 质量归一阈值是固定档位（`snr_db >= 18` → high，`>= 10` → medium，否则 low）；归一结果进入特征编码，
     不回写 raw observation。
- **证据**：[evidence: tests/unit/electronic_surveillance_radar/esr_kdtree_clusterer_test]

## 特征编码与聚类

- **意图**：把 observation 转为特征向量（RF/PW/AOA/SNR 尺度来自 `InterceptClusterConfig`），用 `KdTreeClusterer`
  按半径和 min-points 判断 cluster/noise 并处理 border point。
- **实现边界**：
  1. 该层只把同一 emitter 的多条观测归为候选簇，**不**直接生成最终 emitter hypothesis（hypothesis 由
     `HypothesisAssociator` 跨周期关联产出）。
  2. noise/border point 处理是确定性的；聚类参数变化必须同步 focused 测试。
- **证据**：[evidence: tests/unit/electronic_surveillance_radar/esr_kdtree_clusterer_test]

## 假设关联

- **意图**：`HypothesisAssociator` 维护内部 track state，每周期把 cluster centroid 关联到现有 track，未命中
  track 累计 missed cycles 达阈值后回收。
- **实现边界**：
  1. waveform class 相同、距离有限且不大于 `gate_distance` 的 cluster-track pair 进入候选集；pulse、continuous、
     sweep 和 noise **不允许**跨周期互相改写 track 类型。
  2. 候选图执行一对一全局分配：**先最大化匹配数量，再在最大匹配中最小化总距离**；总代价相同时按
     cluster input index、track hypothesis id 确定性裁决。不接受任意或贪心分配。
  3. 匹配 track 使用 `confidence_alpha` blending 更新 feature、bearing、mode、threat、confidence；未匹配
     cluster 创建新 hypothesis id；未命中 track 达 `max_missed_cycles` 阈值的当周期回收。
  4. 模式/威胁推断只来自观测统计（pulse width、PRI、SNR 推断 search/tracking/guidance；guidance 或高 SNR
     提升 threat level）；waveform class 只参与同类跨周期关联门控，当前**不**生成 deception/ambiguous
     candidate。
- **反直觉点**：snapshot continuation 与 replay 必须保持 waveform class gate 确定性——跨周期关联门控属于
  累积运行态，恢复失败不得留下半恢复状态（见 data-flow.md 状态所有权）。
- **证据**：[evidence: tests/unit/electronic_surveillance_radar/esr_hypothesis_associator_test]
- **证据**：[evidence: tests/replay/electronic_surveillance_radar/esr_replay_session_test]

## 非目标（刻意不实现 / 预留死字段）

1. **基于占用率的环境噪声建模（`spectrum_occupancy_ratio` 死字段）**：该字段的设计意图是表示尚未显式
   建模的环境噪声/占用背景；一旦相同 RF 源已经作为 emission 输入，**不得**再通过 occupancy 标量重复计入。
   大气物理继续只提供单程附加传播损耗。**当前实现边界**：`spectrum_occupancy_ratio` 在
   `EsrEnvironmentService` 中被冻结并随 snapshot/replay 持久化，但检测链（`InterceptDetectionExecutor`）
   **尚未消费**该字段——既无 noise PSD 换算，也无环境噪声倍率项。故当前**不得**宣称 ESR 具备基于占用率的
   环境噪声建模或压制干扰感知能力；该字段为预留死字段，去留见 `docs/common/open_questions.md` ESR-OQ-1。
   [evidence: tests/unit/electronic_surveillance_radar/esr_environment_service_test]
2. **`is_jammed` 二次布尔置信度惩罚**：压制效应只通过同一分辨单元的 SINR 与检测结果影响输出，没有
   `is_jammed` 标志或二次布尔置信度惩罚（见拦截检测链 §6）。
3. **信号级反欺骗反制**：欺骗标注（Strategy A）是纯观测层分类，不改变检测门限，不触发信号级反制
   （见拦截检测链 §8）。
4. **复数 IQ / 亚时间单元器件瞬态 / 未标定压缩 / 欺骗转发**：不在本模块接收范围（见拦截检测链 模块范围）。
5. **truth identity 进入输出**：platform/equipment/emission identity 不进入 observation 或 hypothesis；
   ECM sensor-driven adapter 只能复制估计字段和稳定 hypothesis ID（见拦截检测链 §7）。
