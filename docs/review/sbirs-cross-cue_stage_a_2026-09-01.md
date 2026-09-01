---
Status: frozen
Date: 2026-09-01
Review-Baseline: `evidence/sbirs-cross-cue` @ `7a2e4f58`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# sbirs-cross-cue：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

- 触发来源：用户 2026-09-01 指令"基于递话式（cross-cue，交叉提示：一颗星发现目标后把'往这儿看'的消息递给另一颗星，另一颗星直接把窄场转过去盯）立项"。
- 前置事实：8.7° 窄视场试验三轮全部失败并回退（伴星宽场从不检出 → 双星配对 0/80 → SMOKE FAILED），根因是两星无法在配对窗（配对窗：双星定位要求两星窄场检出落在相邻周期内才能配成一对的时限规则）内同时拿到窄场数据。
- **证据**：[evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.md]
- 已定输入（用户已裁定，本轮不再讨论）：
  1、机制选递话式 cross-cue；排班式（两星扫描相位对时刻表）不做。
  2、放宽配对窗不做——中段目标 5 km/s，窗放几十秒交会误差即百公里级，物理上不可接受。
- 四问总览（逐项详见 §1 矩阵）：
  1、冻结什么：跨星递话引导是否为窄视场路线真实且对症的前置需求，及其最小改动边界。
  2、什么证据证明：失败试验记录 + 库内无跨星路径的探针 + 既有消息/预测原语。
  3、什么证据否定：断链另有根因（能量/SNR），或实现必须触碰冻结面（回放协议、公开头文件语义、验收行格式）。
  4、通过后最小范围：会话输入加可选外部引导条目（默认空）+ 组件层消息接线 + 场景开关，不改任何既有行为。

## §1 证据矩阵

<!-- 1、探针/测试列写已执行的动作与结果。
2、建议判定只能取 pass / reject / narrow / defer，最终以用户裁定为准。
-->

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、时间断链真实存在且递话对症 | 窄视场路线唯一前置缺口是"两星无法在配对窗内同时产出窄场数据"；补星间递话即闭合 | 1、三轮试验全否定：配对 0/80、SMOKE FAILED、根因=占空稀疏×无协同×1 周期窗 [evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.md]<br>2、配对窗语义=两星检出错开超过窗即弃对 [evidence: src/precision_evaluation/PrecisionEvaluationSession.cpp]<br>3、排班式相位协同已裁定 reject（现实无"三颗星约好对表"，递话式=发现即任务化转头，与之不同物） [evidence: docs/review/sbirs-dual-sat-timing_stage_a_2026-08-31.md] | 1、grep 全库 cross/cue/inter_sat/external_cue：无任何跨星引导路径（命中仅为向量叉乘同名 Cross）<br>2、读配对实现确认弃对条件即"错开>窗" | 断链根因唯一指向引导不同时，递话直击根因 | 断链另有根因（能量/SNR），或放宽窗代价可接受 | pass |
| 2、消息链路建在组件层 | cue 消息可走 examples 组件/事件层（星→地→星），库内不新增消息协议 | 1、卫星组件已有消息投递模式并构造 SbirsBearingSample [evidence: examples/components/sbirs_sensor_component.cpp]<br>2、地面站逐条消费 event.detections 并重建观测阶段 [evidence: examples/components/ground_station_fusion_component.cpp]<br>3、事件面已有检测事件可挂载 [evidence: examples/core/events.h] | 已读组件投递分支（检测/落地均走事件往返）与地面站消费循环 | 组件层周期级消息往返在用，cue 复用同通道不碰库协议 | cue 必须进回放/flatbuffer 冻结协议，范围失控 | narrow |
| 3、库内输入面扩展（可选、默认空） | 会话输入加可选外部引导条目（目标键+来源+ECI 视线角与带误差距离），pipeline 与自星宽场引导并行消费 | 1、输入组装器现仅自星状态+场景目标、无外部面 [evidence: src/sbirs_sensor/session/SbirsExternalInputAdapter.cpp] ::MakeSbirsCycleInput<br>2、cue 预测器只用连续带噪测角、从不吃真值（递话后预测可直接复用） [evidence: src/sbirs_sensor/pipeline/SbirsCuePredictor.h] ::Update<br>3、候选自带误差距离，供他星三角化目标 ECI 位置 [evidence: src/sbirs_sensor/pipeline/SbirsNfovScheduler.h] ::SbirsCandidate<br>4、ECI 角度原语齐备 [evidence: src/sbirs_sensor/foundation/SbirsGeometry.h] ::ComputeAzimuthDeg | 已读四个文件确认能力面；grep 确认输入结构无外部引导字段 | 新字段可选且默认空，既有字段语义与顺序不动 | 必须改既有输入字段语义或强制消费顺序 | pass |
| 4、配对窗维持 1 周期的时间账 | cue 迟到 ≤1 周期 + 窄场转头可在数周期内完成捕获，窗不放宽 | 1、窄场转速上限 30°/s [evidence: include/1q/sbirs_sensor/config/SbirsMissionConfig.h] ::narrow_pointing_max_slew_rate_deg_per_sec<br>2、可见窗全宽 41.38°（最大转角上界） [evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp] ::SbirsPipeline | 推理账：视线角速率 ≈5 km/s÷36,000 km≈0.008°/s，远小于窄半角 4°；最大转角 ≤41° → 转头 ≤1.4 s | Stage B 场景实测双星窄场配对 >0 且 74 行精度不降 | 实测需放宽窗才能配对 → 回到项 1 否定，立项终止 | pass（条件式，Stage B 裁定） |
| 5、验收/归属语义零破坏 | 递话引导的窄场检出在验收行与评估口径里与自星引导同格式，仅 debug/归属记录加"引导来源"标记 | 1、生命周期记录器为独立扩展面 [evidence: src/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.cpp]<br>2、调试视图为独立扩展面 [evidence: src/sbirs_sensor/session/SbirsOutputDebugViewBuilder.cpp] | 已读两文件确认是独立记录面，不进验收行拼装路径 | 验收行格式不变，既有测试全绿 | 规范要求来源必须进验收行 → 涉及规范口径，单列冻结项 | narrow |

## §2 判定汇总与待裁定问题

判定汇总（建议，最终以用户裁定为准）：

1、项 1 pass：断链记录完整（0/80 + 冒烟失败 + 三因根因），库内确认无跨星路径，递话直击"引导不同时"根因。
2、项 2 narrow：消息链路限定在 examples 组件/事件层；若实现中发现必须扩回放/flatbuffer 协议，停下回 Stage A 单列冻结项。
3、项 3 pass：库内只加"可选外部引导条目"输入面，默认空 = 现行为逐位不变。
4、项 4 pass（条件式）：推理账成立（转头 ≤1.4 s、视线余量数百秒量级）；Stage B 场景实测配对 >0 且精度不降才算数，失败即立项终止。
5、项 5 narrow：验收行格式不动，"引导来源"只进 debug/归属记录。

需要用户拍板的问题：

1、cue 通道拓扑：星→地→星（经地面站转发；复用现有消息往返，真实系统地面段亦参与）还是组件内星间直连事件？建议前者。
2、递话内容口径：传"ECI 视线角 + 带误差距离"（他星据此三角化目标位置再转自己视角；不携带位置类合成量，贴近传感器实况）还是直接传目标 ECI 位置估计？建议前者。
3、验证载体：先在现行 24° 场景加"默认关"的 cross-cue 开关验证链路（现行为零变化），窄视场（8.7°）场景重启另行触发？建议前者。
4、"引导来源"是否需要出现在验收日志行内（涉及规范 OCR 口径变更）？建议不进验收行，仅 debug/归属记录。

修订 4（2026-09-01，用户指令，关闭全部待裁定点）：
1、通道仲裁：自星引导与他星递话目标同池同规则，统一按现行 SNR↓→距离↑→ID↑ 排序；"引导来源"只记录，不参与排序。

修订 5（2026-09-01，用户指令，Stage B 启动时修正输入面设计）：
1、cross-cue 消息属**运行时输入**，不放置在每帧 `*Input` 结构体里：撤销 §3"Allowed scope"首条"`SbirsCycleInput` 加 `external_cues` 字段"的设计。
2、改为会话层运行时注入：新增公开 POD `SbirsExternalCue` 与 `SbirsSession::SubmitExternalCue` 注入接口；组件在收到地面站转发后、下一周期步进前调用；管线挂运行期队列，周期开始时消费（消费后清空，未消费不跨周期残留语义由注入节拍保证）。
3、附带收益：回放/快照路径零接触（快照不含该队列，回放从不注入即逐位一致）；`SbirsInputValidation` 帧校验面不动；非法 cue 在注入点丢弃并计 issue（kInfo）。

修订 6（2026-09-01，实现期新证据，Stage B 记录）：
1、缺口：裁定 2 要求递话携带"带误差距离"，但归属层 `estimated_range_m` 在宽场段填的是真值距离（`candidate.range_m`），带误差的 `measured_range_m` 只存在于库内候选，消息路径（组件→事件→他星）拿不到。
2、处置：归属记录（`SbirsOutputTypes.h` 同一结构）补一个诊断字段 `measured_range_m`（误差模型距离，仅归属/诊断层），宽场段填充 `candidate.measured_range_m`；组件层 stage-0 记录据此构造递话距离。属 §3"归属/调试"允许面的加法，非新模块。
3、既有语义不改：`estimated_range_m` 各处含义维持原样（宽场=真值、窄场=融合估计），不做语义清洗（超出本契约）。

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、验收门：构建、聚焦测试、契约测试、特征化测试。
5、非目标。
-->

Proven requirement：
- 窄视场（8.7°）路线的时间断链由"两星无法在配对窗内同时产出窄场数据"导致；星间递话（cross-cue）引导使受话星窄场直接转头，闭合该断链。

Allowed scope：
- 库内输入面（修订 5 改为运行时注入）：新增公开 POD `SbirsExternalCue`（目标键+来源星实体 ID+来源星 ECI 位置+ECI 视线角+带误差距离+周期号）与 `SbirsSession::SubmitExternalCue` 注入接口；**不改 `SbirsCycleInput`**；管线内运行期队列，周期开始时消费；`MakeSbirsCycleInput` 与 `SbirsInputValidation` 零改动；非法 cue 注入点丢弃并计 issue（kInfo）。
- 库内消费：`src/sbirs_sensor/pipeline/` 目标循环中，自星宽场门失败但本周期有外部引导的目标改为构造外部候选（按裁定 2 三角化：来源星位置+距离×视线方向 → 目标 ECI 位置 → 本星视线角）；`SbirsCandidate`（`SbirsNfovScheduler.h`）加 `cue_source_satellite_entity_id`（-1=自星宽场）；外部候选与自星候选进同一调度池，统一 SNR↓→距离↑→ID↑（修订 4）；连续命中计数按外部引导到达累计；NFOV 捕获/跟踪/丢锁门全部复用既有逻辑（真值落窄场+窄场 SNR 门，物理真值链路不放松）。
- 归属/调试：`SbirsOutputDebugViewBuilder` 与检测生命周期记录加"引导来源"标记；验收日志行格式零改动（修订 1 条 3）。
- 组件层：`examples/core/events.h` 新增 cross-cue 事件；`examples/components/sbirs_sensor_component` 每周期外发自星宽场量测（ECI 视线角+带误差距离+自身 ECI 位置）并在周期输入前注入收到的事件；`examples/components/ground_station_fusion_component` 转发（星→地→星，修订 1 条 1）。
- 场景：`examples/scenes/sbirs_triple_sat_fix_messages` 宽场改 8.7°、cross-cue 常开（修订 2），俯仰栅格键按需启用（库内已有解析）；场景 md 同步。
- 测试：外部候选三角化与同池排序、默认空输入逐位不变、来源标记落 debug 的聚焦测试；场景冒烟。

Explicitly out of scope：
- 公开头文件：除 `SbirsExternalCue` POD 与 `SbirsSession::SubmitExternalCue` 注入接口（修订 5）外，任何 `include/1q` 改动（含 fusion/precision_evaluation 接口、`SbirsCycleInput`）。
- 跨模块类型：不为 cue 新建跨模块公共类型或兼容层。
- schema/回放：`SbirsReplayFlatbufferCodec` 不扩 schema；外部引导为组合层运行期输入，回放快照保持 cue 空。
- 测试阈值/skip：不放宽任何既有门限。
- 配对窗：`dual_sat_pair_window_cycles` 维持 1。
- 发现星自家链路：WFOV→NFOV 自星引导行为零改动（修订 3 条 1）。

Behavior boundary：
- 输入：运行期队列默认空（无人注入）→ 全部行为与现状逐位一致；有注入时仅影响所引导目标的候选构造路径（修订 5）。
- 输出：检测记录/验收行字段与格式不变；debug/归属记录多一个来源标记。
- 错误回退：非法 cue（非有限/非正距离/未知目标键）整条丢弃并计 issue（kInfo），不影响其余目标。
- 生命周期：外部引导目标丢锁后回候选态，需再次收到引导才重入调度；standby/关机清空。

Acceptance gates：
- 构建：全量构建零警告回归。
- 聚焦测试：新增单测 + sbirs_sensor 既有测试全绿（基线 379+）。
- 契约测试：`external_cues` 空输入下与基线输出逐位一致（特征化测试钉住）。
- 场景门：8.7° 场景重跑，双星窄场配对 >0（对照失败基线 0/80）且冒烟通过；24° 基线场景行为不回归。

Non-goals：
- 窄场锁定后用精测刷新递话（修订 3 条 4，Stage B 可选项，不做）。
- 排班式相位协同、配对窗放宽（已 reject）。
- 真实 SBIRS 全协议仿真（消息内容仅限裁定 2 口径）。

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

修订 1（2026-09-01，用户指令）：
1、拍板问题 1：cue 拓扑=星→地→星（经地面站转发，复用现有组件层消息往返）。
2、拍板问题 2：递话内容=ECI 视线角 + 带误差距离（不携带位置类合成量，他星自行三角化）。
3、拍板问题 4："引导来源"不进验收日志行，仅 debug/归属记录。

修订 2（2026-09-01，用户指令"3 使用新链路触发验证"）：
1、验证载体=重建窄视场（8.7°）场景，cross-cue 常开，链路打通（双星窄场配对恢复、冒烟通过）即验证通过。
2、不在现行 24° 场景做"默认关开关"的零变化验证。
3、理解口径已向用户复述；如有出入以用户更正为准，改本条不静默改 §2。

修订 3（2026-09-01，用户新问题引出，待裁定）：
1、语义确认：发现星自家 WFOV→NFOV 引导为既有行为，cross-cue 仅复制递话给伴星，不改变自家链路；两星最终各自窄场盯同一目标。
2、新增待裁定项：自星引导与他星递话目标竞争同一 NFOV 通道池（上限=max_concurrent_nfov_locks，每星独立，结构体默认 1、场景 2）时的优先级规则。
3、建议：同池同规则（统一 SNR↓→距离↑→ID↑），"引导来源"只作记录不参与排序。
4、Stage B 可选项（不进契约）：发现星窄场锁定后用精测刷新递话，提高伴星指向精度。

## §4 运行记录（Stage C 后填写）

<!-- 1、实现范围。
2、验证命令与结果。
3、权威回写去向：哪个结论写进了哪个文件。
4、残留风险。
5、后续冻结项。
-->

修订 7（2026-09-01，实现期新证据，Stage B 记录）：
1、缺口：cross-cue 外发需要"本星宽场候选的量测（带误差测角+带误差距离）"，但宽场候选被调度选中后其量测无输出出口（输出帧只有窄场捕获/跟踪记录；调试视图同源也无宽场候选状态）。
2、处置（允许面扩展）：`SbirsCycleResult` 新增诊断列表 `wide_cue_measurements`（target_id + ECI 带误差 az/el + measured_range_m，仅归属/调试面，不进验收行）；管线在自星宽场候选创建处填充；组件 `PublishCrossCue` 据此外发。基线（无消费者）行为零变化。
3、外发口径不变：只递宽场量测（发现即递话），窄场精测刷新仍不做（修订 3 条 4）。
