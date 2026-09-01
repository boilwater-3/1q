---
Status: draft
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

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、验收门：构建、聚焦测试、契约测试、特征化测试。
5、非目标。
-->

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

## §4 运行记录（Stage C 后填写）

<!-- 1、实现范围。
2、验证命令与结果。
3、权威回写去向：哪个结论写进了哪个文件。
4、残留风险。
5、后续冻结项。
-->
