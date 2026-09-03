---
Status: final
Date: 2026-09-03
Review-Baseline: `evidence/rf-full-polarization` @ `fba3ea13`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# rf-full-polarization：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

触发来源：用户代码评审意见——`RfLinkBudget.h` 的 `RfPolarization` 枚举"加一个全极化的枚举值"。
- **证据**：[evidence: include/1q/electromagnetics/RfLinkBudget.h]::RfPolarization（kFullPolarization = 5 已按用户指示先行加入，未提交）

背景白话：
1、极化（polarization）＝电磁波里电场振动的方向；天线接收像对插头插座，方向对得上才收得全。
2、现有五个值＝四种固定振动方式（水平/垂直线极化、左/右旋圆极化）＋一个"方向随机乱跳"的非极化。
3、全极化（full polarization）＝同一套收发系统同时保持两条相互垂直的极化通道（如水平＋竖直各一条）；来波只要有固定振动方式，两条通道合起来就能把能量几乎全收回来。
4、全极化 vs 非极化（最易混）：非极化说的是"波没规律"（属性在对方），全极化说的是"我两条通道都收"（属性在自己）。

现状一句话：枚举值已声明但库内校验不认，处于"一用就被拒绝"的半落地状态，本轮裁定补齐还是回退。
- **证据**：[evidence: src/common/electromagnetics/RfLinkBudget.cpp]::IsKnownPolarization（switch 未列新值）

待裁定项总览（每项四问展开见 §1 矩阵）：
1、半落地枚举值的收敛风险是否成立（冻结"必须补齐或回退，不许停在半落地"）。
2、全极化落点：公共 API 枚举（RfPolarization）与活跃仿真链枚举（RfScenePolarization）是两条并行链，加在哪条或两条都加。
3、全极化对四种固定极化的失配损耗语义（合并 0 dB 还是保守 3.01 dB）。
4、全极化对非极化的失配损耗语义（保守 3.01 dB 还是功率守恒 0 dB）。
5、场景层配套打通范围（字符串拼写、加载器、ESR 校验、回放往返）。
6、默认值与存量行为零变更。
7、非目标边界：RIR 极化散射矩阵（另一套"全极化测量"特性）与极化捷变（时变极化）不触碰。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、半落地枚举值构成"声明即拒绝"风险，必须收敛 | kFullPolarization=5 已进头文件，但库校验拒绝它；停在半落地是最差状态 | [evidence: include/1q/electromagnetics/RfLinkBudget.h]::RfPolarization；[evidence: src/common/electromagnetics/RfLinkBudget.cpp]::IsKnownPolarization | 源码检视：IsKnownPolarization 的 switch 未列新值，TryValidateRfEmissionFrame 与 TryEvaluateRfLink 入口先查它，新值必被拒；`ctest -R "unit::common" -C Release`：1/1 Passed（存量测试不受影响） | 矩阵确认风险，本轮补齐语义或回退枚举值，二选一 | 库内存在能正常消费值 5 的其他入口（检视未发现） | pass（风险成立） |
| 2、需求落点：两条并行极化链，全极化必须落在活跃链才真正可用 | RfPolarization 是给集成方的纯函数 API，src/examples 零调用方；RfScenePolarization 才是活跃链（四模块配置、RF v2 计算、回放、场景 JSON 全走它） | [evidence: include/1q/electromagnetics/RfScene.h]::RfScenePolarization；[evidence: src/common/electromagnetics/RfScene.cpp]；[evidence: include/1q/airborne_radar/config/ArHardwareConfig.h]；[evidence: include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h]；[evidence: include/1q/electronic_countermeasure/EcmTypes.h] | 全仓 grep（已执行）：RfPolarization 在 src/examples 零调用方，仅 RfLinkBudget.cpp 自身与单测引用；RfScenePolarization 被 AR/RIR/ESR/ECM 四模块配置、四个回放编解码器、场景脚本、示例加载器消费 | 用户确认落点范围（仅公共 API / 仅活跃链 / 两侧对称） | 用户表明只要休眠 API 上的声明 | narrow（建议两侧对称加值、语义一次冻结；活跃链是最低要求） |
| 3、全极化对固定极化的失配损耗：合并口径 0 dB | 全极化接收侧对 H/V/R/L 任一固定极化发射 → 0 dB（两条正交通道功率合并，任意固定极化投影 cos²θ+sin²θ=1）；单极化侧收全极化发射 → 3.01 dB（只能收到正交两分量之一）；全极化对全极化 → 0 dB；全极化配对不进 cross_polarization_isolation 分支（定义上两条都收，不存在"正交收不到"） | [evidence: src/common/electromagnetics/RfLinkBudget.cpp]::TryPolarizationLoss、::kLinearCircularLossDb；[evidence: src/common/electromagnetics/RfScene.cpp]::TryPolarizationLoss | 源码检视：现有损耗只有三档——同值 0 dB、同族互垂取 cross_polarization_isolation_db（默认 30）、线圆互配或任一侧非极化 3.0103 dB；[evidence: tests/unit/common/common_rf_link_budget_test.cpp] 已钉 3.0103±1e-4 | 用户裁定取 0 dB 合并口径 | 用户选保守口径（对固定极化也 3.01 dB） | pass（推荐 0 dB——双通道合并是全极化存在的物理意义） |
| 4、全极化对非极化的失配损耗：保守 3.01 dB | 保持现有公理"任一侧非极化固定 3.01 dB"不动，全极化对非极化同取 3.01 dB | [evidence: include/1q/electromagnetics/RfScene.h]（注释块明文该配对规则）；[evidence: src/common/electromagnetics/RfLinkBudget.cpp]::TryPolarizationLoss | 推理：物理上双通道功率相加可回收全部非极化功率（0 dB），但对干扰/杂波场景这意味着全极化接收无代价收满干扰——保守建模取单通道一半（3.01 dB）更安全，且改动最小（现有公理零变更） | 用户裁定取 3.01 dB（保守、公理不变） | 用户裁定取 0 dB（功率守恒口径，干扰计算更严） | pass（推荐 3.01 dB 保守） |
| 5、场景层配套打通范围 | RfScenePolarization 若加值，需配套：字符串拼写、示例加载器、ESR 配置校验上限、回放往返；回放字段全是 int 裸转型，值 5 天然往返，无需 schema 变更 | [evidence: examples/common/config_loaders/remote_identification_radar/config_loader_common.h]::RfScenePolarizationFromString；[evidence: src/electronic_surveillance_radar/session/EsrSessionConfigValidation.cpp]（范围钉 0..4）；[evidence: schemas/replay/rir_session_replay.fbs]、[evidence: schemas/replay/airborne_radar_session_replay.fbs]、[evidence: schemas/replay/esr_session_replay.fbs]、[evidence: schemas/replay/ecm_replay.fbs]（极化字段全为 int）；[evidence: src/remote_identification_radar/session/RirReplayFlatbufferCodec.cpp]（双向 static_cast 无范围检查） | 源码检视（已执行）：1、加载器未知字符串静默回退 kHorizontal（既有陷阱模式，拼错不报错）；2、ESR 校验上限钉死 kUnpolarized，加值后必须同步；3、回放裸转型，值 5 可往返但无测试钉住 | 配套范围进冻结契约（拼写建议 "kFullPolarization" 与现有五个拼写同构；回放往返测试转正） | 用户裁定场景层不动（仅公共 API 加值） | pass |
| 6、默认值与存量行为零变更 | 所有默认值不动：AR/RIR scene_polarization 默认 kHorizontal、ESR polarization 与 ECM transmit_polarization 默认 kUnpolarized、RfEmission/RfReceiverSite 默认 kUnpolarized、场景发射侧硬编码 kHorizontal 不动 | [evidence: include/1q/remote_identification_radar/config/RirHardwareConfig.h]；[evidence: include/1q/airborne_radar/config/ArHardwareConfig.h]；[evidence: include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h]；[evidence: include/1q/electronic_countermeasure/EcmTypes.h]；[evidence: examples/scenes/scene_script.cpp]（发射侧硬编码） | 源码检视默认值清单（已执行，与假设一致） | 冻结契约写明"默认值不变更、存量配置行为逐位不变" | 用户要求改默认 | pass |
| 7、非目标：RIR 极化散射矩阵与极化捷变不触碰 | RIR 已有的极化散射矩阵特性（双通道 RCS、S 矩阵、acceptance-S：span/abs_det/depolarization/psi/tau）是独立的"全极化测量"识别特性，与链路预算极化模式无数据流交联；枚举是"名义极化"常量，不承诺极化捷变（随时间切换极化） | [evidence: tests/unit/remote_identification_radar/rir_polarization_acceptance_s_test.cpp]；[evidence: schemas/replay/rir_replay.fbs]::RirPolarizationRcsSample；[evidence: include/1q/electromagnetics/RfLinkBudget.h]（注释"名义极化"） | 源码检视：acceptance-S 由双通道 RCS 样本驱动，输入来自场景极化 RCS 表，不经过 RfScenePolarization 配对逻辑 | 冻结契约列为非目标 | 用户要求联动（如全极化模式改变 S 矩阵行为） | reject（列入非目标） |

## §2 判定汇总与待裁定问题

判定汇总：
1、半落地风险：pass——必须补齐或回退，不许停在"声明即拒绝"。
2、需求落点：narrow——建议 RfPolarization 与 RfScenePolarization 两侧对称加值、语义一次冻结；活跃链（RfScenePolarization）是最低要求。
3、全极化对固定极化：pass（建议 0 dB 合并口径）——全极化接收对任意固定极化 0 dB；单极化收全极化发射 3.01 dB；全对全 0 dB；不进隔离分支。
4、全极化对非极化：pass（建议 3.01 dB 保守）——现有"非极化=一半"公理不动；若选 0 dB 则干扰/杂波功率计算更严，见待裁定问题 3。
5、场景层配套：pass——字符串拼写、加载器、ESR 校验上限、回放往返测试一次配齐；回放 schema 无需变更。
6、默认值零变更：pass。
7、非目标：reject——RIR 极化散射矩阵、极化捷变明确不碰。

需要用户拍板的问题：
1、落点范围：仅公共 API（RfPolarization）/ 仅活跃链（RfScenePolarization）/ 两侧对称（建议）？
2、全极化对固定极化损耗：0 dB 合并（建议）还是 3.01 dB 保守？
3、全极化对非极化损耗：3.01 dB 保守（建议，现有公理不动）还是 0 dB 功率守恒（干扰/杂波口径更严）？
4、场景层字符串拼写取 "kFullPolarization"（与现有五个拼写同构）是否同意？
5、四个模块配置（AR/RIR 收、ESR 收、ECM 发）是否都允许设全极化，还是先限部分模块？

## §3 冻结契约（用户讨论结束后填写）

已证明的需求：
- 全极化作为第六种极化工作模式进入两条枚举链；失配损耗配对表扩展全极化规则；模块放行范围仅 RIR。

允许范围：
- 公共头：include/1q/electromagnetics/RfLinkBudget.h——RfPolarization 已加 kFullPolarization=5（工作区既有改动转入本分支），枚举注释改"名义极化或极化工作模式"，kFullPolarization 配白话注释。
- 公共头：include/1q/electromagnetics/RfScene.h——RfScenePolarization 加 kFullPolarization=5＋白话注释，枚举块配对规则注释同步全极化。
- 实现：src/common/electromagnetics/RfLinkBudget.cpp——IsKnownPolarization、TryPolarizationLoss。
- 实现：src/common/electromagnetics/RfScene.cpp——IsKnownPolarization、TryPolarizationLoss 同步。
- 示例加载器：examples/common/config_loaders/remote_identification_radar/config_loader_common.h——RfScenePolarizationFromString 加 "kFullPolarization"。
- 测试：tests/unit/common/common_rf_link_budget_test.cpp（全极化配对单测，探针转正）；tests/unit/common/common_rf_scene_test.cpp（RfScene 侧配对单测）；tests/replay/remote_identification_radar/rir_replay_session_test.cpp（scene_polarization=5 往返）。
- 文档：Stage C 回写目标见 §4 清单。

明确禁止范围：
- AR/ESR/ECM 冻结：配置不放行（ESR 现有 0..4 上限校验保持不动即天然拒绝 5；AR/ECM 不加任何放行或校验改动）、加载器不加拼写、回放编解码不动。
- 回放 schema：四个 .fbs 一律不动（极化字段全为 int，值 5 天然往返）。
- 不新增 dual_channel/能力类字段，不新增兼容层。
- 加载器未知字符串静默回退行为不改（既有行为，未立案）。
- 所有默认值不变更：AR/RIR scene_polarization 默认 kHorizontal、ESR polarization 与 ECM transmit_polarization 默认 kUnpolarized、RfEmission/RfReceiverSite 默认 kUnpolarized、examples/scenes/scene_script.cpp 发射侧 kHorizontal 硬编码不动。
- RIR 极化散射矩阵特性（双通道 RCS、acceptance-S）不动；极化捷变（时变极化）不做。
- 测试阈值不放宽、不新增 skip。

行为边界（失配损耗配对表，行=发射侧，列=接收侧）：
- 固定极化 × 固定极化：现有三档规则不变（同值 0 / 同族互垂取 cross_polarization_isolation_db / 线圆互配 3.0103）。
- 固定极化 × 全极化（全极化接收）：0 dB。
- 全极化 × 固定极化（单极化接收）：3.0103 dB。
- 全极化 × 全极化：0 dB。
- 全极化 × 非极化、非极化 × 全极化：3.0103 dB（"任一侧非极化固定 3.0103 dB"公理不动）。
- 全极化配对一律不进 cross_polarization_isolation_db 分支。
- 输入：枚举值 5 为合法值；其余非法值拒绝行为不变（IsKnownPolarization 之外的路径零改动）。
- RIR 实路径：scene_polarization 一个旋钮填收发两侧（同值），"kFullPolarization" 下 RIR 自配对 full/full＝0 dB。

爆炸半径与回滚：
- 下游影响：RIR 配置 JSON 可填 "kFullPolarization"；其余模块行为逐位不变；回放文件向后兼容（旧文件不含值 5）。
- 回退难度：无损——纯加法改动，回滚＝删枚举值＋分支＋测试。

验收门：
- 构建：全量 cmake 构建（含 common、remote_identification_radar、examples）。
- 聚焦测试：ctest unit::common、unit::remote_identification_radar 全绿。
- 回放契约：RIR replay 测试全绿（含新增 scene_polarization=5 往返断言）。
- 探针转正：两条枚举链的全极化配对单测＋RIR 回放值 5 往返断言。
- 存量回归：3.0103 dB 与 cross_polarization_isolation 既有断言不变。

非目标：
- RIR 极化散射矩阵联动、极化捷变、AR/ESR/ECM 放行全极化、加载器回退行为整改、dual_channel 字段化建模。

## 修订记录

修订 1（2026-09-03，用户指令）：落点裁定＝两侧对称——RfPolarization（公共 API）与 RfScenePolarization（活跃链）同时加 kFullPolarization=5，语义一次冻结；矩阵第 2 项按此落地。
修订 2（2026-09-03，用户指令）：全极化对固定极化＝0 dB 合并口径——全极化接收对 H/V/左旋/右旋任一固定极化 0 dB；单极化接收全极化发射 3.0103 dB；全极化对全极化 0 dB；全极化配对不进 cross_polarization_isolation_db 分支。
修订 3（2026-09-03，用户指令）：全极化对非极化＝3.0103 dB 保守——"任一侧非极化固定 3.0103 dB"现有公理不动。
修订 4（2026-09-03，用户指令）：场景字符串拼写取 "kFullPolarization"，与现有五个拼写同构。
修订 5（2026-09-03，用户指令）：允许设置全极化的模块仅 RIR；AR/ESR/ECM 全部冻结——配置不放行、加载器不加拼写、回放不接。
修订 6（2026-09-03，用户讨论确认）：枚举注释改为"名义极化或极化工作模式"，kFullPolarization 配白话注释（两条相互垂直的极化通道同时工作）。
修订 7（2026-09-03，新证据）：RIR 会话校验现状不检查 scene_polarization 取值范围，加值后天然放行、校验层零改动；RIR 的 scene_polarization 一个旋钮同时填发射侧与接收侧（收发同值），全极化下 RIR 自配对为 full/full=0 dB。
- **证据**：[evidence: src/remote_identification_radar/session/RirSessionConfigValidation.cpp]（仅查交叉隔离有限性）
- **证据**：[evidence: src/remote_identification_radar/dwell/RirEmissionFactory.cpp]::RirEmissionFactory（emission->polarization = receiver.scene_polarization）

## §4 运行记录（Stage C 后填写）

1、实现范围：
1、公共头两侧：RfPolarization/RfScenePolarization 加 kFullPolarization=5，注释"名义极化或极化工作模式"。
2、实现两侧：RfLinkBudget.cpp 与 RfScene.cpp 的 IsKnownPolarization、TryPolarizationLoss 全极化分支（逐行同构）。
3、RIR 配套：加载器拼写 "kFullPolarization"；回放配置值 5 字节精确往返断言。
4、测试：两条枚举链五组配对单测（探针转正）＋帧校验放行断言。

2、验证命令与结果（VisualStudio.15.0-amd64 Release，2026-09-03）：
1、`cmake --build build/VisualStudio.15.0-amd64 --config Release --parallel 8`: pass（零错误，仅存量告警）。
2、`ctest -C Release --parallel 4`: pass——全量 63/63（含 docs_structure_guard）。
3、聚焦：unit::common、unit::remote_identification_radar、contract/integration/replay::remote_identification_radar 全绿。
4、审查：completeness-review Lane 1（major 门禁）：0 高 / 0 中 / 3 低（断言区分力与注释详略建议，不阻塞）。

3、权威回写去向：
1、正向边界：docs/common/rf_architecture.md（单程链路边界·极化配对规则四条＋证据锁；Last-reviewed 2026-09-03）。
2、否决记录：docs/remote_identification_radar/design.md"架构裁定与否决记录"专节（AR/ESR/ECM 放行、dual_channel 字段化、非极化 0 dB 守恒口径三项否决）。
3、开放议题：docs/common/open_questions.md（COMMON-OQ-11 全极化放行冻结、COMMON-OQ-12 非极化守恒口径）。
4、证据锁：[evidence: tests/unit/common/common_rf_link_budget_test.cpp]::FullPolarizationPairingAppliesMergeAndHalfRules、[evidence: tests/unit/common/common_rf_scene_test.cpp]::FullPolarizationPairingAppliesMergeAndHalfRules、[evidence: tests/replay/remote_identification_radar/rir_replay_session_test.cpp]::FullPolarizationSessionConfigRoundtripsByteExact。

4、残留风险：
1、全极化对非极化保守 3.0103 dB 口径对全极化接收方漏计一半非极化干扰功率（COMMON-OQ-12 跟踪）。
2、AR/ECM 无 scene_polarization 校验拦截：程序化直填值 5 时共享计算层会按全极化求解（ESR 有 0..4 校验拒绝；配置文件路径无拼写放行，实际不可达）。
3、加载器未知拼写静默回退 kHorizontal 既有陷阱未整改（未立案，非本轮范围）。

5、后续冻结项：
1、AR/ESR/ECM 全极化解冻（COMMON-OQ-11）。
2、非极化守恒/分路口径（COMMON-OQ-12）。
