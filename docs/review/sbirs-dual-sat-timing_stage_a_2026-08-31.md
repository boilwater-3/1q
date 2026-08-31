---
Status: final
Date: 2026-08-31
Review-Baseline: `evidence/sbirs-dual-sat-timing` @ `49a4da81`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# sbirs-dual-sat-timing：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

1、触发来源：甲方需要一个带扫描模式的双星定位场景示例；当前示例场景为凝视态（扫描速率=0）。
- **证据**：[evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json]（三颗星 scan_rate_deg_per_sec=0.0）

2、用户裁定（2026-08-31 会话）：现实多星之间"不同步、只重叠"——各星扫描镜独立扫全盘，无相位协同；地面站拿打时间戳的测角数据做航迹关联，滤波器把相差零点几秒的观测对齐到同一航迹；双星定位价值在基线不在同时。现实是"连续扫+按时刻融合"，本仿真是"每秒抽一帧波束位置+严格同周期交会"。

3、术语白话：
1、双星交会（双视线交会）：两颗星各给一条"从卫星指向目标"的视线，两条线在空中最接近处取中点当目标位置。
2、WFOV 宽视场扫描：大视场来回扫，扫到谁算发现谁；NFOV 窄视场跟踪：发现后把小视场光轴转过去盯住它连续测。
3、同周期交会：评估会话只在"同一个仿真周期内两颗星都检出了同一目标"时才做一次交会。

4、待裁定项（各回答四问：冻结什么、什么证据能证明、什么证据能否定、通过后最小改动范围）：

1、扫描模式示例场景的需求是否成立：冻结"客户要扫描模式场景而现场景是凝视"这一需求。证明=用户指令+场景 JSON 速率为 0；否定=已有其他扫描场景覆盖该需求。通过后最小改动=示例场景层（JSON/装载），不动库。
2、"开扫描导致双星交会样本塌缩"的风险是否按原叙述成立：冻结风险叙述本身（波束跳 6° 错过目标 → 样本从 80 掉到十几）。证明=复现探针；否定=探针显示塌缩机制不同（另有门控）。通过后最小改动=按实证机制重新登记风险。
3、测量数据通道缺时间语义的架构缺口是否成立：冻结"量测无时间戳、配对按周期号、时间对齐无原料"这一判断。证明=输出/融合/评估三层数据结构均无时间字段+时间分辨率探针；否定=发现既有时间通道被忽略。通过后最小改动=为时间戳补位立项（范围另议）。
4、是否应引入星间扫描相位协同来恢复样本：冻结"把多星相位调成同步"这一方案取舍。证明=现实存在此类协同且仿真缺失它导致问题；否定=现实无此协同（用户裁定）且探针显示重叠由几何布局主导。不通过则零改动。
5、极区高轨（大椭圆轨道）补覆盖是否属于本议题：冻结"布局配合是否要随本议题处理"。证明=本议题链路涉及轨道布局；否定=它是布局维度不是时间维度且现场景全 GEO。defer 登记，不在本轮实现。

## §1 证据矩阵

<!-- 1、探针/测试列写已执行的动作与结果。
2、建议判定只能取 pass / reject / narrow / defer，最终以用户裁定为准。
-->

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 扫描模式示例场景需求成立 | 甲方要扫描模式双星场景；现示例场景为凝视 | 1、用户指令（2026-08-31 会话）<br>2、场景三颗星速率均为 0 [evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json]<br>3、扫描机构已按真实往复式实现并验收 [evidence: docs/review/sbirs-scan-realism_stage_a_2026-08-31.md] | 基线复算（仓库场景原样运行）：dual_sat_cycles=80/80、综合分 0.447267，与扫描真实化运行记录逐位一致 | 需求来源明确且现场景确为凝视 | 已有其他扫描示例场景覆盖此需求 | pass |
| "开扫描→样本塌缩"风险按原叙述成立 | 波束每秒跳 6°，目标只在部分周期进视场，A/B 碰上时刻不重合 → 样本 80 掉到十几 | 1、交会条件=同周期双检出 [evidence: src/precision_evaluation/PrecisionEvaluationSession.cpp] ::Step<br>2、探针（仓库 JSON 仅改速率 0→6，可复现）：速率 6/30/60°/s 三档分别 74/80、74/80、72/80——未塌到十几，NFOV 捕获建立后跟踪段每周期供数掩盖了扫描稀疏 | 1、探针-跟踪在环：速率 6°/s → 74/80（前 6 周期为 B 星捕获起始延迟）<br>2、探针-掐断 NFOV（窄场门限抬至不可达）：WFOV 验收行仍"检测标志=是"，但输出帧零检出，dual_sat=0/80，冒烟失败——kSearchAndStare 下 WFOV 扫描检测无独立外发通道 [evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp] ::SbirsPipeline<br>3、探针-B 星扇区平移 20°：A 星连续检出、B 星全程未见，dual_sat=0/80，冒烟失败 | 探针复现原叙述的塌缩量级与机制 | 探针显示机制不同：塌缩形态是"0"（捕获不可达/几何错开）与"起始延迟"，不是"十几"；推理："十几"或来自部分目标建链的中间配置 | narrow |
| 测量数据通道缺时间语义（架构缺口） | 量测全程无时间戳、配对按周期号；无时间戳则任何融合算法都做不了"跨零点几秒对齐" | 1、传感器输出无时间字段 [evidence: include/1q/sbirs_sensor/session/SbirsOutputTypes.h] ::SbirsDetectionRecord<br>2、融合输入无时间字段 [evidence: include/1q/fusion/DetectionRecord.h] ::DetectionRecord<br>3、融合引擎按周期号驱动 [evidence: include/1q/fusion/FusionEngine.h] ::Update<br>4、交会原语本身时间无关（纯几何，收两条视线）[evidence: include/1q/precision_evaluation/DualLosFix.h] ::TryComputeDualLosFixM——"同周期"约束在编排会话消费规则，不在原语 [evidence: docs/precision_evaluation/algorithms.md]<br>5、初始扫描相位硬置 0、配置无相位字段（多星同相起点是人为的）[evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp] ::ApplyConfig；推理：配置头无相位字段（对 SbirsMissionConfig.h 全文检索无 phase） | 探针-时间分辨率（跟踪在环，仅 dt 1s→0.25s、周期 80→320 保持 80 秒）：dual_sat 从 74/80 变为 320/320，综合分 0.425107——样本率由时间分辨率决定 | 数据结构证据齐（三层无时间字段）且探针显示时间分辨率主导样本率 | 发现既有时间通道被忽略（如周期内时间已在某层携带） | pass |
| 引入星间扫描相位协同恢复样本 | 把各星扫描相位调成同步可恢复样本且符合现实 | 1、用户裁定（2026-08-31）：现实没有"三颗星约好你从左我先扫"的相位协同，各星独立扫全盘，地面按时间戳关联，滤波器对齐零点几秒的观测差——价值在基线不在同时；极区覆盖是布局配合不是时间协同<br>2、探针-B 星扇区平移：重叠由扇区几何布局决定，相位同步不能补几何错开 | 无（本项为方案取舍，否定证据已列） | 现实存在此类协同且仿真缺失它导致问题 | 现实无此协同（用户裁定）+探针显示布局主导；加协同反而掩盖真实缺口（按时刻融合） | reject |
| 极区大椭圆轨道高轨补覆盖属于本议题 | 布局配合需随本议题处理 | 1、用户裁定（2026-08-31）：极区补覆盖是布局上的配合，不是时间上的；现场景三颗星全 GEO | 无（范围判断） | 本议题链路必须动轨道布局 | 时间议题与布局议题正交；非 GEO 集成需求出现前无载体 | defer |

## §2 判定汇总与待裁定问题

1、扫描场景需求（pass）：客户明示需求；扫描机构已实现并验收，只差场景层把速率开起来。
2、样本塌缩风险（narrow）：风险成立但机制要改写——不是"波束跳快错过目标"，实证是三层：
1、跟踪在环时样本不塌（72-74/80），损失只是捕获起始的几何延迟；
2、捕获不可达（弱目标/资源不够/门限）时样本归零——WFOV 扫描检测在 kSearchAndStare 下没有独立外发通道，量测产品被 NFOV 交接门控；
3、双星可见窗几何错开时样本归零——交会天然要求近同时可见，这不是缺陷而是几何交会的定义域。
3、时间语义缺口（pass）：传感器输出、融合输入、评估配对三层均无时间字段；探针证明样本率由时间分辨率主导（dt=0.25s 时 320/320）。缺的是"量测打时间戳+按时间窗配对"的地面融合环节，不是交会算法。
4、相位协同（reject）：现实无此协同（用户裁定）；探针显示重叠由扇区布局主导；加同步会掩盖真实缺口。
5、极区补覆盖（defer）：布局维度，与时间议题正交。下一步探针=非 GEO 轨道集成需求出现时再立项（与 SBIRS-OQ-5 同族触发条件）。

6、需要用户拍板的问题：
1、Stage B 形态：本议题一次做完"场景层扫描示例 + 时间戳架构补位"，还是拆两步（先交付扫描场景示例，时间戳架构另立议题）？
2、扫描场景切换会使特征化基线（综合分 0.447267、dual_sat=80/80）失效，是否接受重新特征化（建立扫描态新基线）？
3、若做时间戳补位：时间先进哪一层——传感器输出层（扫描穿越时刻，改动大、最真实）还是先融合/评估层（周期号+窗宽放宽，改动小、但量测仍是抽帧产物）？

## §3 冻结契约（用户讨论结束后填写）

### 已证明的需求

1、甲方需要扫描模式双星定位场景示例；扫描机构已实现，场景层速率开启 + 评估口径适配一次做完（修订 1）。
2、评估配对缺"按时间窗对齐"语义：现状只有"严格同周期"，双星检出错开一个周期即丢样本，与"地面按时刻融合"的现实链路不对应。
3、扫描态重新特征化被接受（修订 2）：旧基线（综合分 0.447267、dual_sat 80/80）随场景切换失效，历史评审记录不回改。
4、时间语义本轮只进融合/评估层（修订 3）：周期号为时间基准，评估配对放宽为时间窗；传感器输出层不加时间戳。

### 允许范围

1、公开头一处增量：[evidence: include/1q/precision_evaluation/PrecisionEvaluationConfig.h] 加字段 `dual_sat_pair_window_cycles`（默认 0 = 严格同周期，零行为变化）。
2、[evidence: src/precision_evaluation/PrecisionEvaluationSession.cpp] ::Step/Impl：窗口内跨周期配对——缓存每星逐目标最新检出及该周期锚点（卫星位置 + GMST），窗内配对、各视线锚定各自测量周期。
3、测试：[evidence: tests/unit/precision_evaluation/precision_evaluation_session_test.cpp] 加聚焦测试（窗 0 逐位不变、窗内配对、超窗丢弃、跨周期锚定正确）。
4、示例场景：[evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json] 三颗星速率 0→6；同目录 main.cpp 解析窗宽字段进评估配置；同目录说明文档同步。
5、文档：[evidence: docs/precision_evaluation/algorithms.md] 与 boundaries.md 增窗口语义与偏差口径；[evidence: docs/common/open_questions.md] 登记传感器量测时间戳缺口与融合亚秒对齐两项开放问题。

### 明确禁止范围

1、传感器输出 DTO（SbirsDetectionRecord 等）不加时间字段；replay schema/编解码不动。
2、FusionEngine 与 DetectionRecord 不动。
3、`TryComputeDualLosFixM` 原语不动（时间无关纯几何）。
4、不加星间相位协同、不加扫描初始相位配置字段。
5、WFOV 外发门控与 kWideSearch 行为不动。
6、不降测试阈值；场景特征化按裁定重建，不弱化库内断言。

### 行为边界

1、输入：窗宽 0 = 现行为；窗宽 W>0 = 同目标双星检出相差不超过 W 个周期可配对。
2、输出：样本归属当前周期；两视线各锚定各自测量周期的卫星位置与 GMST；位置误差对照当前周期真值；每目标每周期至多一条双星样本。
3、错误回退：几何不可解（平行/出参空）与现有分支一致丢弃；缓存按目标滚动更新，不跨会话持久。
4、口径：跨周期样本天然含两测量间隔的目标运动偏差（速度×周期差），解算不做真值修正，口径说明写进 algorithms.md。

### 验收门

1、构建：1q_lib + precision_evaluation 测试目标 + 场景目标。
2、聚焦测试：窗口配对四项全过。
3、回归：既有 precision_evaluation 单测默认窗 0 零改动全绿。
4、场景：冒烟通过；扫描态新特征化基线（综合分 + dual_sat_cycles）记入 §4。

### 非目标

1、传感器层扫描穿越时刻时间戳（登记开放问题，再进入=扫描模式进正式验收需要亚周期精度）。
2、融合层亚秒时间对齐（同登记）。
3、极区 HEO 布局（defer）。
4、星间相位协同（reject）。

## 修订记录

修订 1（2026-08-31，用户指令）：Stage B 形态裁定"一次性做完"——扫描场景示例与评估窗口配对语义在本议题一并落地，不拆分议题。
修订 2（2026-08-31，用户指令）：接受扫描场景重新特征化——综合分 0.447267 与 dual_sat 80/80 基线随场景切换失效；历史评审文档（nadir-stare/scan-realism）中的特征化记录为当时事实，不回改。
修订 3（2026-08-31，用户指令）：时间语义裁定"先融合/评估层"——周期号为时间基准、评估配对放宽为时间窗；传感器输出层本轮不加时间戳，缺口登记开放问题。

## §4 运行记录（Stage C 后填写）

1、实现范围：
1、`PrecisionEvaluationConfig` 增 `dual_sat_pair_window_cycles`（默认 0 = 严格同周期，历史行为）[evidence: include/1q/precision_evaluation/PrecisionEvaluationConfig.h]。
2、`PrecisionEvaluationSession` 双星交会改为锚点缓存 + 窗口配对：每星逐目标缓存最近检出及测量周期锚点（卫星位置 + GMST），窗内配对、两视线各锚定各自测量周期、每目标每周期至多一条样本 [evidence: src/precision_evaluation/PrecisionEvaluationSession.cpp] ::Step。
3、示例场景切扫描模式：三颗星速率 0→6°/s（往复、nadir 基准），场景声明配对窗 1；main.cpp 解析 `precision.dual_sat_pair_window_cycles` [evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json]。
4、聚焦测试 4 项（窗 0 逐位不变、窗内跨周期配对 + 锚定正确性、超窗丢弃、窗内同周期非回归）[evidence: tests/unit/precision_evaluation/precision_evaluation_session_test.cpp]。

2、验证命令与结果（Release，VisualStudio.15.0-amd64）：
1、`1q_precision_evaluation_unit_tests`：26/26 pass（含新增 4 项；既有 22 项默认窗 0 零改动）。
2、回归：sbirs unit 263/263、replay 38/38、contract 14/14、integration 29/29、fusion 61/61、examples 134/134、public API 8/8、cross-domain 6/6 全 pass。
3、场景冒烟：`sbirs_triple_sat_fix_messages.exe` exit=0。
4、特征化（修订 2 接受重建）：扫描态新基线 综合分 0.430377、dual_sat_cycles=74/80、交会 RMSE 85 m；与探针 B（同参数无窗字段）逐位一致，证明窗宽 1 在跟踪在环配置下无行为漂移；缺失 6 周期为 B 星捕获起始几何延迟（期间 B 星无检出）。旧基线 0.447267/80/80（凝视态）作废，历史评审记录不回改。

3、权威回写去向：
1、窗口配对语义 + 偏差口径 + 反直觉点 → `docs/precision_evaluation/algorithms.md`。
2、"配对窗是编排会话消费规则、原语不动" → `docs/precision_evaluation/boundaries.md`。
3、SBIRS-OQ-6（量测时间戳 + 扫描初始相位缺失，含 WFOV 外发门控边界）与 FUSION-OQ-1（亚周期时间对齐缺失）→ `docs/common/open_questions.md`。
4、扫描态几何/预期事件表/特征化 → `examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.md`。

4、残留风险：
1、量测仍为整秒抽帧产物：窗口配对的时间基准是周期号，不是测量时刻；"穿越时刻"信息缺失登记在 SBIRS-OQ-6，本议题不解决。
2、跨周期样本含目标运动偏差（速度×周期差），口径已写入 algorithms.md；场景窗宽 1、目标 ~1.8 km/s 时偏差上限 ~1.8 km，远小于 10 km 参考误差，当前不影响评分形态。
3、kSearchAndStare 下 WFOV 扫描检测无独立外发通道（探针 C 实证）：捕获不可达时量测断供而非降频，属场景配置选择范围外的库行为，已随 SBIRS-OQ-6 边界登记。

5、后续冻结项：
1、SBIRS-OQ-6：传感器量测时间戳 + 扫描穿越语义 + 初始相位字段（再进入=扫描模式进正式验收需亚周期精度，或多星异步扫描集成需求）。
2、FUSION-OQ-1：DetectionRecord 时间字段 + 逐航迹滤波按量测时刻推进（再进入=随 SBIRS-OQ-6 立项）。
3、极区大椭圆轨道补覆盖（本议题 defer 项）：再进入=非 GEO 轨道集成需求。
