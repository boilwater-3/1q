---
Status: draft
Date: 2026-09-03
Review-Baseline: `evidence/rir-polarization-discrimination` @ `8f072ffe`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# rir-polarization-discrimination：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

触发来源：甲方需求（用户转述，2026-09-03）——输入极化散射数据（目标在 H/V 发射×接收组合下四路复数散射响应，统一双通道结构），输出行列式/总功率/去极化系数/方向角/椭圆率各（平均值+标准差）以及目标类型（枚举 RV/HD/LD/DB；HD=重诱饵、LD=轻诱饵）。
- **证据**：[evidence: src/remote_identification_radar/runtime/PolarizationAcceptanceS.cpp]（现有旁路恰好覆盖五量中的单值计算，缺统计与分类）

背景白话：
1、极化散射矩阵（Sinclair S）：雷达用水平/竖直两种极化"提问"、听两种极化"回答"，四次问答组成 2×2 复数表——目标形状的极化"指纹"。
2、现有实现是"验收旁路"：只算单值写日志、不进识别判决；识别走 SQLite 特征库匹配另一条链。
3、本需求=三件事：①输入升级为真四路复数字典；②五个量配均值/标准差（统计特征）；③新增 RV/重诱饵/轻诱饵/DB 四类判决输出。
4、讨论已达成的方向性共识（本轮多轮对话裁定）：
   1、真实雷达对照：开机载入的是"类型手册"（特征库，库内一次性载入，现状正确）；个体真值字典是仿真替身，随周期输入流动。
   2、架构 B：甲方大表在场景层开机一次性载入，每周期只把"当前视角附近窗口行"（十几行）放进周期输入；库的输入契约形状不变。
   3、类型手册归库、个体字典归场景——SQLite 特征库不改窗口化。
5、重要外部依赖：甲方已提供一张大数据表格（用户口述"数据特别多"），**原件未到手**——判决判据、DB 语义、验收行格式均依赖该原件。

待裁定项总览（四问展开见 §1）：
1、旁路转正的需求成立性。
2、架构 B 冻结（场景层载入+窗口流入）。
3、新旧输入结构并存策略。
4、统计窗口口径（窗口=姿态扇区行；跨度待定）。
5、方向角/椭圆率周期量的统计口径（圆统计）。
6、RV/HD/LD/DB 枚举落位（平行字段 vs 扩现有枚举）。
7、判决判据来源（阻塞项，依赖甲方原件）。
8、DB 语义确认（依赖甲方原件）。
9、验收行格式对齐（依赖甲方原件）。
10、非目标边界（特征库窗口化、加噪测量升级、他模块联动）。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、旁路转正需求成立 | 现有验收旁路无法满足新需求（单值无统计、不进识别、无四类输出），升级为识别正式链路成立 | [evidence: src/remote_identification_radar/runtime/PolarizationAcceptanceS.h]（自述"不进识别"）；[evidence: include/1q/remote_identification_radar/session/RirRecognitionResult.h]（七粗类枚举无弹道细分） | 验收日志检视（已执行）：`build/VisualStudio.15.0-amd64/tests/log/rir_acceptance.log` 极化行仅"目标类型=未知"共 7 行，五个数值行因场景无数据整体省略；枚举检视（已执行）：RirRecognitionCategory=弹道目标/临近空间/其它/未知/战斗机/轰炸机/导弹 | 立项：允许范围=旁路转正+四路复数输入+统计+分类输出 | 甲方另有实现通道或需求撤回 | pass |
| 2、架构 B：场景层一次性载入+窗口流入 | 大表零重复进库且爆炸半径最小；库输入契约形状不变 | [evidence: include/1q/remote_identification_radar/session/RirSceneTypes.h]（RirCycleInput 每周期自描述契约）；[evidence: examples/scenes/scene_data.cpp]（开机一次性读场景文件先例）；[evidence: src/remote_identification_radar/runtime/RirController.cpp]（database_path 库内载入先例=类型知识归库） | 检视（已执行）：1、现有铺表=一组数抄 15 遍占位（scene_script.cpp 极化铺样循环）；2、grep 全部场景 JSON：pol_ch1_dbsm 零出现（无存量场景消费）；推理：窗口 12 行×8 数≈1KB/周期，全表 360 行级——窗口方案体积可控 | 冻结 B；A 变体（公共 API 一次性载入，仿 database_path）留档备选 | 甲方表格实测规模超出窗口方案承受力（表未到手，暂无法测） | pass（B） |
| 3、新旧输入并存 | 新四路复数结构与旧 dBsm+has_* 结构并存，旧场景/旧回放逐位不变 | [evidence: include/1q/remote_identification_radar/session/RirSceneTypes.h]（RirPolarizationRcsSample 冻结口径：0 dBsm 合法、has_* 声明、不得把 0 当缺省）；[evidence: schemas/replay/rir_replay.fbs]（旧样本表在回放 schema 中） | grep 场景 JSON（已执行）：旧键零出现——无场景消费者；回放 roundtrip 测试现状（已执行，全量 63/63 内含） | 新结构并行加入 RirSceneTarget 与回放 schema 新表；旧结构仅既有验收旁路继续消费 | 甲方要求旧结构废弃 | pass |
| 4、统计窗口口径 | 均值/标准差的统计总体=场景按当前视角裁剪的姿态扇区窗口行（语义：目标在当前姿态附近的起伏） | 本轮讨论共识（窗口即统计总体）；[evidence: examples/scenes/scene_script.cpp]（场景掌握雷达与目标位置，可算当前视角） | 推理：窗口跨度决定标准差含义，跨度未定则统计口径悬空；甲方文本未给窗口定义 | 用户裁定窗口跨度缺省值（建议 ±5°，可配置） | 甲方文档另有定义（如按周期数累计） | narrow（缺省 ±5°，待原件核对） |
| 5、方向角/椭圆率周期量统计口径 | ψ∈±90°、τ∈±45° 环绕，算术平均在缠绕处出错（+89° 与 −89° 物理同向，算术均值 0° 错误），须圆统计 | [evidence: src/remote_identification_radar/runtime/PolarizationAcceptanceS.cpp]（ψ=½atan2、τ=½asin 值域即环绕区间） | 推理：缠绕反例成立（无需求测）；未验证项=圆统计公式选型（均值角+角度散布） | 冻结圆统计口径（均值角+角标准差），公式进冻结契约 | 用户裁定展平规则（如统一搬运到连续区间） | pass |
| 6、RV/HD/LD/DB 枚举落位 | 四类是"弹道目标"粗类的细分；平行新字段可保留粗类不动 | [evidence: include/1q/remote_identification_radar/session/RirRecognitionResult.h]（现有七类枚举与 target_category 消费面）；[evidence: include/1q/remote_identification_radar/config/RirPolicyConfig.h]（polarization_weight=0.25 极化权重占位） | 枚举检视（已执行）：现七类；验收行 CategoryName 映射"弹道目标"等中文名 | 用户裁定：平行字段（建议）vs 扩枚举 | 甲方要求替换粗类体系 | narrow（建议平行字段） |
| 7、判决判据来源（阻塞项） | 五个统计量→四类的映射判据存在于甲方材料（阈值表或样本库）；无判据则判决层只能做空壳 | 甲方已给大数据表格（用户口述，未见原件） | 无法执行——表格与判据文本均未到手 | 原件到手后：判据转正进冻结契约（阈值表/特征库维度/演示规则三选一） | 甲方无判据→判决层做演示级规则并登记开放议题 | defer（依赖甲方原件） |
| 8、DB 语义确认 | DB=碎片/残骸类（Debris/Booster，弹体碎片与助推器残骸合并类） | 推理：诱饵已按轻重分完，第四类自然候选为非威胁残骸类；DB 非通用标准缩写 | 无法执行——需甲方原文 | 原件到手后冻结枚举命名与中文注释 | 甲方另有定义 | defer |
| 9、验收行格式对齐 | 第 46 项五行改"均值/标准差"双值+目标类型写真判决，格式逐字对齐甲方验收文本 | [evidence: src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp]（第 46 项现有五行单值格式与 EmitOrNone 省略规则） | 检视（已执行）：现有五行=行列式/Span/去极化/方向角/椭圆率，单值；甲方验收文本未到手 | 原件到手后逐字对齐；方向先行（双值格式） | 甲方格式与现状不可调和 | narrow（方向先行，格式等原件） |
| 10、非目标边界 | 以下明确不做：1、SQLite 特征库改窗口化（类型知识归库、整库比对无窗口概念）；2、RIR-OQ-1 加噪测量保真度升级（真值×效能→加噪量测，独立议题已登记）；3、其他模块（AR/ESR/ECM）极化散射联动 | [evidence: src/remote_identification_radar/recognition/RecognitionFeatureDatabase.cpp]（特征库整库匹配消费方式）；[evidence: docs/common/open_questions.md]（RIR-OQ-1 已登记） | 检视（已执行）：特征库消费=特征向量整库比对，无按角度查行语义 | 冻结契约列非目标 | 用户或甲方要求联动 | reject（列入非目标） |

## §2 判定汇总与待裁定问题

判定汇总：
1、旁路转正需求成立：pass——现有实现三处缺口（单值/不进识别/无分类）均有实证。
2、架构 B：pass——大表零重复进库、库契约形状不变、复用场景层开机读入先例；A 变体留档备选。
3、新旧输入并存：pass——旧 dBsm 口径是冻结验收约定，无存量场景消费者，并存代价最低。
4、统计窗口：narrow——窗口=姿态扇区行已共识；跨度缺省建议 ±5° 可配置，待甲方文本核对。
5、ψ/τ 圆统计：pass——缠绕反例数学上成立，冻结圆统计口径。
6、枚举落位：narrow——建议平行"弹道细分类型"字段，粗类七值不动。
7、判决判据：defer——甲方表格与判据文本未到手，判决层被阻塞；输入层+测量层可先行。
8、DB 语义：defer——推理为碎片/残骸类，等原件确认。
9、验收行格式：narrow——方向先行（双值+真判决），格式逐字对齐等原件。
10、非目标：reject——特征库窗口化、加噪测量升级、他模块联动不碰。

需要用户拍板的问题：
1、架构 B 是否按建议冻结？（大表场景层载入、每周期窗口流入、库契约不变）
2、窗口跨度缺省值取 ±5° 是否同意？
3、枚举落位：平行字段（建议，粗类不动）还是扩现有枚举？
4、落地场景载体：扩展现有 rir_ground_site_recognition，还是新建弹道场景（ballistic_trajectory 组件已在 examples/scenes，四类目标是弹道语义）？
5、甲方表格原件与判据文本何时可到手？（阻塞第 7/8/9 项；输入层+测量层可先行实现）

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、爆炸半径与回滚：下游消费方影响、回退难度（无损/破坏性/回滚注意点）。
5、验收门：构建、聚焦测试、契约测试、特征化测试、探针转正（有回归价值的探针转正式测试）。
6、非目标。
-->

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

## §4 运行记录（Stage C 后填写）

<!-- 对照强制回写清单勾项，全部完成才允许拆脚手架（见 SKILL.md 收尾规则）：
1、实现范围：改动文件与接口。
2、验证命令与结果：命令: pass/fail（含转正的探针测试）。
3、权威回写去向：
   1、正向边界：docs/<module>/design.md（boundaries/data-flow/algorithms 按归属选）。
   2、否决记录：docs/<module>/design.md 的"架构裁定与否决记录"专节（体裁见 docs-governance-standard）。
   3、开放议题：docs/common/open_questions.md 登记（编号）。
   4、证据锁：新增/修订规则后附 - **证据**：[evidence: 路径]。
4、残留风险。
5、后续冻结项。
-->
