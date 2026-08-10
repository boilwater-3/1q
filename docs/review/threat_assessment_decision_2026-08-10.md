---
Status: draft
Date: 2026-08-10
Review-Baseline: `feature/threat-assessment`（新算法面决策记录，非单分支审查）
Authority: 目标威胁评估算法面（threat_assessment）双向评估与选型决策记录。
非规范性审查记录；不得替代 `docs/threat_assessment/{design,boundaries,algorithms}.md`
与 `docs/common/contract.md`。若本文与库实现冲突，以库为准。
---

# 目标威胁评估算法面：双向评估与选型决策

> **实施状态（2026-08-10）**：本文即"证据优先开发模式"（`docs/common/contract.md`
> 强制门禁）的 **Stage A 证据矩阵 + 冻结契约**。结论 4 项冻结全部 `pass`/`narrow`，
> 已进入 Stage B 实现，落地为新模块 `threat_assessment`（公开头 + 实现 + 单测 +
> 模块文档三件套 + 示例接线）。Stage C 回写见文末 §9。

## 0. 结论速览（大白话）

- 目标威胁评估 = 给每个目标算一个**威胁分/威胁等级**（高/中/低），供战术决策消费。
- 文献里威胁评估算法有很多种（本调研梳理了 8 大流派）。逐个对照本库
  **目标融合算法**（`fusion` 模块：多源关联 + 置信度融合，输出 `FusedTarget`）
  能给什么、缺什么，两边双向对照后选型。
- **双向评估结论**：
  - **文献 → 本库**：文献要的特征（速度/接近速率、距离、高度、加速度、RCS、
    类型概率、证据质量），`FusedTarget` 只给位置/方位/判决/质量/融合置信度——
    **缺运动学/RCS/类型概率**；这些在 AR `TrackStateSnapshot` 里有。→ 威胁评估
    的输入帧由**调用方组装**（融合输出 + 运动学快照），不动 fusion 冻结边界。
  - **本库 → 文献**：融合置信度 `[0,1]`（冻结公式，滑窗精确求和**不归一化、可 >1**）
    怎么进威胁评估，文献没有标准做法（只有 D-S 折扣 / ER 可靠性加权 / BN 证据似然
    三条路线）→ 本库自定义为**归一化属性**，写入设计文档成为空白点结论。
- **选型**：归一化加权和（MADM 多属性决策）——文献工程共识主流、可解释、实时、
  无训练数据；做**独立算法面** `threat_assessment`，与 `fusion`/`navigation` 平级。
- 现有 AR 决策层 `ThreatAssessmentEvaluator` 启发式**保持不动**（单传感器路径），
  新算法面独立存在；示例层接线优先消费新算法面。

## 1. 本库现状（证据）

| 资产 | 提供什么 | 证据 |
|---|---|---|
| `fusion` 模块 | `FusedTarget`：key、各源量测（位置/方位/特征/判决值/质量）、融合置信度、滑窗量测数；**无运动学**（boundaries 非目标 1：不做轨迹滤波） | `include/1q/fusion/FusedTarget.h`、`docs/fusion/boundaries.md:37` |
| 融合置信度公式（冻结） | confidence = Σ 判决值 × 质量归一化 × 权重，滑窗内精确求和，**不归一化可 >1** | `docs/fusion/algorithms.md` 反直觉点 |
| AR 决策层威胁评估 | `ThreatAssessmentEvaluator::ComputeThreatScore`：速度>300 +2、RCS>3 +1、确认态 +0.25；未归一化、缺距离/类型概率/融合置信度 | `src/airborne_radar/decision/ThreatAssessmentEvaluator.cpp:122-145` |
| AR 轨迹快照 | `TrackStateSnapshot`：位置/速度/加速度三分量+模长、RCS、类型/识别概率、滤波不确定度迹 | `include/1q/airborne_radar/session/TrackStateSnapshot.h` |

## 2. 方向 A：文献算法要什么 ↔ 融合输出给什么

文献威胁评估常用特征（依据见 §8 文献）与本库两个数据源的逐项对照：

| 特征 | 文献依据 | FusedTarget 提供 | TrackStateSnapshot 提供 | 结论 |
|---|---|---|---|---|
| 速度 / 接近速率 | TEWA 综述、Coskun 2022、Yin 2023 | ✗（无运动学） | ✓ speed + 速度向量 | 需调用方组装 |
| 距离（斜距） | 同上 | ✓（LLA 位置可算） | ✓（笛卡尔位置模长） | 可直接用 |
| 高度 | JSEE/火力与指挥控制多篇 | ✓（LLA 高度） | ✓（z 分量） | 可直接用 |
| 航向 / 切入角 | Okello & Thoms 2003、Ma 2017 | ✗（仅 az/el 方位） | ✓（速度三分量可算） | 首期用接近速率代替，非目标 |
| 加速度 / 机动性 | Ma 2017、Sun 2019 | ✗ | ✓（加速度三分量+模长） | 需调用方组装 |
| RCS | Coskun 2022、JSEE 2018 | ✗（融合不融合 RCS） | ✓ | 需调用方组装 |
| 目标类型 / 识别概率 | Okello & Thoms 2003 | ✗（仅库内身份键） | ✓（类型+概率） | 需调用方组装 |
| 证据质量 / 多源一致性 | Mercier 2008、Fan 2022 | ✓（verdict×quality、滑窗量测数） | ✗ | **本库独特优势** |
| 融合置信度 | 见 §3 | ✓（confidence，可 >1） | ✗ | 本库自定义消费方式 |
| 编队规模 / 意图 | Kong 2018、Salerno 2010 | ✗ | ✗（可派生但首期不做） | 非目标 |

**方向 A 结论**：融合输出是"**证据侧**"（多源一致性与置信度），运动学/RCS/识别是
"**属性侧**"。文献算法两类都要 → 威胁评估输入帧 = 融合输出 + 运动学快照的组装，
组装责任在调用方（与 `DetectionRecord` 泛型化的同构做法），fusion 冻结边界不动。

## 3. 方向 B：融合输出 ↔ 文献算法（置信度怎么被消费）

文献中"源可靠性/置信度进入威胁评估"有四条路线：

1. **D-S 证据折扣**：按可靠性对 BPA 折扣，质量低则证据向无知退化（Mercier 2008；
   Yang 2013）；
2. **证据推理 ER 可靠性权重**：MADM 里给每属性同时设权重+可靠性（Yang & Xu 2002，
   被引 884）；
3. **贝叶斯证据似然**：观测节点携带不确定度做端到端传播（Okello & Thoms 2003；
   Di 2018）；
4. **不确定性建模**：云模型/广义证据理论把评估本身建在不确定环境上（Ma 2017；
   Zhou 2022）。

**空白点（本库写进设计文档的结论）**：文献没有"把融合器给出的 `[0,1]` 融合置信度
直接作为威胁评估属性"的标准做法——融合置信度通常是融合过程的中间产物，不对外承诺
语义。本库 `FusedTarget.confidence` 是**冻结的精确求和公式**（不归一化、可 >1，
`docs/fusion/algorithms.md` 反直觉点），作为威胁属性时必须先**钳制归一化到 [0,1]**，
语义为"证据强度"，与威胁分做正相关加权。该做法记为 threat_assessment 的设计决策
（`docs/threat_assessment/algorithms.md` 反直觉点）。

## 4. 八大流派对比（选型依据）

| 流派 | 特征需求 | 吃融合置信度 | 复杂度 | 可解释性 | 先验/训练 | 选型 |
|---|---|---|---|---|---|---|
| 加权和 / AHP（MADM） | 属性向量+权重 | 可作属性 | 极低 O(n·m) | 高 | 专家权重 | **选中** |
| TOPSIS / VIKOR | 属性矩阵+权重 | 弱 | 低 | 高 | 权重 | 排序场景，加权和可覆盖 → 非目标 |
| 灰色关联分析 | 属性序列+理想序列 | 弱 | 低 | 高 | 参考序列 | 非目标（理想序列主观） |
| 模糊综合/规则 | 属性+隶属函数+规则库 | 弱-中 | 低-中 | 中-高 | 规则库（重） | 非目标（规则膨胀） |
| D-S 证据 / ER | 各源 BPA+折扣 | **强** | 中 | 中-高 | BPA 构造、折扣参数 | 非目标（未来冻结项，见 §5） |
| 贝叶斯网络 / DBN | 属性观测+CPT | 强 | 中-高 | 中 | CPT/结构 | 非目标（CPT 标定重） |
| 神经网络 / DL | 属性向量+标注样本 | 弱 | 推理低/训练高 | 低 | **标注数据（军用稀缺）** | 排除 |
| 直觉模糊集族（IFS/IVIFS） | 属性+隶属/犹豫度 | 中 | 中 | 中 | 隶属度标定 | 非目标（叠加在 MADM 上的扩展） |

工程共识支撑（Roux & van Vuuren 2007 TEWA 综述；Coskun 2022 IEEE TAES）：作战实时
决策场景以**规则/加权/MADM 为主流**，贝叶斯/D-S 做不确定性增强；纯数据驱动因标注
样本稀缺与可解释性要求多用于离线。→ 加权和是首期正确选型。

## 5. 选型结论与明确不做

- **选中**：归一化加权和 MADM。属性 = 距离、速度、加速度、RCS、类型概率、融合置信度
  （6 项）；每属性归一化到 [0,1] 后加权求和，钳制 [0,1]，映射等级（高/中/低）。
  权重与归一化参考值全部可配置（`ThreatEvaluatorConfig`）。
- **不做（写进 boundaries.md 非目标）**：
  - D-S/ER、BN、NN、TOPSIS、GRA、模糊推理、IFS 族——本首期不实现；
  - 武器分配（TEWA 的 WA 部分）、拦截策略——威胁评估只出分数/等级；
  - 解析场景真值/外部身份通道（守去真值化纪律，与 fusion 一致）；
  - 跨周期记忆/平滑（首期纯函数式，输入帧由调用方组装，历史信息由调用方携带）；
  - 编队规模/意图建模。
- **与现有 AR 启发式的关系**：`ThreatAssessmentEvaluator` 不动（单传感器路径保留）；
  新算法面消费融合+运动学组装帧，二者并存，示例层接线选择消费新算法面。

## 6. Stage A 证据矩阵

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|---|---|---|---|---|---|---|
| F1 输入帧由调用方组装（运动学来自 AR 快照，不扩 fusion） | 威胁评估需要的速度/加速度/RCS/类型概率融合输出不提供，但 AR 快照提供 | `FusedTarget.h`、`TrackStateSnapshot.h` 字段清单 | 头文件字段核对（本 §2 方向 A 表格） | 字段全部可在既有公开类型中找到，无需改 fusion | 必须修改 `FusedTarget`/`DetectionRecord`/`FusionConfig` 才能表达输入 | pass |
| F2 融合置信度钳制归一化后作为属性 | confidence 冻结公式不归一化可 >1，直接加权会破坏威胁分 [0,1] 语义 | `docs/fusion/algorithms.md` 反直觉点、`FusionEngine.cpp` | 单测喂 confidence>1 输入验证钳制 | 威胁分始终 ∈ [0,1]，且 confidence 增大威胁分不降（单调） | 钳制引入不可解释的非单调 | pass |
| F3 归一化加权和选型 | 文献工程共识：实时决策以加权和为主流，无需训练数据 | 本 §4 对比表（Roux & van Vuuren 2007；Coskun 2022 等） | 文献比对 + 原型实现复杂度评估 | 满足实时/可解释/无训练三项需求 | 需求包含概率语义或不确定性表达（本首期无此需求） | pass |
| F4 新算法面纯函数式无跨周期状态 | 威胁分可由单帧输入独立计算；跨周期记忆属调用方 | `ThreatAssessmentEvaluator` 现有跨周期 state_store 属决策层职责 | 设计评审：Evaluate(inputs)→results 无状态成员 | 同输入同输出，可并发可重入 | 场景验证显示必须跨周期才能出正确威胁分 | narrow（首期无状态；若场景证伪再回 Stage A） |

## 7. Frozen Contract

```markdown
## Frozen Contract

Proven requirement:
- 提供目标威胁评估算法面：输入目标属性帧（运动学 + 证据 + 融合置信度），
  输出 [0,1] 威胁分 + 等级（HIGH/MEDIUM/LOW）+ 每属性贡献分解；
- 输入由调用方组装（fusion 输出 + AR 快照），算法不感知具体传感器类型。

Allowed scope:
- 模块：include/1q/threat_assessment/（公开头）、src/threat_assessment/（实现）；
- 类：ThreatEvaluator（PIMPL）、ThreatEvaluationInput、ThreatEvaluatorConfig、ThreatResult；
- 算法：归一化加权和（6 属性），等级映射，可配置权重/参考值/阈值；
- 文档：docs/threat_assessment/{design,boundaries,algorithms}.md、docs/review/ 本文；
- 测试：tests/unit/threat_assessment/；示例：examples/component_attachment 新组件+场景。

Explicitly out of scope:
- 公开头：不动 include/1q/fusion/、include/1q/airborne_radar/ 任何现有类型；
- 跨模块类型：不改 FusedTarget/DetectionRecord/FusionConfig/TrackStateSnapshot；
- 算法：D-S/ER/BN/NN/TOPSIS/GRA/模糊/IFS、武器分配、意图/编队建模、跨周期记忆；
- 真值：不解析场景真值、不建外部身份通道。

Behavior boundary:
- 输入：非负属性值；负值/NaN 按属性缺失处理（归一化 0 并记入贡献分解）；空输入输出空；
- 输出：威胁分钳制 [0,1]；等级映射阈值可配置（默认 HIGH≥0.7 / MEDIUM≥0.4 / LOW）；
- 确定性：同输入同输出，输出按输入顺序；
- 纯函数：无跨周期状态，Evaluate 可重入可并发。

Acceptance gates:
- 构建：llvm-ninja-release-local 全量构建通过；
- 聚焦测试：tests/unit/threat_assessment/ 全绿（归一化边界、加权和、钳制、等级映射、
  空/非法输入、确定性、贡献分解 Σ=总分）；
- 集成：examples::component_attachment demo 单跑通过，威胁场景预期表核对一致；
- 契约：completeness-review 门禁通过（major 变更）。

Non-goals:
- 不预测未来（仅当前帧）；
- 不输出武器分配/拦截建议；
- 不与 AR 决策层 ThreatAssessmentEvaluator 合并（各自保留）。
```

## 8. 参考来源（文献）

流派与工程共识依据（均经 OpenAlex/Crossref 核实，未编造）：

- Saaty, T.L., *The Analytic Hierarchy Process*, McGraw-Hill, 1980（AHP 奠基）
- Hwang, C.-L., Yoon, K., *Multiple Attribute Decision Making*, Springer, 1981（MADM/TOPSIS 源头）
- Roux, J.N., van Vuuren, J.H., "Threat evaluation and weapon assignment decision support: A review of the state of the art", ORiON 23(2), 2007, DOI 10.5784/23-2-54（TEWA 综述）
- Salerno, J. et al., "Issues and challenges in higher level fusion: Threat/impact assessment and intent modeling", Proc. 13th Intl. Conf. on Information Fusion, 2010, DOI 10.1109/icif.2010.5711862（JDL Level 3）
- Steinberg, A.N., Bowman, C.L., White, F.E., "Revisions to the JDL data fusion model", Proc. SPIE 3719, 1999, DOI 10.1117/12.341367（JDL 模型）
- Okello, N., Thoms, G., "Threat assessment using Bayesian networks", Proc. 6th Intl. Conf. on Information Fusion, 2003, DOI 10.1109/icif.2003.177361
- Coskun, M. et al., "Fuzzy Logic-Based Threat Assessment Application in Air Defense Systems", IEEE Trans. AES, 2022, DOI 10.1109/taes.2022.3212032
- Ma, S. et al., "Target threat level assessment based on cloud model under fuzzy and uncertain conditions in air combat simulation", Aerospace Science and Technology, 2017, DOI 10.1016/j.ast.2017.03.033
- Gao, Y. et al., "A novel target threat assessment method based on three-way decisions under intuitionistic fuzzy MADM", Engineering Applications of AI, 2019, DOI 10.1016/j.engappai.2019.103276
- Fan, C. et al., "A New Model of Interval-Valued Intuitionistic Fuzzy Weighted Operators and Their Application in Dynamic Fusion Target Threat Assessment", Entropy, 2022, DOI 10.3390/e24121825（题目即"融合目标威胁评估"）
- Yang, J.-B., Xu, D.-L., "On the evidential reasoning algorithm for multiple attribute decision analysis under uncertainty", IEEE Trans. SMC-A 32(3), 2002, DOI 10.1109/tsmca.2002.802746（ER 可靠性权重）
- Mercier, D., Quost, B., Denœux, T., "Refined modeling of sensor reliability in the belief function framework using contextual discounting", Information Fusion 9(2), 2008, DOI 10.1016/j.inffus.2006.08.001（D-S 折扣）
- Wang, G.-G. et al., "Wavelet Neural Network Using Multiple Wavelet Functions in Target Threat Assessment", The Scientific World Journal, 2013, DOI 10.1155/2013/632437（NN 代表）
- Yin, Y., Zhang, R., Su, Q., "Threat assessment of aerial targets based on improved GRA-TOPSIS method and three-way decisions", Mathematical Biosciences & Engineering, 2023, DOI 10.3934/mbe.2023591

中文机构论文（OpenAlex/Crossref 收录，CNKI 全文不可达，引用摘要级结论）：
Sun 2019 JSEE DITOPSIS、NWPU 2018 JSEE 动态 VIKOR、Kong 2018 Applied Soft Computing
IVIFS 群目标、Zhou 2007 / Xia 2014 火力与指挥控制灰关联、王 2014 电子学报 IFS-MADM。

## 9. Stage C 回写（实现完成后更新）

| 项 | 实际结果 |
|---|---|
| 实现范围 | 新模块 `threat_assessment`：公开头 5（ThreatEvaluationInput / ThreatEvaluatorConfig / ThreatResult / ThreatEvaluator / threat_assessment.hpp）+ 实现 1 + CMake 注册（src/CMakeLists.txt + 新模块 CMakeLists + Unit.cmake partition）；单测 13 用例；docs 三件套；示例接线 8 文件（threat_component、events/signals/demo_log/demo_output/scene_data/demo 装配/CMake）+ 场景 `threat_multi_target.{json,md}` + README |
| 验证命令与结果 | `cmake --build --preset llvm-ninja-release-local` 通过；`ctest -R threat_assessment` 13/13 通过；`ctest -R component_attachment` 通过（21.4s）；威胁场景单跑：三目标威胁分 0.72/0.42/0.33（与几何先验手算一致）、等级高/中/低、升级事件与 ENGAGE_HIGH_THREAT 指令 cycle 1 触发、threat_views=400；baseline 场景零回归（fused 峰值 23 等既有行为不变） |
| 残留风险 | ①融合目标早期膨胀（3→18→4）为 fusion 既有行为（无身份探测互不合并），非本模块引入；②示例层无加速度/类型概率数据源 → 两属性贡献恒 0（威胁分偏保守，文档已注明）；③F4 narrow 验证通过：纯函数式单帧评估在场景中行为正确，跨周期记忆确属调用方职责 |
| 后续冻结项 | D-S/ER 折扣消费融合置信度（不确定性表达，若未来需要）；接近速率/高度属性扩展；融合侧运动学补全（未来 fusion 轨迹滤波后）；示例层加速度/类型概率数据源补全；跨周期平滑/记忆 |

**Stage A 决策更新**：F1/F2/F3 `pass`、F4 `narrow`（首期无状态，场景验证未证伪）。
实现中发现并修正一处设计缺陷：`ThreatLevel` 枚举初始序（kHigh=0）导致"升级判定
（level > prev）"语义颠倒——已改为等级递增序（kLow=0 < kMedium < kHigh），
单测与场景验证均覆盖（首见升级事件 cycle 1 正确触发）。
