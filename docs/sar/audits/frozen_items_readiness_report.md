# SAR 冻结项实际开发度评估报告(解冻前探索)

Date: 2026-06-23
修订: 2026-06-23(v2)— 纠正"零测试"结论:测试曾存在,在冻结提交中被一并删除,可从 git 历史恢复。

> ## ⚠️ 状态更新(2026-06-24):本报告的"🥇 先解冻"建议已执行
>
> 本报告撰写于 `68dd8213` 解冻**之前**(2026-06-23),评估的是 `ce4aa2d9` 冻结状态下的开发度。
> 其后提交 **`68dd8213`**(2026-06-23 "unfreeze SarFocusingSelector, RadiometricCalibration and
> Omega-K component chain")**已执行本报告 §5 建议的 🥇 优先级项**:
>
> - ✅ **辐射定标** + **自适应选择 Auto** 已解冻(恢复 6+4 单测、清横幅、重新注册进 `SAR_ENGINE_SOURCES`)。
> - ✅ **Omega-K 组件链**(14 源)已解冻,52 部件测试恢复。
> - ✅ 护栏 `check_sar_frozen_sources.cmake` 的 `FROZEN_SAR_SOURCE_PATTERNS` **收窄到仅剩 `sar/imaging/SarCsa`**。
>
> 因此:
> - **§3 总览矩阵**与**§4 逐项结论**反映的是**解冻前**快照,保留作 git 历史参考与技术评估依据;其中的"解冻工作量/难度"列对已解冻三项不再适用。
> - **§5 解冻优先级建议**:🥇(辐射定标 + Auto)、🥈(Omega-K 部件)已落地;🥈 标注的"补 front-end+orchestrator"与 ⚠️PGA、🔴CSA、🔴二阶补偿仍未完成。
> - 当前权威状态以 `docs/sar/design/module_design.md` v2.6「已实现但未接入数据链路的模块」章节为准。
>
> 注:解冻三项虽纳入构建,但**均未接入 `SarSession` 数据链路**——Selector 是纯咨询(session 用 policy 布尔位绕过)、辐射定标属后处理标标、Omega-K 缺 orchestrator。未接线原因详见 module_design v2.6。

## 1. 背景与目的

用户指出 6 个冻结项(CSA、Omega-K、自适应选择 Auto、辐射定标、PGA 闭环、二阶运动补偿)
在本阶段**可以解冻**,但核心问题是这些功能的**实际开发程度**。本报告基于对源码的逐文件
深度探查(每项含 file:line 证据),给出"如果解冻,距离生产可用还差多少"的结论。

## 2. ⚠️ v1 报告的重大纠正:测试曾存在,被冻结提交一并删除

v1 报告基于当前工作树判定 5 项"零活体测试",并据此质疑辐射定标验收报告"5/5 通过"的可信度。
**该判定是错误的**。经 git 历史核查:

- 提交 **`ce4aa2d9`(2026-06-22)"chore(sar): mark frozen modules deprecated and drop their tests"**
  在**同一个冻结动作**里做了两件事:
  1. 给 Omega-K / CSA / FocusingSelector / RadiometricCalibration 的 **38 个源文件**统一加
     `【未进行设计需求,不再扩展 — DEPRECATED】` 废弃横幅(纯注释,每文件 6 行)。
  2. **删除对应的 19 个测试**(diffstat: 1698 行删除)。
- 提交信息明确:**"冻结代码保留以维持可恢复性,仅排除构建并加警示,不改变任何逻辑"**。
- 被删测试在 `ce4aa2d9^` 完整保留(blob 类型已验证),**可 `git checkout ce4aa2d9^ -- <test>` 直接恢复**,无需重写。

### 被删测试规模(从 `ce4aa2d9^` 读取,精确统计)

| 冻结项 | 被删测试文件数 | TEST() 数 | 被删行数 |
|---|---|---|---|
| Omega-K | **15** | **52** | ~1194 |
| CSA | 2 | 9 | ~221 |
| FocusingSelector(Auto) | 1 | 4 | 102 |
| RadiometricCalibration | 1 | 6 | 181 |
| **合计** | **19** | **71** | **~1698** |

> PGA / autofocus 测试**未被删除**(估+真值链仍在构建中),当前 4 个测试文件现存:
> `sar_autofocus_phase_truth_test`、`sar_pga_phase_gradient_estimator_test`、
> `sar_pga_support_gradient_truth_test`、`sar_pga_gradient_truth_comparison_test`。

### 对评估的颠覆性影响

- **v1 的"零测试"风险基本消除**:解冻时测试可从 git 恢复,71 个 TEST 覆盖各冻结项部件。
- **辐射定标验收报告"5/5 通过"现在可信**:测试确实存在过,是被冻结提交删除的,不是从未提交。
- **解冻工作量普遍下调**:原"需补测试"变为"需恢复测试 + 适配 DEPRECATED 横幅后的头文件"。
- 冻结是**一致的、可逆的整体动作**,解冻也应是整体逆操作(恢复测试 + 去横幅 + 加回 manifest)。

## 3. 修正后的总览矩阵

> **注(2026-06-24)**:下表反映 `68dd8213` 解冻**前**的冻结状态。辐射定标、自适应选择 Auto、Omega-K 组件链已在该提交解冻;"解冻到生产可用 / 难度"列对这三项已不适用。CSA 仍冻结,二阶补偿仍零实现。

| 冻结项 | 代码存在量 | 逻辑完成度 | 测试(恢复后) | 解冻到生产可用 | 难度 |
|---|---|---|---|---|---|
| **PGA 闭环** | 估+真值链 ~35% 已建 | 35%(闭环 0%) | 估+真值有(现存活) | 大(闭环集成/unwrap/迭代/apply) | 🟠 中 |
| **Omega-K** | 14 文件 ~3500 行 | 部件级完整,**管线未组装** | **52 TEST 可恢复** | 中(补 front-end+orchestrator,测试可恢复) | 🟠 中 |
| **辐射定标** | 2 文件 ~340 行 | 对**冻结契约**完整 | **6 TEST 可恢复** | 小(恢复测试+去横幅) → 中(若要全雷达方程) | 🟢 小 |
| **自适应选择 Auto** | 2 文件 ~190 行 | **逻辑完整** | **4 TEST 可恢复** | 极小(恢复测试+去横幅+调用方) | 🟢 小 |
| **CSA** | 4 文件(几何+真值oracle) | **主流程完全未实现** | 9 TEST(仅覆盖几何+oracle) | 极大(主流程从零,测试只护几何) | 🔴 大 |
| **二阶运动补偿** | **零**(无符号、无脚手架) | 0% | 2(阶段 A 证据矩阵,2026-06-24 新增) | **不实现**(阶段 A 判定:失效根因=RDA 转弯假设崩溃) | ✅ 证据封存 |

## 4. 逐项结论(已据测试恢复事实修正)

### 4.1 PGA 闭环 —— 🟠 中难度,已有 35% 骨架(测试现存活)

**已建且真实(非 stub)**:
- `SarPgaPhaseGradientEstimator`:真实共轭乘积 `arg()` 梯度,真实消费 `support_mask`(`.cpp:53-55`)。
- `SarAutofocusPhaseTruth`:真实多项式相位注入 + 闭式最小二乘分离不可观测分量(`.cpp:50-55`)。
- `SarPgaGradientTruthComparison`:真实 wrapped 误差 + RMS/max 指标(`.cpp:75-91`)。
- **测试现存且活体**(4 文件),估+真值链覆盖到 1e-12 精度。

**完全缺失(闭环 0%)**:梯度积分、unwrap(只有 Wrap 无逆)、`applyCorrection`、迭代+停止准则、统一 `Autofocus` 编排器、MapDrift/ContrastOptimization。4 模块只吃 1-D `ComplexVector`,**从不触碰生产 2-D 图像**,RDA/GBP/session 零引用。

**结论**:闭环 65% 待写,且需解决 1-D 剖面→2-D 图像桥接。工作量主要在闭环,不在已建部分。

### 4.2 Omega-K —— 🟠 中难度,部件完整+52 测试可恢复

**部件级真实(生产级)**:
- Stolt 插值(`SarOmegaKStoltInterpolation.cpp:42-145`):真实二分+线性,严格拒绝越界。**5 个 TEST 曾覆盖**(恢复后即有)。
- 真值链:**手写 SHA-256 确认**(`SarOmegaKTruthPayloadDigest.cpp:20-116`),ingestion/manifest/eligibility/orchestrator 真实且内部接线。**truth_* 测试 13 个可恢复**。
- 6 个数值阶段各自完整,16 个声明函数 1:1 有定义,**可编译**(符号全 resolve)。

**致命缺口(测试不能弥补)**:
- **无 `FocusOmegaK` 编排器**,无函数把阶段串起来 —— 阶段靠字段名约定隐式链接,无调用方驱动。
- **无 front-end**:chain 起点 `ExecuteOmegaKExplicitGridReduction` 要 `source_spectrum`(已算好的 2-D 波数谱),但**没有从 raw history 产生它的阶段**。即使接线也跑不起来。

**结论**:module_design 的"生产级完整"**对部件成立,对整体管线不成立**。解冻 = 恢复 52 测试 + 补 front-end + 补 orchestrator + 补端到端测试。比 v1 评估乐观:部件有完整测试守护。

### 4.3 辐射定标 —— 🟢 小难度(测试可恢复,验收报告可信)

**真实实现(对冻结契约完整)**:
- 多目标加权融合 `K_fused=ΣwK/Σw`(`.cpp:45-57`)、RCS 反演 `σ=K|I|²R⁴`(`InvertRcs:199`)、残差 `Δσ_dB`(`.cpp:209`)、原子化执行流水线 `ExecuteCalibrationRequests`(`.cpp:129-188`,7 种失败枚举全覆盖)。
- ~150 行真实数值,无 stub,无 TODO,C++11 兼容,可编译。
- **精确匹配冻结契约** `radiometric_calibration.md`(image-response 因子,非全雷达方程)。

**v1 错误已纠正**:
- v1 称"验收报告 5/5 通过但测试未提交,数字无法复现"—— **错误**。`sar_radiometric_calibration_test.cpp`(181 行,6 TEST)在 `ce4aa2d9^` 完整存在,是被冻结提交删除的。验收数字**可信**。

**结论**:6 项中**最容易解冻**之一。恢复 6 测试 + 去 DEPRECATED 横幅 + 加回 manifest 即可达到"对冻结契约生产可用"。若要绝对 RCS 定标(全雷达方程 `Pt·G²·λ²·σ/((4π)³R⁴|I|²)`),需额外补系统因子(契约明确后置)。

### 4.4 自适应选择 Auto (SarFocusingSelector) —— 🟢 小难度,逻辑完整+4 测试可恢复

**逻辑完整(最接近可用)**:
- 真实决策树(`RecommendFocusingAlgorithm`,`cpp:46-100`),完整覆盖 L1/L2/L3 × 3 目的矩阵,4 种推荐 + 结构化 reason/rejection 枚举 + 请求回显。
- **正确排除 CSA/OmegaK**(枚举无 kCsa/kOmegaK,value 2/3 故意空缺)。
- **纯咨询、零副作用、无回退**(最关键契约属性):不 include 任何算法头,不调用 Focus/Execute,不可用则 `kAlgorithmUnavailable` 绝不静默换。
- 自包含可编译,无悬挂符号,无 TODO。
- **4 TEST 可恢复**(`sar_focusing_selector_test.cpp`,102 行)。

**结论**:**6 项中最易解冻**。逻辑无需进一步实现,恢复测试 + 去横幅 + 一个内部调用方即可。能立即给 Phase 4 聚束/扫描提供算法路由建议。

### 4.5 CSA —— 🔴 大难度,主流程完全未实现(测试只护几何)

**仅有(半成品)**:
- `SarCsaGeometry`:计算频率轴、`D(fa)=sqrt(1-(λfa/2v)²)`、**chirp_scaling_factor α=1/D−1 算出但从不被任何操作消费**(`.cpp:80-81`)。
- `SarCsaIntermediateTruth`:逐阶段真值 oracle(FFT/multiply-phase-kernel + NRMS 对比),**只是未来 CSA 的测试预言机,不是 CSA 本身**。
- **9 TEST 可恢复**(几何 92 行 + 中间态真值 129 行),但**只覆盖几何和 oracle,不覆盖主流程**(因主流程不存在)。

**完全缺失**:chirp scaling 应用、RCMC、距离/方位压缩、SRC、残余相位校正 —— 主流程一个都没有。

**结论**:与 module_design"半成品"描述一致。解冻 = 从零写完整 CSA 聚焦(工作量大);可恢复的 9 测试只守护已有的几何/oracle 部分,主流程需新测试。RDA 已覆盖 broadside,CSA 价值需重估。

### 4.6 二阶运动补偿 —— ✅ 阶段 A 证据已采集,判定不实现

**零存在**:grep `SecondOrder`/`second_order`/`HigherOrder`/`QuadraticCompensation` 在 `src/sar/`、`include/1q/sar/` **零命中**。无符号、无脚手架、无预留接口。一阶 `SarMotionCompensation.h` 只有 `ApplyFirstOrderMotionCompensation`,无重载。

**阶段 A 失效证据矩阵已运行(2026-06-24),判定 `DO_NOT_TRIGGER_PHASE_B`**:
- 契约 `second_order_motion_compensation.md` §3 要求实现前先证明"残余相位是主要失效来源"。
- 实测在 9×9 固定 PRF 场景(与 `l3_first_order_applicability_matrix.md` 工况一致),参考点自身(二阶相位项严格为零)从 **9 m** 横向偏移起即失效(NRMS 0.27→0.67)。
- 这定量印证了 `l3_first_order_applicability_matrix.md:44`"12m 失效不应只归因于残余相位":**失效根因是非直线轨迹导致 RDA 平移不变聚焦假设崩溃**,不是补偿精度不足。
- 二阶补偿试图"把 RDA 在转弯时救回来",但参考点(零残余)都糊了,二阶项修不了根因。正确路径是转弯改用 **BP**(已就绪)。

**结论**:**不实现**。证据封存于 `docs/sar/audits/second_order_motion_compensation_phase_a_verdict.md`,测试 `sar_second_order_motion_compensation_evidence_test.cpp` 进入主链路(2 测试,295 SAR 测试全绿)。除非未来 RDA 引入轨迹自适应聚焦后仍失效,否则不重开。

## 5. 解冻优先级建议(修正后,按性价比)

> **注(2026-06-24)**:本节 🥇 优先级(辐射定标 + Auto selector)及 🥈 的 Omega-K **部件解冻**已由 `68dd8213` 执行。🥈 标注的"补 front-end+orchestrator"、⚠️ PGA 闭环、🔴 CSA、🔴 二阶补偿仍未完成。

| 优先级 | 冻结项 | 理由(修正后) |
|---|---|---|
| 🥇 **先解冻** | **辐射定标** + **自适应选择 Auto** | 两者逻辑对冻结契约完整,测试可恢复(6+4 TEST),工作量极小(恢复测试+去横幅+加回 manifest)。辐射定标验收报告现证可信。 |
| 🥈 **次解冻** | **Omega-K** | 部件质量高(含手写 SHA-256),**52 测试可恢复**守护部件;补 front-end+orchestrator+端到端测试即成聚焦路径。对聚束/扫描有价值。 |
| ⚠️ **谨慎** | **PGA 闭环** | 估+真值 35% 已建且活体测试,但闭环 65% 待写,且需 1-D→2-D 桥接。 |
| 🔴 **暂缓** | **CSA** | 主流程从零,9 可恢复测试只护几何/oracle。RDA 已覆盖 broadside,价值需重估。 |
| 🔴 **不实现** | **二阶运动补偿** | 阶段 A 证据矩阵已运行,判定不实现:失效根因是 RDA 转弯假设崩溃而非补偿精度,转弯改用 BP。证据封存,不重开。 |

## 6. 解冻的标准操作(基于 ce4aa2d9 的可逆性)

由于冻结是 `ce4aa2d9` 的整体动作,解冻任一冻结项的标准逆操作:

1. **恢复测试**:`git checkout ce4aa2d9^ -- tests/unit/sar_<frozen>_test.cpp`(已验证可行,blob 存在)。
2. **去 DEPRECATED 横幅**:删除源文件顶部 6 行 `【未进行设计需求,不再扩展 — DEPRECATED】` 注释。
3. **加回构建 manifest**:在 `src/sar/SarSources.cmake` 的 `SAR_ENGINE_SOURCES` 注册源文件。
4. **解冻护栏**:`check_sar_frozen_sources.cmake` 会拦截步骤 3 —— 需同步从 frozen pattern 列表移除对应项(或临时禁用该合同测试)。
5. **验证**:跑恢复的测试 + `sar_ci` + `sar_performance` + C++11 兼容门。

> 注意:步骤 4 的护栏是**有意设计的解冻门槛** —— 它确保解冻是显式、可审计的动作,
> 不会被误操作触发。这符合 `module_design.md` 孤儿章节的"恢复任一项前须先重开设计审批"。

## 7. 跨项关键发现(修正后)

1. **"完整实现非空壳"对部件成立,对整体管线/Omega-K 不成立**:Omega-K 部件级生产质量(52 测试可恢复),但管线未组装、front-end 缺失。
2. **测试可恢复性是 v1 的最大误判**:71 个 TEST 在 git 历史完整保留,解冻工作量普遍下调一档。
3. **冻结护栏设计正确且可逆**:`check_sar_frozen_sources.cmake` 是有意门槛(非缺陷),`ce4aa2d9` 的"保留代码+删测试+加横幅"三件套是可逆的整体动作。
4. **二阶运动补偿是唯一"从未存在"的项**:其余 5 项均有实质代码,其中 4 项有可恢复测试;二阶补偿零代码零测试,是真正的从零起步。**2026-06-24 更新**:阶段 A 证据矩阵运行后判定**不实现**——失效根因是 RDA 转弯聚焦假设崩溃(非补偿精度),二阶补偿无法修复根因,转弯场景改用 BP。该项从"从零起步待开发"转为"证据封存不实现"。

## 8. 给用户的直接结论(修正后)

如果目标是"解冻并尽快让某些项进入生产可用":

- **立即可解冻、工作量最小**:**辐射定标** + **自适应选择 Auto**(恢复测试即可,逻辑已完整,验收可信)。
- **解冻后接近可用**:**Omega-K**(恢复 52 测试 + 补 front-end+orchestrator)。
- **解冻后仍需大量工作**:**PGA 闭环**(闭环 65% 待写)。
- **判定不实现**:**二阶运动补偿**(阶段 A 证据已采集,失效根因是 RDA 转弯假设崩溃而非补偿精度,转弯改用 BP)。
- **解冻意义存疑**:**CSA**(主流程从零)。

**重要**:任何解冻仍需走设计审批(护栏有意设此门槛),本报告不构成授权。

## 9. v1 错误的归因

v1 报告"零测试"结论源于**只检查当前工作树,未查 git 历史**。这是审计方法的疏漏 —— 对"被
冻结"的项目,测试被删除是冻结动作的预期结果,不应据此判定"从未有测试"。本次修正已通过
`ce4aa2d9` 提交的 diffstat + 各测试 TEST() 逐文件计数 + blob 可恢复性验证,彻底纠正。

后续对"冻结/弃用"类资产的审计,必须默认核查 git 历史(deleted-files 检索),不能只看当前树。

## 10. 本评估的非目标

- 不构成任何解冻/实现的正式授权(护栏有意要求重开审批)。
- 不修改任何代码或 CMake(冻结护栏保持不动)。
- 不评估解冻后对 Phase 4/5 的影响(另议)。
- 人天估算未给出(需各项解冻契约细化后才能负责任地估)。
