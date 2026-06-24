# SAR L1/L2/L3 多保真度一致性矩阵工程契约

Date: 2026-06-24
状态: **已实现。一致性矩阵 5 测试全绿,纯测试层零侵入,全量 354 SAR 测试零回归,sar_ci/sar_performance/Eigen 3.3.9 门通过。**

> **实现记录(2026-06-24)**:一致性矩阵已实现。新增 `tests/unit/sar_l2_l3_fidelity_matrix_test.cpp`,
> 5 个测试覆盖契约 §2.4 全部断言:
> 1. `L2ZeroPerturbationDegeneratesToL1` — L2 σ=0 逐样本等价 L1(零扰动→零补偿)。
> 2. `StraightSceneL1L2L3PeakConsistent` — 直线 σ=0 三路径(L1-RDA/L2-RDA/L3-BP)峰值列一致 + 能量集中。
> 3. `L2SmallPerturbationCompensationImproves` — σ_y=10 补偿后 NRMS<0.3、相干>0.95。
> 4. `L2LargePerturbationCompensationPartial` — σ_y=30 补偿仍改善但不完全(精度边界)。
> 5. `L3BpRecoversTurningScene` — 转弯(12m 横偏)BP 峰值正确 + 对比度优于 RDA(二阶补偿冻结决策证据)。
>
> **零侵入**:矩阵只调用现有自由函数(`FocusStripmapRda`/`FocusSmallSceneBp`/
> `ApplyFirstOrderMotionCompensation`/轨迹生成),不改任何生产源代码、session、配置。
> 354 个 SAR 测试零回归(基线 349)。sar_ci(含 frozen_sources 校验)、sar_performance(8 测试)、
> Eigen 3.3.9/C++11 门全通过。
前置评估: `l2_l3_coupling_value_assessment.md`(阶段 A 价值评估——组合聚焦冻结,一致性矩阵实现)
前置契约: `l2_session_integration.md`、`l3_bp_session_integration.md`(L2/L3 个体已闭环)、
`second_order_motion_compensation.md`(二阶补偿冻结依据)

> **本契约范围(据阶段 A 评估决策)**:
> - **仅实现一致性矩阵**(纯测试层,零算法/session/配置改动)。
> - **不实现** L2×L3 组合聚焦(经评估判定 BP 已逐脉冲精确,联动无算法新增价值,
>   见 `l2_l3_coupling_value_assessment.md` §3.1)。
> - **不实现** Session 保真度编排(selector 已完整,保真度本就该显式选择,§3.3)。
> - **不解** L2/L3 互斥——它是**正确的物理约束**(BP 不需补偿,L2 补偿仅对 RDA 有意义)。

## 1. 目标

构建 **L1/L2/L3 多保真度一致性矩阵**:对同一物理场景,分别经三条保真度路径聚焦,系统化对比
输出,形成"跨保真度退化可控、边界清晰"的可审计证据。

三条路径(全部个体已闭环,本契约是只读消费者):

| 路径 | 保真度 | 轨迹 | 聚焦 | 补偿 | 现状证据 |
|---|---|---|---|---|---|
| **L1** | 理想直线 | `GenerateStraightStripmapTrack` | `FocusStripmapRda` | 无 | `l2_session_integration.md` §3 |
| **L2** | 直线 + 随机扰动 | `GeneratePerturbedStripmapTrack` | `FocusStripmapRda` | `ApplyFirstOrderMotionCompensation` | `l2_session_integration.md`(已闭环) |
| **L3** | 航路点(含转弯) | `GenerateWaypointTrack` | `FocusSmallSceneBp` | 无(BP 逐脉冲精确,内含运动效应) | `l3_bp_session_integration.md`(已闭环) |

> **关键物理区分(阶段 A 评估 §2.4 确立)**:
> - L1/L2 用 **RDA**(频域,假设直线匀速),L2 需一阶补偿修正扰动。
> - L3 用 **BP**(时域逐脉冲精确,`SarGbp.cpp:83` 用实际位置精确投影),**不需补偿**——
>   平台运动的相位/包络效应已完全内含。
> - 故 L2/L3 互斥是正确设计:补偿对 BP 无意义,BP 对任意 actual 轨迹都精确。

## 2. 矩阵设计

### 2.1 矩阵维度

矩阵为 **场景类型 × 扰动档位 × 保真度路径**,每格产出聚焦图像 + 一致性指标。

```
                        L1-RDA    L2-RDA(+补偿)   L3-BP
直线场景 σ=0        [基线]     [退化不变]      [独立参考]
直线场景 σ=小        N/A       [补偿修复]      N/A
直线场景 σ=大        N/A       [补偿极限]      N/A
转弯场景            N/A       N/A(RDA失效)   [BP修复]
```

> N/A = 该格在物理上无意义或已被冻结决策排除(见 §3 不变量)。

### 2.2 场景类型(冻结)

| 场景 | 物理含义 | 轨迹构造 | 适用路径 |
|---|---|---|---|
| **直线 σ=0** | 理想匀速直线(基线) | `GenerateStraightStripmapTrack` | L1 / L2 / L3(直线 waypoint) |
| **直线 σ 小** | 直线 + 微小随机扰动(一阶补偿可修复) | `GeneratePerturbedStripmapTrack`(σ_x=2,σ_y=10,σ_z=5 m/s) | L2 |
| **直线 σ 大** | 直线 + 大随机扰动(一阶补偿接近极限) | `GeneratePerturbedStripmapTrack`(σ_y=30 m/s) | L2 |
| **转弯** | 航路点轨迹中段横向偏移(RDA 失效区) | `GenerateWaypointTrack`(末端 cross_range=12m) | L3 |

- 直线场景用同一目标同一几何,L1/L2/L3 三路径产出**可直接对比**的图像。
- 转弯场景是 RDA 失效区(二阶补偿已冻结的实测依据,见 `second_order_motion_compensation.md` §3),
  仅 L3-BP 有效——**这格本身就是二阶补偿冻结决策的正确性证据**。

### 2.3 一致性指标(冻结)

每格聚焦图像产出以下指标,跨格对比:

| 指标 | 计算 | 对比意义 |
|---|---|---|
| **峰值位置**(row, col) | `EvaluateImageQuality` | 三路径在同场景峰值应落在目标真实位置附近 |
| **方位/距离 3dB 主瓣宽度** | `EvaluateImageQuality` | 保真度退化 → 主瓣展宽 |
| **图像对比度/熵** | `EvaluateImageQuality` | 散焦 → 对比度下降、熵上升 |
| **峰值幅度** | 图像 max \|·\| | 补偿/精确投影的有效性 |

### 2.4 对比断言(冻结)

矩阵的核心断言(每条都有明确物理预期):

1. **直线 σ=0 三路径一致**:L1-RDA、L2-RDA(σ=0)、L3-BP(直线 waypoint)在直线无扰动场景下,
   峰值位置一致(落在目标真实方位×斜距)。
   - L1/L2 走 RDA 应逐窗一致(σ=0 补偿不变,L2 退化为 L1)。
   - L3-BP 是独立算法,峰值位置应与 RDA 一致(不同算法同目标)。
2. **L2 σ=0 退化不变**:L2-RDA(σ=0)输出与 L1-RDA 逐样本一致(零扰动 → 零补偿 → 等价)。
   - 这是 `l2_session_integration.md` §3 已确立的不变量,矩阵复验。
3. **L2 补偿改善**:σ 小档,补偿后 L2-RDA 的主瓣宽度/对比度 **优于** 未补偿。
4. **L2 补偿极限**:σ 大档,补偿后仍有散焦(主瓣展宽)——但优于未补偿(补偿有部分效果)。
5. **L3-BP 转弯修复**:转弯场景下 BP 峰值位置正确、主瓣集中(对比 RDA 在同场景的散焦)。
   - 这证明转弯场景应改用 BP(二阶补偿冻结决策的正确性)。

## 3. 不变量与边界

### 3.1 矩阵只读,零侵入

- 矩阵测试**只调用**现有自由函数:`FocusStripmapRda`、`FocusSmallSceneBp`、
  `ApplyFirstOrderMotionCompensation`、`GenerateStraightStripmapTrack`、
  `GeneratePerturbedStripmapTrack`、`GenerateWaypointTrack`。
- **不改任何源代码**。不改 session、不改配置、不改 validation、不改 replay。
- 这是 `l2_l3_coupling_value_assessment.md` §3.2 的核心决策:一致性矩阵是只读消费者。

### 3.2 物理互斥保持(不解锁)

矩阵**不解** L2/L3 互斥。每格的 N/A 是物理事实,非配置限制:
- 直线 σ≠0 仅 L2(RDA 需补偿);L3-BP 对直线扰动也精确,但**无补偿对比意义**
  (BP 不补偿,扰动直接进 actual,BP 照样精确——对比的不是"补偿效果")。
- 转弯仅 L3-BP;L2-RDA 在转弯失效(平移不变假设崩溃,§2.5),**不应进入矩阵对比**
  (会产出散焦图像,但那是 RDA 已知失效,非保真度对比)。

> 注:若需展示"转弯场景 RDA 失效"本身作为证据,那是 `second_order_motion_compensation.md`
> 阶段 A 已封存的证据,不重复进本矩阵。

### 3.3 BP 尺寸门

L3-BP 路径受 `kMaxApprovedDimension=128` 门约束(`SarGbp.cpp:15`)。
矩阵的 BP 格用 ≤ 128×128 场景(沿用参考场景支撑库的 9-33 脉冲 × 64 样本尺度)。

### 3.4 参考场景确定性

矩阵用 `tests/support/sar_reference_scene.h` 的确定性场景(seed 可复现)。
L2 扰动用固定 seed(`l2_random_seed`),保证矩阵可 replay、可审计。

## 4. 测试组织(实现指南,非冻结)

新增测试文件 `tests/unit/sar_l2_l3_fidelity_matrix_test.cpp`,镜像现有矩阵测试的组织模式:

```
参考模板:sar_reference_scenario_matrix_test.cpp(GBP/RDA 裁窗对比矩阵)
         sar_second_order_motion_compensation_evidence_test.cpp(L2 档位扫描 + 转弯)
```

测试套件命名:`SarL2L3FidelityMatrixTest`。

测试用例(对应 §2.4 断言):
1. `StraightSceneL1L2L3PeakConsistent` — 直线 σ=0 三路径峰值一致。
2. `L2ZeroPerturbationDegeneratesToL1` — L2 σ=0 逐样本等价 L1。
3. `L2SmallPerturbationCompensationImproves` — σ 小补偿改善主瓣/对比度。
4. `L2LargePerturbationCompensationPartial` — σ 大补偿有部分效果但仍散焦。
5. `L3BpRecoversTurningScene` — 转弯场景 BP 峰值正确、主瓣集中。

## 5. 验收门

1. 矩阵全绿(5 个测试用例通过)。
2. 三路径个体零回归(L1/L2/L3 各自现有测试不受影响——矩阵只读)。
3. 默认与 Eigen 3.3.9/C++11、sar_ci、sar_performance 门通过(§5.6 沿用聚束/扫描契约)。
4. 矩阵产出可解释:直线场景退化可控(L1→L2 补偿修复),转弯场景边界清晰(仅 BP 有效)。

## 6. 非目标(据阶段 A 评估)

- **不实现** L2×L3 组合聚焦(BP 已精确,无算法新增价值,`l2_l3_coupling_value_assessment.md` §3.1)。
- **不实现** Session 保真度编排 / 接 SarFocusingSelector(§3.3,保真度本就该显式选择)。
- **不解** L2/L3 互斥(物理正确约束)。
- 不重开二阶运动补偿冻结(转弯场景证据已封存)。
- 不改变 BP 128×128 门。
- 不引入聚束/扫描/多视/GeoTIFF(独立子项)。
- 不改任何生产源代码。

## 7. 实现难度评估

| 维度 | 评估 |
|---|---|
| 算法改动 | 🟢 零(只读消费者) |
| session/配置改动 | 🟢 零 |
| 测试编写 | 🟢 低(镜像现有矩阵模板,5 个用例) |
| 预计人天 | **2-3**(纯测试,据阶段 A 评估 §6) |
| 对比 phase4 原估 | phase4 §7 原估 L2/L3 联动 8-13 人天(含组合场景 + Session 编排);阶段 A 评估把范围收窄到一致性矩阵,降至 2-3 人天 |

**难度极低的关键**:L1/L2/L3 三条路径的执行器(RDA/BP/补偿/轨迹生成)全部已闭环且有独立测试,
矩阵只是"在同一场景下把三者各跑一遍 + 对比输出"。复用 `sar_reference_scene.h` 的确定性场景构造,
复用 `sar_reference_scenario_matrix_test.cpp` 的裁窗对比模式。

## 8. 与阶段 A 评估的一致性

本契约严格遵循 `l2_l3_coupling_value_assessment.md` §4 决策:
- 方案 β(一致性矩阵)→ §2 本矩阵设计。
- 方案 α(组合聚焦)冻结 → §6 非目标。
- 方案 γ(Session 编排)冻结 → §6 非目标。
- L2/L3 互斥保持 → §3.2。

无任何超出阶段 A 决策范围的内容。
