# SAR PGA 闭环工程契约

Date: 2026-06-24
状态: **阶段 A 完成,阶段 B 否决(MoCo 已完全修复直线场景散焦,PGA 无必要)。详见
`docs/sar/audits/pga_autofocus_closure_phase_a_verdict.md`。**
实现难度: 🟠 中(估+真值 35% 已建;缺口在闭环集成 + 1-D→2-D 桥接)
前置契约: `pga_phase_gradient_estimator.md`、`pga_support_gradient_truth.md`、`pga_gradient_truth_comparison.md`、`autofocus_phase_truth.md`

## 1. 目标

在已测试的 PGA 部件链(4 部件 + 4 活体测试,1e-12 精度)基础上,补齐闭环核心,形成
完整自聚焦能力:`Autofocus` 编排器(梯度估计 → 积分 → unwrap → applyCorrection → 迭代)。

**本契约不授权立即实现闭环**:实现前必须先完成 §3 阶段 A(散焦需求证据),因为:
- 二阶运动补偿阶段 A 已证明 L3 强转弯失效主因是轨迹假设崩溃,非运动误差——
  PGA 修的是运动误差,对转弯无效。
- 需先证明**直线场景下确实存在运动误差导致的散焦**,且 PGA(而非 MoCo)是正确修复路径。

## 2. 背景与冻结依据

### 2.1 部件链现状(均解冻、纳入构建、有测试)

| 部件 | 能力 | 状态 |
|---|---|---|
| `SarPgaPhaseGradientEstimator` | 相邻样本共轭乘积 arg() 得 wrapped 梯度,消费 support_mask | ✅ 真实 |
| `SarPgaSupportGradientTruth` | 幅度阈值选 support + 注入相位算真值梯度 | ✅ 真实 |
| `SarPgaGradientTruthComparison` | wrapped 误差 RMS/max + 通过/失败判定 | ✅ 真实 |
| `SarAutofocusPhaseTruth` | 多项式相位注入(常数/线性/二次/三次)+ 最小二乘分离不可观测分量 | ✅ 真实 |

**关键约束:全部 4 部件只处理 1-D `ComplexVector`(方位剖面),从不触碰生产 2-D 图像**。
闭环缺失(grep 确认零存在):梯度积分、unwrap(只有 Wrap 无逆)、applyCorrection、
迭代+停止准则、统一 `Autofocus` 编排器。

### 2.2 证据模板已就绪

`sar_motion_compensation_test.cpp::ImprovesL2RdaAgainstIdealReference` 已建立完整的
"扰动 → 散焦 → 补偿修复"证据模式:
- `GeneratePerturbedStripmapTrack`(直线 + 高斯速度抖动,`SarGeometry.h:85`)产生扰动轨迹。
- `FocusStripmapRda` 对理想/未补偿/补偿三种 raw history 成像。
- `CompareImagesWithGlobalPhaseReference` + `EvaluateImageQuality` 量化散焦(NRMS↑、
  相干↓、熵↑、对比度↓、3dB 宽度↑)。

**这正是 PGA 阶段 A 证据矩阵的模板**——但它测的是"MoCo 修复",PGA 需证明"PGA 修复"。

### 2.3 PGA vs MoCo 的定位区分

- **MoCo(一阶运动补偿)**:用**已知理想轨迹 vs 实际轨迹**的几何差,逐脉冲补偿。
  依赖轨迹测量(INS/导航),是**前馈**的。
- **PGA**:**不依赖任何外部轨迹信息**,纯粹从图像本身估计相位误差,是**反馈**的。

两者互补:MoCo 处理"可测量的轨迹误差",PGA 处理"MoCo 残留的 + 无法测量的相位误差"。
阶段 A 必须证明存在"MoCo 补偿后仍残留的散焦",才能证明 PGA 的必要性。

## 3. 阶段 A:散焦需求证据矩阵(前置,必做)

### 3.1 矩阵设计

在 broadside 直线条带场景(非 L3 转弯),扫描:

- **轨迹抖动强度**:`GeneratePerturbedStripmapTrack` 的 `velocity_error_stddev_y_mps`
  ∈ {0, 5, 10, 20, 30} m/s。
- **MoCo 处理**:理想轨迹成像 / 扰动未补偿 / 扰动 + MoCo 补偿,三种。
- **测量指标**(对每种组合):
  - `CompareImagesWithGlobalPhaseReference`:NRMS、相干相关(相对理想参考)。
  - `EvaluateImageQuality`:方位 3dB 宽度、熵、对比度、ISLR。
  - **方位剖面相位残差**:用 `ComputeTargetOffsetNonlinearPhaseResidualRad`
    (`sar_reference_scenario_matrix_test.cpp:135`)或等价方法,量化 MoCo 后的残留相位误差。

### 3.2 阶段 A 通过准则(触发阶段 B 的条件)

**全部满足**才允许进入阶段 B:

1. **存在散焦**:MoCo 补偿后,至少一个抖动档位 NRMS > 0.25 或 相干 < 0.97(即 MoCo 未完全修复)。
2. **散焦是相位主导**:MoCo 补偿后的相位残差(4π·max_residual/λ 量级)对应的相位误差
   超过 π/4(PGA 的可观测阈值),且非幅度噪声主导(SNR 足够高时散焦仍存在)。
3. **PGA 可观测**:用 `SarAutofocusPhaseTruth` 注入对应量级的二次相位误差,
   确认 `SarPgaPhaseGradientEstimator` 能从合成剖面恢复该误差(wrapped RMS < tolerance)。

**若阶段 A 不通过**(MoCo 已完全修复,无残留散焦),则 PGA 闭环**不实现**,改走
"MoCo 足够"路径——这与二阶运动补偿的判定逻辑同构。

## 4. 阶段 B:闭环实现(条件触发)

### 4.1 闭环数据流

```
聚焦图像(2-D ComplexMatrix)
  → [选 support] 逐距离列取方位剖面 + 幅度阈值选 support_mask(复用 SupportGradientTruth 逻辑)
  → [估计梯度] EstimatePgaPhaseGradient(每列) → wrapped_gradient_rad[列][N-1]
  → [积分+unwrap] 梯度累积求和 + 解缠绕 → 相位误差函数 φ_error(azimuth)
  → [applyCorrection] 对图像逐行乘 exp(-j·φ_error)(方位相位校正)
  → [评估] EvaluateImageQuality / CompareImagesWithGlobalPhaseReference
  → [迭代] 若改善 < 阈值则停止,否则回到"估计梯度"
```

### 4.2 新增部件

| 部件 | 职责 | 输入→输出 |
|---|---|---|
| `SarPgaPhaseIntegration` | wrapped 梯度 → 解缠绕相位误差 | `ComplexVector gradient` → `ComplexVector phase_error` |
| `SarPgaPhaseCorrection` | 相位误差 → 校正后图像 | `ComplexMatrix image` + `ComplexVector phase_error` → `ComplexMatrix corrected` |
| `Autofocus` 编排器 | 串联上述 + 迭代停止 | `ComplexMatrix image` → `ComplexMatrix focused` + 诊断 |

### 4.3 不变量

1. **零扰动退化**:无相位误差时,PGA 输出 = 输入(不引入失真)。
2. **不可观测分量保持**:常数 + 线性相位不被校正(它们是全局相位 + 位移,非散焦)。
3. **确定性**:相同输入相同输出。
4. **不改变聚焦算法**:PGA 是后处理,不改 RDA/GBP/BP/Omega-K 的聚焦逻辑。

## 5. 验收门

### 5.1 阶段 A 验收

1. 矩阵覆盖 §3.1 全部组合,确定性可复现。
2. MoCo 补偿后的残留散焦可量化(NRMS / 相位残差 / 熵)。
3. PGA 估计器对合成相位误差的可观测性已确认。
4. 默认与 Eigen 3.3.9 C++11 门通过。

### 5.2 阶段 B 验收(若触发)

1. 零扰动严格退化为恒等(逐样本一致)。
2. 合成相位误差(二次/三次)经 PGA 闭环后,相位残差 RMS 下降 > 50%。
3. 扰动轨迹图像经 PGA 后,方位 3dB 宽度收窄、熵下降、对比度上升。
4. 不通过全局相位重参考掩盖空间变化误差。
5. L1/L2 现有 Session 回归不变。
6. 默认与 Eigen 3.3.9 C++11、sar_ci、sar_performance 门通过。

## 6. 冻结边界

- 仅 broadside 直线条带后处理。不支持 L3 转弯(二阶补偿阶段 A 已证明转弯失效非运动误差)。
- 不接入 public Session(内部后处理入口)。
- 不实现 MapDrift / ContrastOptimization(其它自聚焦方法,后置)。
- 不依赖 INS / IMU / 外部轨迹测量(纯图像域)。
- 不改变聚焦算法(RDA/GBP/BP/Omega-K)的签名或行为。
- 阶段 A 未通过则阶段 B 永不执行。

## 7. 实现难度评估

| 维度 | 评估 |
|---|---|
| 算法复杂度 | 中(积分+unwrap 是经典算法,但 unwrap 有歧义需处理) |
| 部件就绪度 | 中(4 部件已建 + 测试,但闭环 0% + 1-D→2-D 桥接) |
| 前置工作量 | 中(阶段 A:复用 MoCo 测试模板 + 散焦量化) |
| 风险 | 中(unwrap 歧义 + 迭代收敛性) |
| 预计人天 | 阶段 A: 3-4;阶段 B: 5-7(闭环 + unwrap + 迭代);合计 8-11 |
| 关键不确定性 | MoCo 是否已完全修复直线场景散焦——若已修复,阶段 B 不触发 |

## 8. 非目标

- 不重开 CSA、二阶运动补偿冻结。
- 不实现斜视/聚束自聚焦(Phase 4)。
- 不接入 public Session / Selector / schema / trace / replay。
- 不替换 MoCo(PGA 是 MoCo 的补充,非替代)。
