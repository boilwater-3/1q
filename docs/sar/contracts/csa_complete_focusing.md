# SAR CSA 完整聚焦工程契约

Date: 2026-06-24
状态: 草案(待审批)
实现难度: 🔴 大(主流程完全从零;冻结护栏仍拦截;Phase 4 squint/SRC 才是其真正价值区)
前置契约: `csa_math_reference.md`(§6 明令未批准完整 CSA 相位函数)

## 1. 目标

实现完整的 Chirp Scaling Algorithm(CSA)聚焦入口 `FocusStripmapCsa`,作为 RDA 的替代
聚焦路径。CSA 用 chirp-scaling 技巧把距离走动校正(RCMC)变成纯相位乘法(不插值),
数学上更适合宽波束/大斜视角场景。

**本契约不授权立即实现**:实现前必须先完成 §3 阶段 A(价值证据),因为:
- CSA 的现有部件(几何 + 中间态 oracle)主流程完全不存在,alpha scaling 因子算出但从未被消费。
- broadside 下 CSA 相对 RDA 的增量**存疑**(RDA 退化主因是二次相位曲率近似,CSA 在纯 broadside 未必显著更优)。
- **Omega-K(已实现)本身就是聚束/宽波束友好算法**,CSA 与其竞争——阶段 A 必须回答"为何 CSA 而非扩展 Omega-K"。

## 2. 背景与冻结依据

### 2.1 现有部件(半成品,仍被护栏冻结)

| 部件 | 能力 | 状态 |
|---|---|---|
| `SarCsaGeometry` | 频率轴 + D(fa) + **α(fa) scaling 因子** | ⚠️ 算出但**从不被消费** |
| `SarCsaIntermediateTruth` | 逐阶段 FFT/multiply-phase-kernel/NRMS oracle | ⚠️ 只是测试预言机,**无 CSA 特定相位公式** |

**主流程完全缺失**:chirp scaling 应用、RCMC(纯乘法版)、距离/方位压缩、SRC(二次距离压缩)、
残余相位校正——一个都没有。现有 9 个可恢复测试只护几何 + oracle。

**冻结护栏**(`check_sar_frozen_sources.cmake`):当前唯一拦截面 `SarCsa`。`SarSources.cmake`
中无任何 SarCsa 条目(既不在 manifest,又被 pattern 拦截,双保险)。

### 2.2 CSA 数学参考 §6 的冻结

`csa_math_reference.md` §6(与 `omega_k_math_reference.md` §7 同构措辞)明令以下"尚未批准":

- 第一 chirp-scaling 相位函数。
- 距离压缩、SRC 与 RCMC 合并相位函数。
- 方位压缩与残余相位校正。
- 参考距离变化与距离相关 Doppler rate。

> 不得依据高层六步流程自行补写公式并宣称完整 CSA。

§8 要求三个完整相位函数必须配经审计的独立参考真值(离线脚本/固定相位样本/GBP 对比)。

### 2.3 RDA 当前能力边界(已量化)

RDA(`FocusStripmapRda`)仅支持 broadside 条带,7 步管线:
- RCMC 用 linear/sinc 插值(非 scaling)。
- 距离走动公式是 broadside 二阶近似:`ΔR = λ²R_ref·f_a²/(8v²)`(`SarRda.cpp:230`)。
- **零 squint 处理**(多普勒中心硬编码为 0;`ComputeDopplerParams` 对 RDA 是死代码)。
- **无 SRC**(二次距离压缩)。

现有审计已量化 RDA 退化:
- 大孔径(33 脉冲)NRMS 升至 0.676(`rda_diagnostic.md`)。
- RCMC 插值(linear vs sinc)差异 <0.0003 → **RCMC 插值非主因**(`azimuth_sampling.md`)。

### 2.4 CSA vs Omega-K 的竞争论证(阶段 A 必答)

`phase4_difficulty_assessment.md:73` 指出"冻结的 Omega-K 是聚束友好的算法"。Omega-K 已
于本阶段实现(`FocusStripmapOmegaK`,`4c1301fc`),有 18 个解冻源 + Stolt 几何。

**阶段 A 必须回答**:在宽波束/squint 场景下,为何选择 CSA(主流程从零,9 测试只护几何)
而非扩展 Omega-K(已有完整 Stolt 链路)?

候选论证方向(需阶段 A 证据支撑):
- CSA 的距离相关 Doppler rate 在**中等斜视 + 大测绘带**比 Omega-K 的 Stolt 插值更高效。
- CSA 全程频域(无插值),数值稳定性优于 Omega-K 的 Stolt 非均匀插值。
- 但若证据不足以区分,则应**优先扩展 Omega-K 而非新建 CSA**。

## 3. 阶段 A:价值证据矩阵(前置,必做)

### 3.1 矩阵设计

由于仓库**完全没有 squint/宽波束测试场景**,阶段 A 需新建。扫描:

- **斜视角 θ** ∈ {0°(broadside), 5°, 15°, 30°}(新建场景,用 `ComputeDopplerParams`
  的 squint 支持 + 平台航向偏置)。
- **方位孔径** ∈ {9, 33, 65} 脉冲。
- **目标方位偏置** norm_offset ∈ {0, 0.25, 0.5, 1.0}(复用 `ComputeTargetOffsetNonlinearPhaseResidualRad`)。

每个组合用 RDA 聚焦,与 GBP 独立参考对比,记录:
- `CompareImagesWithGlobalPhaseReference`:NRMS、相干。
- `EvaluateImageQuality`:方位/距离 3dB 宽度、ISLR。
- **非线性相位残差**:`ComputeTargetOffsetNonlinearPhaseResidualRad`(已有现成测量器)。
- **CSA 几何预测**:`SarCsaGeometry` 的 α(fa) scaling 因子在该斜视角下的量级
  (α 越大,scaling 修正越显著,CSA 增量越大)。

### 3.2 阶段 A 通过准则(触发阶段 B 的条件)

**全部满足**才允许进入阶段 B:

1. **RDA 在非 broadside 场景失败**:存在 θ > 0 或大孔径 + 大偏置组合,RDA NRMS > 0.25。
2. **失效是相位模型不足**:非线性相位残差(4π·max_residual/λ)超过 π/4,且 RCMC 插值
   不是主因(azimuth_sampling 已证插值差异 <0.0003)。
3. **CSA 几何预测显著**:α(fa) scaling 因子在失效场景的量级 |α| > 0.01(若 α≈0,
   CSA 退化为 RDA,无增量)。
4. **CSA 优于扩展 Omega-K**(§2.4 竞争论证):在失效场景下,CSA 的理论修正
   (距离相关 Doppler rate + 纯频域 SRC)相比 Omega-K Stolt 插值有明确优势(效率/稳定性/精度),
   或 Omega-K 在该场景同样不足。

**若阶段 A 不通过**(RDA 在所有场景足够,或 Omega-K 已覆盖),则 CSA **不实现**。

### 3.3 阶段 A 的诚实预期

基于现有审计,broadside 下 CSA 相对 RDA 增量存疑。CSA 的真正价值区是 squint + SRC,
而这属于 Phase 4。**阶段 A 很可能判定 CSA 在当前阶段不值得实现**——这是合法且重要的结论
(与二阶运动补偿同构),应如实记录,不强行触发阶段 B。

## 4. 阶段 B:完整 CSA 实现(条件触发,难度最高)

### 4.1 完整六步流程

```
raw history
  → [1] 距离 FFT → 距离频域
  → [2] 方位 FFT → 2D 频域(range-Doppler)
  → [3] 第一 chirp-scaling 相位乘(D1,用 α(fa))—— RCM 变成距离依赖的 chirp 尺度变化
  → [4] 距离 IFFT → 距离压缩 + SRC + RCMC 合并相位乘(D2) → 距离压缩频域
  → [5] 距离 IFFT → 距离压缩时域
  → [6] 方位压缩 + 残余相位校正(D3) → 方位 IFFT → 聚焦图像
```

### 4.2 新增内容(全部从零)

- **三个相位函数 D1/D2/D3**(数学参考 §6 冻结,需独立真值先行)。
- `FocusStripmapCsa` 编排器(签名沿用 `FocusStripmapRda` 模式)。
- SRC(二次距离压缩)——RDA 完全没有。
- squint 支持(多普勒中心估计 + squint 修正方位匹配滤波)。
- 独立参考真值(离线脚本/GBP 对比,数学参考 §8)。
- 端到端测试(现有 9 测试只护几何 + oracle,主流程需新测试)。

### 4.3 不变量

1. broadside(θ=0)退化:CSA 退化为 RDA 等价(α=0 时 D1 为单位)。
2. 确定性:相同输入相同输出。
3. 不动 RDA 默认路径(CSA 是独立入口)。
4. 不引入 NaN/Inf。

## 5. 验收门

### 5.1 阶段 A 验收

1. 矩阵覆盖 squint × 孔径 × 偏置,确定性可复现。
2. RDA 在非 broadside 场景的失效可量化(NRMS / 非线性相位残差)。
3. CSA α(fa) 在失效场景的量级可计算。
4. CSA vs Omega-K 竞争论证有数据支撑。
5. 默认与 Eigen 3.3.9 C++11 门通过。

### 5.2 阶段 B 验收(若触发)

1. broadside(θ=0)下 CSA 与 RDA 图像一致(NRMS < 0.1)。
2. 非零 squint 下 CSA 优于 RDA(NRMS 更低、3dB 更窄)。
3. 独立参考真值(GBP 或离线脚本)验收通过。
4. 冻结护栏更新(从 `SarCsa` pattern 收窄或移除)。
5. L1/L2 现有 Session 回归不变。
6. 默认与 Eigen 3.3.9 C++11、sar_ci、sar_performance 门通过。

## 6. 冻结边界

- 阶段 A 未通过则阶段 B 永不执行(最高优先级约束)。
- 不接入 public Session / Selector(数学参考 §9 + `internal_focusing_selector.md:98`)。
- 不实现 ScanSAR 多子带(Phase 4,依赖 Spotlight 先解决 squint)。
- 不实现时变 PRF / 运动补偿 / 自聚焦联动。
- 不修改 RDA / GBP / BP / Omega-K 的签名或行为。
- 不修改冻结护栏(除非阶段 B 触发且实现完成)。

## 7. 实现难度评估

| 维度 | 评估 |
|---|---|
| 算法复杂度 | 高(六步流程 + 三个相位函数 + SRC + squint) |
| 部件就绪度 | 极低(几何半成品,主流程 0%,9 测试只护几何) |
| 前置工作量 | 大(阶段 A:新建 squint 场景 + 竞争论证) |
| 风险 | 高(三个相位函数需独立真值,squint 全仓库零基础) |
| 预计人天 | 阶段 A: 4-5;阶段 B: 10-15(六步 + SRC + squint + 真值);合计 14-20 |
| 关键不确定性 | **阶段 A 很可能判定不值得实现**(broadside 增量存疑,squint 属 Phase 4,Omega-K 已覆盖聚束) |

## 8. 非目标

- 不重开 PGA、二阶运动补偿冻结。
- 不实现聚束/扫描 CSA(Phase 4)。
- 不替换 RDA 为默认路径。
- 不扩展 Omega-K(若阶段 A 判定 Omega-K 更优,则 CSA 永久搁置)。
