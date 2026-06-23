# SAR Omega-K 聚焦编排工程契约

Date: 2026-06-24
状态: 草案(待审批)
实现难度: 🟠 中(部件全就绪 + 真值链就绪;缺口在 front-end 谱生成、参考相位公式冻结、编排接线)
前置契约: `omega_k_math_reference.md`(§7 明令未批准完整聚焦)

## 1. 目标

在已解冻、已测试、生产级的 Omega-K 部件链(14 源、52 测试、含手写 SHA-256 真值链)基础上,
补齐三个缺口,形成完整的 `FocusStripmapOmegaK` 聚焦入口:

1. **front-end**:从 raw baseband history 产生 2D 波数谱 `source_spectrum`(目前无人产生,
   `ExecuteOmegaKExplicitGridReduction` 直接要求它作为输入却无来源)。
2. **参考相位公式冻结**:bulk 压缩参考函数 `H_ref` 与距离残余相位 `range_phase_radians`
   的数学约定(数学参考 §7 明令"不得依据高层四步流程自行补写公式并宣称完整")。
3. **编排接线**:`FocusStripmapOmegaK`,按确定性顺序调用全部部件,产出 `FocusedSarImage`。

**本契约不授权直接写编排器**:实现前必须先完成 §3 阶段 A(参考相位冻结 + 真值链验证),
因为数学参考 §7 明令完整聚焦的参考相位需经审计后方可实施。

## 2. 背景与冻结依据

### 2.1 部件链现状(均已解冻、纳入构建、有测试守护)

按数据流顺序,完整链路如下(每步均有独立执行器 + 诊断 + 拒绝枚举):

| 阶段 | 执行器 | 输入 | 输出 | 状态 |
|---|---|---|---|---|
| 0. front-end | **缺失** | raw history | `source_spectrum`(2D 谱) | ❌ 待实现 |
| 1. Stolt 几何 | `EvaluateOmegaKStoltGeometry` | 物理参数 | 频率轴/波数/Stolt 查询 | ✅ |
| 2. 共同支持 | `DiagnoseOmegaKCommonStoltSupport` | 几何诊断 | 公共有效窗口 | ✅ |
| 3. 网格收缩+Stolt 插值 | `ExecuteOmegaKExplicitGridReduction` | 谱+几何+支持 | `reduced_spectrum` | ✅ |
| 4. 相对延迟变换 | `ExecuteOmegaKRelativeDelayTransform` | 收缩谱 | `relative_delay_domain` | ✅ |
| 5. 参考映射 | `ExecuteOmegaKReferenceMapping` | 延迟域+物理 | `referenced_intermediate` | ✅ |
| 6. 参考相位补偿 | `ExecuteOmegaKReferencePhaseCompensation` | 中间态+相位向量 | `compensated_intermediate` | ⚠️ 施加器就绪,**相位向量来源缺失** |
| 7. 方位逆变换 | `ExecuteOmegaKAzimuthInverseTransform` | 补偿中间态 | `numerical_image_candidate` | ✅ |
| 8. 真值验收 | `OrchestrateOmegaKTruthEvaluation` | 候选+真值 | 点目标验收 | ✅ |

### 2.2 数学参考 §7 的冻结

`omega_k_math_reference.md` §7 明令以下内容"尚未批准":

- 参考函数相位与参考距离使用方式。
- bulk azimuth compression 和 residual phase。
- Stolt 插值核(部件已实现,但相位约定未冻结)。
- 二维 IFFT 后输出相位参考和幅度标定。

> 不得依据高层四步流程自行补写公式并宣称完整 Omega-K。

本契约 §3 即为解冻这些内容的审批载体——冻结公式后,§7 的禁令解除。

## 3. 阶段 A:参考相位冻结与真值链验证(前置,必做)

### 3.1 冻结:信号模型

L1 匀速直线 broadside 条带模式。点目标位于斜距 R_0,载频 f_c。raw baseband history 经
2D FFT 后,在双程距离波数 K_r 与方位波数 K_x 域(几何部件已算出)的信号模型冻结为:

$$
s(K_r, K_x; R_0) \;\propto\; \exp\!\bigl(-j \, R_0 \sqrt{K_r^2 - K_x^2}\bigr) \;=\; \exp(-j \, R_0 \, K_z)
$$

其中 `K_z(K_r, K_x) = sqrt(K_r² − K_x²)`(几何部件 §4 已冻结),K_r、K_x 由几何部件产生。
该模型是后续参考函数的唯一物理依据。

### 3.2 冻结:bulk 参考函数(方位相关,front-end 内施加)

将信号 bulk 压缩到参考距离 R_ref 的参考函数冻结为:

$$
H_{bulk}(K_r, K_x) \;=\; \exp\!\bigl(+j \, R_{ref} \, K_z(K_r, K_x)\bigr)
$$

施加后:`s · H_bulk = exp(−j (R_0 − R_ref) K_z)`。参考距离处(R_0 = R_ref)相位恒为零
(退化性不变量)。

**front-end 定义**:front-end = raw baseband history 的 2D FFT(`FftRows` 后 `FftCols`,
均为 forward),再逐元素乘 `H_bulk`。输出即 `source_spectrum`,送入网格收缩阶段。
不单独施加 matched filter——距离压缩被吸收进 `H_bulk` 的 K_r 依赖。

### 3.3 冻结:距离残余相位(方位无关,阶段 6 施加)

经 Stolt 插值(网格收缩)、相对延迟变换、参考映射后,参考距离处应聚焦为零延迟峰值。
距离残余相位 `range_phase_radians[col]` 冻结为参考映射所引入的 FFT 符号/归一化约定修正:

$$
\phi_{range}(r) \;=\; \text{sign}_{map} \cdot R_{ref} \cdot \bigl(K_{z,target}(r) - K_{r,DC}\bigr)
$$

其中 `sign_map` 由 `OmegaKReferencePhaseSign` 枚举冻结(Positive=+1, Negative=−1),
`K_{r,DC} = 4π f_c / c`(f_a=0 时的 K_r),`K_{z,target}(r)` 由几何部件的目标 K_z 网格给出。
该修正仅修 FFT 约定引入的常数距离相位,不引入新的物理项。

> 注:若真值链验证(§3.4)发现 `sign_map` 需调整,以真值链验收结果为准——冻结的是
> "距离残余 = sign_map × R_ref × (K_z − K_r,DC)"这一**形式**,具体 sign 由真值锁定。

### 3.4 真值链验证(强制)

阶段 A 必须:用冻结的信号模型 + H_bulk + 距离残余相位,对**单个点目标**跑完整链路
(阶段 0→7),产出 `numerical_image_candidate`,经现有真值链
(`OrchestrateOmegaKTruthEvaluation` + `EvaluateOmegaKPointTargetCandidate`)验收:

- 峰值落在目标斜距/方位的预期像素内(容差由 `OmegaKPointTargetTolerances` 给定)。
- 参考距离处 R_0 = R_ref 时,峰值相位 ≈ 0(退化性不变量)。
- 真值独立生成(`independently_generated = true`)、在共同支持窗口内。

真值数据来源:可复现脚本生成的固定 2D 频谱中间结果(满足数学参考 §9 的独立参考要求),
经 SHA-256 真值摄取链(`SarOmegaKTruthPayloadDigest`/`Ingestion`/`Manifest`)验证完整性。

### 3.5 阶段 A 通过准则

全部满足才允许进入阶段 B:

1. 单点目标链路跑通,真值链验收 `kPassed`。
2. 参考距离处峰值相位 ≈ 0(退化性,|φ| < 0.1 rad)。
3. `f_a = 0` 时 `K_z = K_r`(数学参考 §8 验收 2,几何部件已保证)在端到端链路成立。
4. front-end 谱不含 NaN/Inf,输入不变且重复计算确定(数学参考 §8 验收 6)。

## 4. 阶段 B:编排器实现(条件触发)

### 4.1 编排器签名

```cpp
bool FocusStripmapOmegaK(const OmegaKConfig& config,
                         const signal::ComplexMatrix& raw_pulse_history,
                         FocusedSarImage* output);
```

`OmegaKConfig` 沿用 `OmegaKGeometryConfig` 的物理参数(sample_rate/prf/carrier/velocity/
reference_range),补充 `bool enable_truth_validation`(默认 false,仅测试用)。
输出 `FocusedSarImage` 与 RDA/GBP 的输出类型一致(便于 Selector 与质量评估复用)。

### 4.2 编排数据流(确定性顺序)

```
raw history
  → [front-end] 2D FFT + H_bulk  →  source_spectrum
  → [stage 1] EvaluateOmegaKStoltGeometry(config)  →  geometry diagnostics
  → [stage 2] DiagnoseOmegaKCommonStoltSupport(geometry)  →  common support
  → [stage 3] ExecuteOmegaKExplicitGridReduction(spectrum, geometry, support)
       → reduced_spectrum
  → [stage 4] ExecuteOmegaKRelativeDelayTransform(reduced_spectrum)
       → relative_delay_domain
  → [stage 5] ExecuteOmegaKReferenceMapping(delay_domain, physical_meta, sign)
       → referenced_intermediate
  → [stage 6] ExecuteOmegaKReferencePhaseCompensation(intermediate, range_phase)
       → compensated_intermediate
  → [stage 7] ExecuteOmegaKAzimuthInverseTransform(compensated_intermediate)
       → numerical_image_candidate  →  FocusedSarImage
  → (可选) [stage 8] OrchestrateOmegaKTruthEvaluation(candidate, truth)
```

任一阶段 `kRejected` 则编排器整体返回 false 并记录拒绝原因(不静默继续)。

### 4.3 front-end 实现要点

- `FftRows(raw_history, false, &range_spectrum)`(forward range FFT)。
- `FftCols(range_spectrum, false, &two_d_spectrum)`(forward azimuth FFT)。
- 用 geometry 阶段的 `K_z` 向量(wavenumbers)逐元素构造 `H_bulk`。
- 越界/色散无效点(geometry 的 `invalid_dispersion_point_count`)计数并拒绝完整处理。

### 4.4 输出诊断

汇总各阶段诊断为 `OmegaKDiagnostics`:

- front-end:谱尺寸、无效点计数。
- geometry:Stolt shift max、越界查询计数。
- common_support:公共有效列比、丢弃列数。
- truth(若启用):点目标验收状态 + PSISR/ISLR/相位误差。

## 5. 不变量

1. **退化性**:R_0 = R_ref 时,峰值相位 ≈ 0(bulk 与残余相位互消)。
2. **f_a=0 对称**:`K_z = K_r`,Stolt shift 为零(数学参考 §8 验收 2)。
3. **无 NaN/Inf**:任一阶段产生 NaN/Inf 则整体拒绝。
4. **确定性**:相同输入产生相同输出(部件已保证,编排器不引入随机性)。
5. **不静默降级**:任一阶段拒绝则整体 false,不部分输出。
6. **不动 RDA 默认路径**:Omega-K 是独立入口,不改变 `FocusStripmapRda` 行为。

## 6. 验收门

### 6.1 阶段 A 验收

1. 单点目标链路跑通,真值链 `kPassed`,峰值在容差内。
2. 参考距离处峰值相位 ≈ 0(|φ| < 0.1 rad)。
3. front-end 谱确定可复现,无 NaN/Inf。
4. 默认与 Eigen 3.3.9 C++11 门通过。

### 6.2 阶段 B 验收

1. `FocusStripmapOmegaK` 对单点目标产出与 GBP/RDA 参考一致(NRMS < 0.25, 相干 > 0.97,
   `CompareImagesWithGlobalPhaseReference`)。
2. broadside 中心目标聚焦位置与几何预期一致(距离/方位误差 < 容差)。
3. 真值链验收(若启用)`kPassed`,PSLR/ISLR 在容差内。
4. 与 RDA 在相同输入下图像一致(Ω-K 与 RDA 在 broadside 理论等价,交叉核对)。
5. 编排器任一阶段拒绝时返回 false 且不输出部分图像。
6. L1/L2 现有 Session 回归不变。
7. 默认与 Eigen 3.3.9 C++11、sar_ci、sar_performance 门通过。

## 7. 冻结边界

- 仅 L1 匀速直线 broadside 条带(数学参考 §2)。不支持 L2/L3、时变 PRF、斜视、聚束、
  运动补偿、自聚焦(数学参考 §7/§10)。
- 不接入 public Session(内部聚焦入口,沿用 RDA 边界)。Selector 不新增 kOmegaK 选项
  (Phase 4 再议)。
- 不修改任何现有部件的签名或行为(front-end 与编排器只**调用**部件)。
- 不修改 RDA/GBP/BP 聚焦路径。
- Stolt 插值核固定为现有线性插值部件(不引入 sinc/高阶核)。
- 阶段 A 未通过则阶段 B 永不执行。

## 8. 实现难度评估

| 维度 | 评估 |
|---|---|
| 算法复杂度 | 低-中(front-end 是标准 2D FFT + 相位乘;编排是顺序调用) |
| 部件就绪度 | 高(14 源全就绪,52 测试守护,真值链含 SHA-256) |
| 前置工作量 | 中(阶段 A:冻结公式 + 真值链验证单点目标) |
| 风险 | 低(部件已验证,真值链已建;风险在相位 sign 约定,由真值链锁定) |
| 预计人天 | 阶段 A: 3-4;阶段 B: 3-5;合计 6-9 |
| 关键不确定性 | 距离残余相位 sign_map 由真值链锁定而非理论推导(§3.3) |

## 9. 非目标

- 不重开 CSA、二阶运动补偿冻结。
- 不实现斜视/聚束/扫描 Omega-K(Phase 4)。
- 不接入 Selector / public Session / schema / trace / replay(沿用 RDA 边界)。
- 不替换 RDA 为默认路径(Omega-K 是并行独立入口,broadside 下两者等价)。
- 不引入时域 back-projection 风格的 Ω-K 变体。
