# SAR 聚束模式(Spotlight)工程契约

Date: 2026-06-24
状态: 草案(待审批)
实现难度: 🔴 高(但聚焦引擎侧因 Omega-K 已实现而大幅降低;主要工作量在上游天线/回波/轨迹建模)
前置契约: `omega_k_focusing_orchestrator.md`(Omega-K 已实现)、`omega_k_math_reference.md` §7

## 1. 目标

支持聚束(Spotlight)模式:平台直线飞行时,天线持续反向跟踪同一场景区域,获得远超条带
波束宽度的合成孔径,从而实现超高方位分辨率。

**本契约确立关键技术决策:聚焦路径采用扩展 Omega-K(而非大改 RDA)。** 理由(经代码
探查证实):
- Omega-K 的 Stolt 映射**天然 squint-invariant**:`K_z = sqrt(K_r²−K_x²)` 对任意非零 K_x
  (squint/聚束的多普勒中心)均成立,几何/front-end **几乎零改动**。
- RDA 硬编码 broadside(调频率 `2v²/(λR0)`、RCMC `∝fa²` 零中心对称),扩展聚束需多处
  RDA 专属手术(线性 RCMC、多普勒中心估计、cos³ 修正),且 RDA 的二次近似在大孔径下失效。

**本契约不授权立即实现**:聚束需要前置完成上游建模(天线时变指向 + 回波方位调制 +
轨迹天线字段),这些是当前完全空白的共享基础设施。实现分三阶段(§3-5)。

## 2. 背景与现状差距(经代码逐行核查)

### 2.1 聚束的物理本质

聚束 = 平台直线匀速运动 + **天线时变波束指向**(反向跟踪场景中心)。与条带的差异:
- 合成孔径由天线转动积累角决定(非平台飞过的波束宽度)→ 方位分辨率更高。
- raw history 带**额外天线方位调制**(目标被更长时间照射,波束包络加窗)。
- 多普勒带宽更宽(更高分辨率),要求 PRF ≥ 聚束多普勒带宽。

### 2.2 聚焦引擎侧:Omega-K 几乎就绪(关键优势)

| 部件 | 聚束改动需求 | 证据 |
|---|---|---|
| Stolt 几何(`SarOmegaKGeometry`) | **无需改动** | K_x 轴(unshifted f_a)+ K_z=sqrt(K_r²−K_x²) 天然 squint-invariant(`SarOmegaKGeometry.cpp:80,98-117`) |
| front-end(`SarOmegaKSpectrumFrontEnd`) | **算法无需改动** | 2D FFT + H_bulk=exp(+j·R_ref·K_z) 与多普勒中心无关(`SarOmegaKSpectrumFrontEnd.cpp:74-104`) |
| 编排器(`FocusStripmapOmegaK`) | 小改:方位坐标原点参数化;契约边界放宽 | `MakeAzimuthCoordinates`(`SarOmegaKFocusing.cpp:39-49`)原点可参数化 |

**Omega-K 不需要多普勒中心估计**——Stolt 映射吸收了 squint。这是相对 RDA 的结构性优势
(RDA 必须显式估计多普勒中心并修正 RCMC/方位压缩)。

### 2.3 共享上游:三个完全空白的缺口(主要工作量)

| 缺口 | 现状 | 证据 |
|---|---|---|
| **天线时变波束指向** | 固定单标量 `boresight_azimuth_rad`,无时变接口,且**全模块未接入** | `SarAntenna.h:21`;`AntennaGain`/`AzimuthPattern` 在 src/sar 内零调用 |
| **回波天线方位调制** | 幅度只有 `√RCS/R²`,**完全无方向图加权** | `SarEcho.cpp:49-50`;`GeneratePointTargetRawEcho` 签名无天线参数 |
| **轨迹天线指向字段** | `PlatformPulseState` 只有 position+velocity,**无天线指向** | `SarGeometry.h:28-35` |

> 注:天线方向图缺失在条带模式下影响小(默认用脉冲窗口当孔径);但聚束**必须**显式建模,
> 否则"聚束"退化为"加长条带"——不物理。

### 2.4 配置/Gate 现状

- `SarPolicyConfig.h:36` 的 `max_allowed_squint_angle_deg{5.0}` 是**保留字段**,运行时
  无 gate(`SarRuntimeConfigValidation.cpp:36-88` 不检查 squint)。
- `include/1q/sar/` 无 spotlight/beam_steering/aperture_angle 字段(grep 零命中)。

## 3. 阶段 A:上游建模契约冻结(前置,必做)

聚焦引擎就绪,但 raw history 物理正确性依赖上游。阶段 A 冻结上游建模约定。

### 3.1 冻结:轨迹与天线指向的正交分离模型

**架构决策**:平台轨迹与天线指向**正交分离**。
- 平台轨迹:沿用 `GenerateStraightStripmapTrack`(直线匀速,聚束平台本身就是直线)。
- 天线指向:新建**波束指向序列**,与平台轨迹正交,在回波生成处汇合。

**`PlatformPulseState` 扩展方案(二选一,阶段 A 冻结)**:
- 方案 1(侵入式):给 `PlatformPulseState` 加 `boresight_azimuth_rad` 字段。单脉冲状态
  自洽,但侵入现有 33 个引用 `PlatformPulseState` 的文件。
- 方案 2(解耦,推荐):新建并行的 `AntennaPulseState` 序列(`spotlight_beam_states`),
  与平台脉冲序列时间对齐。解耦,不动现有代码。

### 3.2 冻结:聚束波束指向律

聚束的波束指向随慢时间反向跟踪场景中心。冻结为:

```
boresight_azimuth(t) = atan2(scene_center_x − platform_x(t),
                              scene_center_y − platform_y(t))
```

即每脉冲计算"平台→场景中心"的方位角。这自动实现聚束的持续照射。
扫描速率 `beam_steering_rate` 由平台速度和场景中心距离决定(非独立配置)。

### 3.3 冻结:回波天线方位调制

`GeneratePointTargetRawEcho` 的幅度项增加方向图加权:

```
off_boresight(t) = target_azimuth(t) − boresight_azimuth(t)
amplitude *= AzimuthPattern(antenna, λ, off_boresight(t))   // 双程取平方或√
```

`AzimuthPattern`(已有 `SarAntenna.cpp:25-34`,sinc²)可复用,只需接入回波生成。
聚束的"加长照射"边界由方向图自然实现(目标离开波束主瓣时幅度衰减)。

### 3.4 冻结:elevation 方向图(距离向)

当前只有方位 sinc²,无 elevation pattern。聚束虽主要方位扫描,但大斜视角下距离向
方向图影响 SRC。阶段 A 冻结是否需要 elevation pattern(平坦地球近似下可能不需要)。

### 3.5 阶段 A 验收

1. 波束指向律 + 回波天线调制实现,聚束 raw history 物理正确(含天线包络)。
2. broadside 退化:beam_steering_rate=0 时,聚束回波退化为条带回波(不变量)。
3. 聚束合成孔径时间由 steering 角推出(非条带公式 `R0·θ_bw/v`)。
4. 配置新增 spotlight 字段 + squint gate 激活。

## 4. 阶段 B:Omega-K 聚束聚焦编排(条件触发)

### 4.1 聚束 Omega-K 编排器

扩展 `FocusStripmapOmegaK` → `FocusSpotlightOmegaK`(或参数化模式枚举):

```cpp
bool FocusSpotlightOmegaK(const SpotlightOmegaKConfig& config,
                          const signal::ComplexMatrix& raw_pulse_history,
                          const std::vector<AntennaPulseState>& beam_states,
                          FocusedOmegaKImage* output);
```

改动极小(相对条带编排器):
- 方位坐标原点用场景中心(参数化 `MakeAzimuthCoordinates`)。
- front-end/几何/Stolt 链路**不变**(squint-invariant)。
- 新增聚束多普勒带宽诊断(PRF 充裕性检查)。

### 4.2 不变量

1. **broadside 退化**:beam_steering_rate=0 时,聚束编排器输出 = 条带编排器输出。
2. **Stolt squint-invariant**:非零多普勒中心不破坏 Stolt 映射(几何已保证)。
3. **PRF 充裕**:聚束多普勒带宽 < PRF,否则拒绝(方位混叠)。
4. 确定性 + 无 NaN/Inf。

## 5. 阶段 C:验证与收尾

1. 聚束单点目标:方位分辨率显著优于条带(由 steering 角决定)。
2. GBP 聚束参考交叉核对(GBP 天然支持任意波束,作独立真值)。
3. broadside 退化等价性。
4. L1 条带 Session 回归不变。
5. 默认与 Eigen 3.3.9 C++11、sar_ci、sar_performance 门通过。

## 6. SRC(二次距离压缩)决策

`phase4_difficulty_assessment.md:71` 和 `csa_complete_focusing.md` 多处提到聚束需要 SRC。
**本契约的判断:Omega-K 路径不需要独立 SRC**——Omega-K 的 Stolt 映射是波数域精确解,
天然处理距离-方位耦合(SRC 要修的那个耦合),不像 RDA 的距离压缩在时域近似。
**SRC 是 RDA 聚束路径的专属需求,Omega-K 路径不需要。** 这进一步降低 Omega-K 聚束难度。

若阶段 B 验证发现 Omega-K 在大斜视/大孔径下仍有距离离散焦,则再评估 SRC。

## 7. 冻结边界

- 聚束聚焦走 **Omega-K 路径**(契约确立)。不扩展 RDA 做 squint(避免 RDA 专属手术)。
- 不实现 ScanSAR(依赖聚束,Phase 4 后续)。
- 不实现滑动聚束(Sliding Spotlight,聚束与条带的混合,后置)。
- 不引入真实天线方向图测量(用 sinc² 解析模型)。
- 阶段 A 未通过则阶段 B/C 不执行。

## 8. 实现难度评估

| 维度 | 评估 |
|---|---|
| 聚焦引擎(Omega-K) | 🟢 低(Stolt squint-invariant,几乎零改动) |
| 天线模型(时变指向) | 🟠 中(新建波束指向律 + 接入,但公式简单) |
| 回波生成(天线调制) | 🟠 中(扩签名 + 加权项,但 `AzimuthPattern` 可复用) |
| 轨迹状态(天线字段) | 🟠 中(方案 2 解耦,侵入小) |
| 配置/Gate | 🟢 低(新增字段 + 激活 gate) |
| SRC | 🟢 不需要(Omega-K 波数域精确解) |
| 预计人天 | 阶段 A: 6-9;阶段 B: 3-5;阶段 C: 3-4;合计 **12-18** |
| 对比报告原估 | phase4_difficulty_assessment §3.3 原估 22-34 人天(RDA 路径);Omega-K 路径降至 12-18 |

**难度降低的关键**:Omega-K 已实现(`4c1301fc`)+ Stolt squint-invariant + 不需要 SRC。
phase4_difficulty_assessment.md:73 "Omega-K 被冻结不能用作聚束路径"的前提**已失效**。

## 9. 非目标

- 不重开 CSA/PGA/二阶运动补偿冻结(已证据否决)。
- 不实现聚束自聚焦(PGA 已否决,MoCo 足够)。
- 不接入 ScanSAR(Phase 4 后续,依赖聚束)。
- 不替换条带 RDA 为默认路径(聚束是独立模式)。
- 不引入 GDAL/PROJ(GeoTIFF 是独立子项)。
