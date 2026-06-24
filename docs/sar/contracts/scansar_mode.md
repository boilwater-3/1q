# SAR 扫描模式(ScanSAR)工程契约

Date: 2026-06-24
状态: **阶段 A+B+C 实现,elevation burst 建模 + 逐 burst Omega-K 聚焦编排器 + GBP 交叉核对 + 重叠区加权平均拼接 + 方位分辨率退化验证已落地(349 测试全绿,含 sar_ci/sar_performance 门)。**
实现难度: 🟠 中(聚焦引擎侧因聚束/Omega-K 已落地而大幅降低;主要工作量在 elevation 向 burst 调度与跨子带拼接)
前置契约: `spotlight_mode.md`(聚束已实现,沉淀了 Omega-K 复用 + 波束正交分离模式)、
`omega_k_focusing_orchestrator.md`(Omega-K 已实现)、`omega_k_math_reference.md` §7

> **实现记录(2026-06-24)**:阶段 A(上游建模)+ 阶段 B(逐 burst 编排)+ 阶段 C(验证)已实现。
> - 阶段 A:`ScanBurstState`(elevation burst 调度状态)+ `GenerateScanBurstSchedule`(周期性子带轮转)+
>   `GenerateScanSarTrack`(组合轨迹)。`GeneratePointTargetRawEchoWithElevationGate`(距离门控回波,
>   `enabled=false` 严格退化为条带)。14 个测试验证调度律/跨周期/单子带退化/门控/拒绝。
> - 阶段 B:`FocusScanSarOmegaK` 逐 burst 聚焦编排器,每 burst 复用 `FocusStripmapOmegaK`
>   (broadside,offset=0,等价条带聚焦)+ burst 方位拼接。**§4.3 重叠区加权平均**:
>   burst 不重叠时走简单堆叠(逐样本等价,单 burst = 条带子集不变量);burst 重叠时走统一方位轴
>   加权平均(重叠区多 burst 贡献幅度平均,最近邻映射)。9 个测试验证单子带退化/单 burst=条带子集/
>   多 burst 拼接/双子带/确定性/拒绝 + 重叠加权平均拼接。
> - 阶段 C:GBP 交叉核对(独立算法验证峰值一致)+ 双子带峰值物理验证 + **§5.2 方位分辨率退化验证**
>   (短 burst 主瓣宽度 > 长孔径主瓣,单调性验证 ρ_az 按 T_burst/T_aperture 展宽)。聚焦引擎零改动。
> - §5.6 门验证:sar_ci(Eigen 3.3.9/C++11 兼容 + 源码冻结校验 `sar_frozen_sources`)通过;
>   sar_performance(8 个性能测试)通过。
> - 零侵入:所有现有函数签名不变,条带/聚束路径完全不改。349 个 SAR 测试零回归(基线 324)。

> **本契约起草背景(2026-06-24)**:聚束(Spotlight)阶段 A+B 已落地(324 测试全绿,
> `cb7abc07`),证伪了 `phase4_difficulty_assessment.md` §3.4 的核心前提——
> "Omega-K 被冻结不能用作扫描路径"。聚束已确立三条可直接迁移的工程范式:
> 1. **Omega-K 复用**:`FocusOmegaKInternal`(`SarOmegaKFocusing.cpp:67`)对 broadside 匀速直线
>    raw history 逐块聚焦,Stolt 映射天然 squint-invariant。
> 2. **波束正交分离**:`SpotlightBeamState`(`SarSpotlightBeam.h:23`)证明"波束序列与平台轨迹
>    时间对齐、解耦"可行,聚束走方位向,ScanSAR 走 elevation 向(正交维度,同套机制)。
> 3. **零侵入**:`GeneratePointTargetRawEchoWithAntenna`(`SarEcho.h:85`)证明"新函数承载新模式、
>    旧函数签名不动"可行(`enabled=false` 严格退化)。
>
> 这些范式使 ScanSAR 的聚焦路径从 phase4 原估的"全新 SPECAN/ECSA"降级为"逐 burst 复用 Omega-K + 拼接"。

## 1. 目标

支持扫描(ScanSAR)模式:平台直线飞行时,天线在 **elevation(距离)向** 交替指向多个子带
(subswath),用各子带的时间分段(burst)合成宽测绘带覆盖,**以牺牲方位分辨率为代价换取
大幅扩展的距离向测绘带宽**。

**本契约确立关键技术决策:**
1. **聚焦路径采用逐 burst Omega-K(而非全新 SPECAN/ECSA)**。理由(经聚束实现证实):
   - ScanSAR 的每个 burst 内部仍是 **broadside 匀速直线** 几何(burst 内天线 elevation 固定,
     方位不扫),Omega-K 的 Stolt 映射天然成立——与条带/聚束共享同一 `FocusOmegaKInternal`。
   - phase4 §3.4 原估"必须 SPECAN/ECSA"的前提"Omega-K 被冻结"已被聚束契约 §8 证伪。
   - ScanSAR 的难点不在单 burst 聚焦(与条带等价),而在 **burst 调度 + 拼接**——而拼接是
     独立于聚焦算法的编排层问题。
2. **elevation burst 调度与平台轨迹正交分离**(迁移聚束 §3.1 方案 2 的解耦模式)。

**本契约不授权立即实现**:ScanSAR 需前置完成上游 elevation 向 burst 建模(当前完全空白),
实现分三阶段(§3-5)。

## 2. 背景与现状差距(经代码逐行核查)

### 2.1 ScanSAR 的物理本质

ScanSAR = 平台直线匀速运动 + **天线在 elevation 向周期性切换子带**。与条带/聚束的差异:
- 测绘带由多个子带在距离向拼接而成(子带数 N_swath,典型 2-4)→ 距离向覆盖远超单波束。
- 每个子带只被**间歇照射**(一个 burst 周期内轮流驻留各子带)→ 每个地面点的合成孔径被
  截断为 burst 长度 T_burst,**方位分辨率下降**为 ρ_az ≈ v / (Doppler_bw_per_burst)。
- 各 burst 之间**方位向有间隙**(天线去别的子带了),需 burst 间适当重叠以保证方位连续覆盖。
- raw history 在方位上是**分段不连续的**(同一子带的 burst 间有空隙,空隙期间天线在别的子带)。

> **与聚束的关键对比**:聚束是**方位向**时变波束(反向跟踪,获得更高方位分辨率);
> ScanSAR 是 **elevation 向**时变波束(切换子带,获得更宽距离测绘带)。两者波束时变机制
> 正交,可复用聚束确立的"波束序列正交分离"范式,只是波束状态字段从 azimuth 换成 elevation/range。

### 2.2 聚焦引擎侧:Omega-K 几乎就绪(关键优势,与聚束同源)

| 部件 | ScanSAR 改动需求 | 证据 |
|---|---|---|
| Stolt 几何(`SarOmegaKGeometry`) | **无需改动** | 单 burst 内 broadside 几何,K_z=sqrt(K_r²−K_x²) 成立。与聚束共享 squint-invariant 优势(`SarOmegaKFocusing.cpp:67` 的 `FocusOmegaKInternal`) |
| front-end(`SarOmegaKSpectrumFrontEnd`) | **无需改动** | 2D FFT + H_bulk 对每 burst 独立执行,与条带/聚束等价(`SarOmegaKSpectrumFrontEnd.cpp`) |
| 编排器 | **新增 burst 包装** | 每个独立 burst 调 `FocusOmegaKInternal(config, 0.0, burst_raw, output)`(broadside,offset=0),复用条带路径 |
| 拼接层 | **新增**(编排层,非算法层) | 把 N_swath × N_burst 个聚焦 burst 拼成宽测绘带图像 |

**关键洞察:单 burst 聚焦 = 条带聚焦(broadside)**。ScanSAR 不引入任何 squint(burst 内
天线 elevation 固定、方位不扫),故聚焦引擎**零改动**——与聚束"复用 Omega-K"同源,且更
简单(聚束还需方位 offset 偏移,ScanSAR 连 offset 都是 0)。工作量几乎全在**上游 burst
调度 + 下游拼接**。

### 2.3 共享上游:elevation 向 burst 建模的缺口(主要工作量)

聚束沉淀了**方位向**时变波束(`SpotlightBeamState`),但 ScanSAR 需要 **elevation 向**
时变波束——这是当前完全空白的维度:

| 缺口 | 现状 | 证据 |
|---|---|---|
| **elevation 波束指向状态** | 仅方位向 `SpotlightBeamState`(boresight_azimuth_rad),无 elevation/子带字段 | `SarSpotlightBeam.h:23-26`;`AntennaParams` 有 `beam_width_range_rad` 但无 elevation steering(`SarAntenna.h:19`) |
| **burst 调度序列** | 零。无 burst 周期/驻留时间/子带切换概念 | `src/sar` grep `burst`/`subswath`/`dwell` 零命中 |
| **回波 elevation 门控** | 回波幅度只有 `√RCS/R²`(可选加方位 sinc²),**无 elevation 距离门/子带选择** | `SarEcho.cpp:51`(`GeneratePointTargetRawEcho`);`AntennaModulationConfig` 只有方位波束(`SarEcho.h:71-75`) |
| **分段 raw history** | raw history 假设方位连续;无"burst 间空隙"表达 | `RawEchoConfig`(`SarEcho.h:44-48`)无 burst 标记 |

> 注:聚束的 `AntennaModulationConfig`(`SarEcho.h:71`)只携带 `SpotlightBeamState`(方位),
> 不携带 elevation 信息。ScanSAR 需要一个**正交的 elevation 门控机制**(目标是否落在当前
> burst 的距离窗口内),这是聚束未覆盖的新维度。

### 2.4 聚束沉淀的可复用资产

ScanSAR 可直接迁移聚束的工程模式(非零基础):

| 资产 | 来源 | ScanSAR 复用方式 |
|---|---|---|
| 波束序列正交分离 | 聚束 §3.1 方案 2(`SpotlightBeamState` 与 `PlatformPulseState` 解耦) | elevation 波束序列同样与平台脉冲序列时间对齐、解耦 |
| 零侵入回波生成 | `GeneratePointTargetRawEchoWithAntenna`(`SarEcho.h:85`,`enabled` 门) | 新建 `GeneratePointTargetRawEchoWithElevationGate`,`enabled=false` 退化为条带 |
| 共享聚焦入口 | `FocusOmegaKInternal(config, 0.0, raw, out)`(`SarOmegaKFocusing.cpp:67`) | 每 burst 直接调,broadside offset=0 |
| broadside 退化不变量 | 聚束 §4.2 | 单子带 + 单 burst 退化为条带 |

### 2.5 配置/Gate 现状

- `SarPolicyConfig.h:36` 的 `max_allowed_squint_angle_deg` 仍是**保留字段**,运行时无 gate。
- `SarMissionConfig.h` 无 subswath/burst/scan 字段(grep 零命中)。
- `SarHardwareConfig.h` 的 `antenna_width_m`(距离向)存在但无 elevation steering 表达。
- `SarFocusingSelector`(`SarFocusingSelector.h`)无 ScanSAR/burst 路径枚举。

## 3. 阶段 A:上游 elevation burst 建模契约冻结(前置,必做)

聚焦引擎就绪(逐 burst = 条带),但 raw history 的 burst 物理正确性依赖上游。阶段 A 冻结
elevation 向 burst 建模约定。

### 3.1 冻结:elevation 波束指向与平台轨迹的正交分离

**架构决策**(迁移聚束 §3.1 方案 2):平台轨迹与 elevation 波束调度**正交分离**。
- 平台轨迹:沿用 `GenerateStraightStripmapTrack`(直线匀速,ScanSAR 平台本身直线)。
- elevation 波束调度:新建**elevation 波束序列** `ScanBurstState`,与平台脉冲序列时间对齐,
  在回波生成处汇合。**不侵入 `PlatformPulseState`**(聚束已验证此路径可行)。

**冻结数据结构(草案,实现时定稿)**:

```cpp
// src/sar/geometry/SarScanBurst.h(新建,镜像 SarSpotlightBeam.h)
struct ScanBurstState {
  double time_s{0.0};
  std::uint32_t subswath_index{0U};    // 当前 burst 指向的子带编号
  double near_range_m{0.0};            // 该子带近端斜距(elevation 门控用)
  double far_range_m{0.0};             // 该子带远端斜距(elevation 门控用)
  bool illuminated{false};             // 该脉冲是否照射该子带(burst 驻留/间隙标记)
};
```

> 与聚束 `SpotlightBeamState`(只一个 boresight_azimuth_rad)对比:ScanSAR 的 burst 状态
> 需要子带索引 + 距离窗口(burst 的本质是距离分段),而非单一角度。

### 3.2 冻结:burst 调度律(周期性子带轮转)

ScanSAR 的天线 elevation 在 N_swath 个子带间周期性轮转。冻结为:

```
对每个 burst 周期 T_cycle(= N_swath × T_dwell):
  for i in [0, N_swath):
    在 [t_start + i·T_dwell, t_start + (i+1)·T_dwell) 区间,天线指向子带 i
    该区间内 illuminated[subswath_i] = true,其余 false
```

每个子带的 **burst 长度** T_burst = T_dwell(驻留时间)。burst 间方位重叠(保证连续覆盖)
通过相邻周期中同一子带的 burst 在方位上有重叠时间实现。

**关键不变量**:同一子带相邻 burst 中心时刻的方位间距 Δx_burst = v · T_cycle 必须满足
Δx_burst ≤ burst 合成孔径对应的方位覆盖,否则方位出现缝隙(门控拒绝)。

### 3.3 冻结:回波 elevation 门控(子带距离窗口选择)

`GeneratePointTargetRawEcho` 扩展为带 elevation 门控的版本(迁移聚束
`GeneratePointTargetRawEchoWithAntenna` 的零侵入模式):

```cpp
// src/sar/echo/SarEcho.h(扩签名,旧函数不动)
struct ElevationGateConfig {
  geometry::ScanBurstState burst_state{};  // 该脉冲的 burst 调度
  bool enabled{false};                      // enabled=false 退化为无门控(条带兼容)
};

bool GeneratePointTargetRawEchoWithElevationGate(
    const RawEchoConfig& config,
    const ElevationGateConfig& gate_config,
    const geometry::PlatformPulseState& platform,
    const std::vector<PointTarget>& targets,
    const signal::ComplexVector& transmit_waveform,
    RawEchoResult* result);
```

门控逻辑:`enabled=true` 时,仅当目标斜距 `R ∈ [near_range_m, far_range_m]` 且
`burst_state.illuminated` 为真时,该目标才贡献回波;否则跳过(幅度置 0)。这自然实现
"天线此刻只看这个子带"的物理效果。

> 与聚束天线调制的对比:聚束用**方位 sinc² 软加权**(目标离 boresight 越远幅度越小);
> ScanSAR 用**距离硬门控**(目标在子带窗口内全贡献,窗外零贡献)。两者正交,可叠加
> (ScanSAR burst 内仍可有方位方向图,但 elevation 用硬窗口近似更清晰)。

### 3.4 冻结:不建模 elevation 方向图连续形状

阶段 A 冻结:**用距离硬门控近似 elevation 方向图**(3.3)。理由:
- 当前无 elevation 连续方向图函数(`AzimuthPattern` 是方位 sinc²,`SarAntenna.cpp:25`;
  无 `ElevationPattern`)。
- burst 的本质是子带切换,硬门控(矩形窗)足以表达"哪个子带被照"。
- 平坦地球近似下,矩形 elevation 窗 + 各子带 reference_range 独立聚焦足够。
- 若后续发现子带边缘有缝隙/重影,再评估 elevation sinc² 软过渡(后置,非阻塞)。

### 3.5 冻结:各子带独立 reference_range

每个子带有自己的近距/远距窗口(3.1 `ScanBurstState`),聚焦时该子带的 burst 用其窗口
中心斜距作为 `OmegaKConfig::reference_range_m`。这保证 Stolt bulk 参考相位对该子带最优
(与条带/聚束共享的 `reference_range_m` 字段语义一致,`SarOmegaKFocusing.h:36`)。

### 3.6 阶段 A 验收

1. burst 调度序列 + elevation 门控实现,ScanSAR raw history 物理正确(burst 分段、子带轮转)。
2. **单子带退化不变量**:N_swath=1 时,ScanSAR 调度退化为全程照射单子带,
   elevation 门控退化为全程通过 → 等价条带 raw history(逐脉冲逐目标等价)。
3. 各子带 burst 的 reference_range 独立配置。
4. burst 周期门控:T_dwell 合理范围,子带方位覆盖无缝隙(否则拒绝)。

## 4. 阶段 B:逐 burst Omega-K 聚焦编排(条件触发)

### 4.1 ScanSAR 聚焦编排器

**核心设计**:ScanSAR 聚焦 = 对每个 burst 独立调 `FocusOmegaKInternal`(broadside,offset=0)
+ 拼接。新建编排器:

```cpp
// src/sar/imaging/SarScanSarFocusing.h(新建)
struct ScanSarSubswathConfig {
  OmegaKConfig omega_k{};               // 该子带的 Omega-K 参数(reference_range 取子带中心)
  std::vector<std::size_t> burst_pulse_ranges;  // 各 burst 的脉冲索引区间 [start, end)
};

struct ScanSarFocusConfig {
  std::vector<ScanSarSubswathConfig> subswaths{};  // N_swath 个子带配置
};

struct FocusedScanSarImage {
  // N_swath 个子带,每个子带是各 burst 拼接后的连续子图像
  std::vector<signal::ComplexMatrix> subswath_images{};
  // 跨子带距离拼接后的完整宽测绘带图像(可选)
  signal::ComplexMatrix mosaicked_image{};
  std::vector<OmegaKDiagnostics> burst_diagnostics{};
  std::string failure_stage{"none"};
};

bool FocusScanSarOmegaK(const ScanSarFocusConfig& config,
                        const signal::ComplexMatrix& raw_pulse_history,
                        const std::vector<geometry::ScanBurstState>& burst_schedule,
                        FocusedScanSarImage* output);
```

**实现极简**(相对聚束编排器):每 burst 直接调
`FocusOmegaKInternal(burst_config.omega_k, 0.0, burst_raw, &burst_image)`——broadside,
offset=0,与条带完全等价。改动:
- burst 脉冲切片(从 raw history 按 `burst_pulse_ranges` 抽出每 burst 的子矩阵)。
- 跨 burst 方位拼接(同子带内 burst 按方位坐标对齐叠加——burst 重叠区加权平均)。
- 跨子带距离拼接(各子带图像按距离坐标纵向拼接)。

### 4.2 不变量

1. **单子带退化**:N_swath=1 且 burst_pulse_ranges 覆盖全程时,`FocusScanSarOmegaK` 输出 =
   `FocusStripmapOmegaK` 输出(broadside 退化,与聚束 §4.2 同构)。
2. **单 burst = 条带子集**:任一 burst 独立聚焦结果,与对该 burst 脉冲区间单独跑条带聚焦
   逐样本一致(因都走 `FocusOmegaKInternal(offset=0)`)。
3. **Stolt squint-invariant**:ScanSAR 不引入 squint(burst 内 broadside),Stolt 映射成立。
4. **方位连续性**:同子带相邻 burst 拼接后方位无突变(重叠区加权平滑过渡)。
5. **距离拼接无重叠伪影**:跨子带距离拼接在子带边界连续(各子带 reference_range 独立,
   拼接前对齐到统一距离轴)。
6. 确定性 + 无 NaN/Inf。

### 4.3 burst 方位拼接策略(冻结)

同子带内 N_burst 个 burst 各自聚焦后,需拼成连续方位图像。冻结策略:
- 各 burst 聚焦后方位坐标已知(由该 burst 脉冲区间对应平台位置推出)。
- burst 间**重叠区**:重叠脉冲聚焦后能量叠加(非相干叠加——取幅度平均,避免相位不连续)。
- 非重叠区:各 burst 贡献各自方位段。
- **不实现 SPECAN 的 deramp 相干拼接**(那是 SPECAN 路径的需求;Omega-K 路径每 burst 独立
  完整聚焦,无需 deramp)。

> 这是 Omega-K 路径相对 SPECAN 的简化优势:SPECAN 需要 burst 内 deramp + burst 间相位
> 补偿,而 Omega-K 每 burst 是完整的小孔径聚焦,拼接只需幅度域对齐。

## 5. 阶段 C:验证与收尾

1. ScanSAR 双子带场景:宽测绘带覆盖 ≈ 2× 单子带(burst 调度正确)。
2. 方位分辨率验证:ρ_az(ScanSAR) ≈ ρ_az(条带) × (T_aperture/T_burst),即 burst 截断导致
   方位分辨率按 burst 占空比下降(物理预期)。
3. 单子带退化等价性(N_swath=1 退化为条带)。
4. GBP 双子带参考交叉核对(GBP 天然支持任意波束/距离窗口,作独立真值——GBP 按 burst 子集
   投影,可对 ScanSAR 各 burst 独立交叉核对)。
5. L1 条带 Session 回归不变。
6. 默认与 Eigen 3.3.9 C++11、sar_ci、sar_performance 门通过。

## 6. SRC(二次距离压缩)决策

继承聚束契约 §6 结论:**Omega-K 路径不需要独立 SRC**。ScanSAR 每 burst 走 Omega-K,Stolt
波数域精确解天然处理距离-方位耦合。SRC 是 RDA 路径的专属需求,ScanSAR 若走 RDA 路径才需,
但本契约冻结走 Omega-K 路径(§1),故不需要 SRC。

## 7. 冻结边界

- ScanSAR 聚焦走 **逐 burst Omega-K 路径**(契约确立)。不实现全新 SPECAN/ECSA 算法
  (避免 phase4 §3.4 原估的高成本;Omega-K 复用证伪了该前提)。
- 不实现干涉 ScanSAR(InSAR,Phase 5+)。
- 不实现 TOPS(Terrain Observation by Progressive Scans,ScanSAR 的方位反向扫描变体,
  后置——TOPS 需方位向 burst 内扫描,引入 squint,接近聚束+扫描混合)。
- 不引入 elevation 连续方向图(用距离硬门控近似,§3.4)。
- 不实现 burst 间相干拼接(用幅度域对齐,§4.3)。
- 不接入聚束 + ScanSAR 组合(聚束方位扫 + 扫描距离扫,模式正交但组合复杂度高,后置)。
- 阶段 A 未通过则阶段 B/C 不执行。

## 8. 实现难度评估

| 维度 | 评估 |
|---|---|
| 聚焦引擎(Omega-K) | 🟢 低(逐 burst = 条带 broadside,零改动,复用 `FocusOmegaKInternal`) |
| elevation burst 调度 | 🟠 中(新建调度律 + `ScanBurstState`,但循环逻辑直接) |
| 回波 elevation 门控 | 🟠 中(扩签名 + 距离门,迁移聚束 `WithAntenna` 零侵入模式) |
| burst 方位拼接 | 🟠 中(幅度域对齐,非相干;重叠区加权平均) |
| 跨子带距离拼接 | 🟠 中(各子带 reference_range 独立,统一距离轴对齐) |
| 配置/Gate | 🟢 低(新增 subswath/burst 字段 + 方位覆盖门) |
| SRC | 🟢 不需要(Omega-K 波数域精确解,继承聚束结论) |
| SPECAN/ECSA | 🟢 不实现(Omega-K 复用路径不需要) |
| 预计人天 | 阶段 A: 7-10;阶段 B: 5-7;阶段 C: 3-5;合计 **15-22** |
| 对比报告原估 | phase4_difficulty_assessment §3.3/§7 原估 26-40 人天(SPECAN 路径);Omega-K 复用路径降至 15-22 |

**难度降低的关键**(与聚束同源,且 ScanSAR 更简单):
1. Omega-K 已实现 + 聚束已复用验证(`FocusOmegaKInternal` 对 broadside 直接可用)。
2. ScanSAR burst 内是 **broadside 零 squint**(比聚束还简单——聚束还需方位 offset 偏移,
   ScanSAR 连 offset 都是 0)。
3. 不需要 SRC(继承聚束结论)。
4. 不需要 SPECAN 的 deramp/相位补偿(Omega-K 每 burst 完整聚焦)。
5. 聚束沉淀的波束正交分离 + 零侵入回波生成模式可直接迁移到 elevation 维度。

**ScanSAR 相对聚束的额外工作量**:burst 调度循环(聚束是连续波束,ScanSAR 是离散子带轮转)+
burst 方位拼接(聚束无需拼接,ScanSAR 需 burst 间对齐)+ 跨子带距离拼接(聚束无)。
这些是**编排层**工作,非算法层。

## 9. 非目标

- 不重开 CSA/PGA/二阶运动补偿冻结(已证据否决)。
- 不实现 ScanSAR 自聚焦(PGA 已否决)。
- 不实现滑动聚束(Sliding Spotlight,聚束与条带的混合,后置)。
- 不实现 TOPS(ScanSAR 的方位扫描变体,引入 squint,后置)。
- 不实现 InSAR(干涉,Phase 5+)。
- 不替换条带 RDA 为默认路径(ScanSAR 是独立模式)。
- 不引入 GDAL/PROJ(GeoTIFF 是独立子项)。
- 不引入 elevation 连续方向图测量(用距离硬门控近似)。

## 10. 与聚束契约的对称性总结

| 维度 | 聚束(Spotlight) | 扫描(ScanSAR) |
|---|---|---|
| 时变波束维度 | 方位向(反向跟踪场景中心) | elevation 向(周期性子带轮转) |
| 波束状态结构 | `SpotlightBeamState{boresight_azimuth_rad}` | `ScanBurstState{subswath_index, near/far_range}` |
| 回波调制 | 方位 sinc² 软加权(`WithAntenna`) | 距离硬门控(`WithElevationGate`) |
| 聚焦路径 | Omega-K(方位 offset 偏移) | 逐 burst Omega-K(broadside,offset=0) |
| squint | 非零(天线方位扫,Stolt 吸收) | 零(burst 内 broadside) |
| 分辨率影响 | 方位分辨率**提升**(更长合成孔径) | 方位分辨率**下降**(burst 截断合成孔径) |
| 拼接需求 | 无(单连续孔径) | burst 方位拼接 + 子带距离拼接 |
| broadside 退化 | beam_steering_rate=0 → 条带 | N_swath=1 → 条带 |
| Omega-K 改动 | 方位原点偏移参数 | 零(broadside offset=0) |
| SRC | 不需要(Stolt 精确解) | 不需要(同源) |

两者波束时变维度正交(方位 vs elevation),聚焦都复用 Omega-K(broadside 不变式),
工程范式高度对称——这正是聚束先行使 ScanSAR 难度大幅下降的结构性原因。
