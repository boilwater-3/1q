---
Status: draft
Date: 2026-08-17
Review-Baseline: `feature/remote-identification-radar-phase1` @ `f7371aef`
Authority: 合同指标符合性审计报告；为 sbirs_sensor 模块合同对齐工程提供逐项判定
  证据与分阶段实施依据。不得替代 `docs/sbirs_sensor/design.md`/`boundaries.md`；
  若本文与库实现冲突，以库为准。本审计只做判定与规划，不修改任何代码。
---

# SBIRS 传感器模块合同指标符合性审计（六项）

## 0. 定位与结论

审计范围：`include/1q/sbirs_sensor/`、`src/sbirs_sensor/`、`docs/sbirs_sensor/`、
`tests/{unit,contract,integration,replay}/sbirs_sensor/`，并与
`include/1q/airborne_radar/`、`src/airborne_radar/` 的安装矩阵/指向逻辑对照。

审计方式：逐项对照源码与模块文档（design.md / algorithms.md / boundaries.md /
data-flow.md），关键结论附 `文件:行号` 证据；"零命中"结论来自全模块 grep。

六项指标总结论：

| # | 合同指标 | 判定 | 一句话结论 |
|---|---------|------|-----------|
| 1 | 同机载雷达的传感器安装矩阵与指向参数逻辑 | ❌ 缺失 | SBIRS 无安装矩阵概念，指向直接定义在 ECI 极坐标参考系，与 AR 的"姿态+安装角+扫描中心"合成链完全不同 |
| 2 | 周期输入是否需要卫星速度、是否影响计算 | ❌ 缺失 | 无卫星速度输入字段；视线角速度隐含卫星静止，低估动态滞后/cue 外推/EKF R 阵，属实际计算偏差 |
| 3 | 简易大气传输模型与气象参数影响 | ✅ 满足 | 经验查表气象衰减 A_total → τ_eff 进入 SNR 链路；但 τ 为标量常数，不随 λ/d 变化 |
| 4 | 输出当前时刻最大探测距离 | ❌ 缺失 | 全模块无该输出；物理上可由 SNR 门限反解，输出链路未建 |
| 5 | 红外测角误差与安装矩阵误差（误差系数） | ⚠️ 部分 | 测角误差链完整（5 类误差+系数）；安装矩阵误差无（因第 1 项无安装矩阵），仅有语义不同的姿态误差项 |
| 6 | 大幅面扫描 / 宽视场扫描探测 / 窄视场跟踪探测 | ⚠️ 部分 | WFOV 扫描探测与 NFOV 跟踪探测完整；无"大幅面"（俯仰向二维/大画幅）扫描模式，WFOV 为固定俯仰的方位一维环扫 |

三项完全缺失（#1、#2、#4）+ 两项部分缺失（#5、#6），构成一个分阶段的对齐工程；
#1 与 #5 强耦合（安装矩阵是安装矩阵误差的前提），实施顺序见 §8。

## 1. 指标一：同机载雷达的安装矩阵与指向参数逻辑 — ❌ 不具备

**机载雷达参照实现**：

- `ArOrientationConfig`（`include/1q/airborne_radar/config/ArOrientationConfig.h:93-114`）
  定义完整安装指向配置：`mount_angles_deg`（Body→Radar 安装欧拉角，:102）、
  `scan_center_deg`、机械/电子扫描限位、扫描起始象限/推进顺序、三种波束稳定方式
  （随体/惯性/对地，:80-84）、指令态波束宽度。
- 合成关系（:89-91）：
  `actual_beam_pointing_base = platform_attitude + mount_angles_deg + scan_center_deg`，
  每周期由平台姿态输入驱动（`src/airborne_radar/session/ArExternalInputAdapter.cpp:28-57`
  的 `ComposeAttitudeDeg(platform_attitude, mount_angles)`）。

**SBIRS 现状**：

- 全模块 grep `mount` / `installation` / `安装` 仅命中 CMakeLists 注释一行，无任何安装矩阵类型或参数。
- 周期输入无卫星姿态，仅有 `has_satellite_position` + `satellite_position_ecef_m`
  （`include/1q/sbirs_sensor/session/SbirsCycleInput.h:26-27`）。
- 指向直接定义在 **ECI 极坐标参考系**（非卫星局部地平系）：
  `scan_start_az_deg`/`scan_center_el_deg`/`scan_span_deg`/`scan_rate_deg_per_sec`
  （`include/1q/sbirs_sensor/config/SbirsMissionConfig.h:34-38`）；方位/俯仰由视线向量
  在 ECI 分量上直接计算（`src/sbirs_sensor/foundation/SbirsGeometry.h:24-44`）。
- 姿态仅以**误差/扰动项**形式存在（`attitude_sigma_deg`、
  `SbirsPointingDisturbanceConfig`），不参与真实指向合成（详见 §5）。

**判定**：SBIRS 把卫星当作无姿态质点，光轴参考系与机体/安装链路完全缺失。
若合同要求"同款安装矩阵与指向参数逻辑"，需要新建
配置类型 + 姿态输入 + 坐标合成链，并贯通 WFOV 门控与 NFOV ATP（见 §8 阶段 2）。

## 2. 指标二：卫星速度输入 — ❌ 无输入字段，且确实影响计算

**输入现状**：

- `SbirsCycleInput`（`SbirsCycleInput.h:21-29`）只有卫星位置、UTC 儒略日、目标场景；
  **无卫星速度字段**。
- 目标速度有输入（`SbirsSceneTypes.h:39-40`，`velocity_ecef_m_per_s` +
  `has_velocity_ecef_m_per_s`），文档明确用途：cue 延迟外推与动态滞后误差。
- ECI 变换（`src/sbirs_sensor/pipeline/SbirsEciScene.cpp:13-36`、
  `docs/sbirs_sensor/algorithms.md:221-227`）：目标位置/速度旋 ECI 且速度含地球自转
  输运项 `v_ECI = R3(θ)·v_ECEF + ω_e × r`；**卫星只有位置参与旋转，无速度可言**。

**影响分析（回答"是否会影响计算"——会）**：

- `src/sbirs_sensor/pipeline/SbirsPipeline.cpp:631-634`：视线角速度按
  `ComputeRelativeAngularRateDegPerSec(los, target.velocity_eci_m_per_s)` 计算，
  即相对速度 = 目标速度、**卫星隐含静止**。
- 该角速度下游消费三处：
  1. **动态滞后误差** `Δθ_lag = ω/(2π·f_det)`
     （`src/sbirs_sensor/foundation/SbirsErrorModel.h:94-99`）；
  2. **WFOV→NFOV cue 延迟外推**（`SbirsCuePredictor`，algorithms.md:95-101）；
  3. **EKF R 阵**角速度相关量测噪声（algorithms.md:288）。
- LEO 卫星约 7.5 km/s，观测慢速空中目标时真实视线角速度主要由卫星运动贡献；
  当前实现系统性低估上述三处结果。**结论：卫星速度不是可有可无的冗余输入，
  缺失导致实际计算偏差**；若合同要求周期输入卫星速度，属实质缺口。

## 3. 指标三：简易大气传输模型与气象参数 — ✅ 具备（注意 τ 为标量）

**气象参数输入**（`include/1q/sbirs_sensor/config/SbirsEnvironmentConfig.h:25-36`）：

天气类型（晴/云/雨/雾枚举）、海浪等级（低/中/高）、环境温度、相对湿度、能见度、
基础大气透过率，以及两个可选气象交互项系数（湿度×能见度、雨×湿度，默认 0 关闭）。

**衰减模型**（`src/sbirs_sensor/environment/SbirsEnvironmentModel.cpp`）：

```
A_total = 0.25·A_weather + 0.15·A_sea + 0.25·A_humidity + 0.25·A_visibility
          − 0.10·A_temp_relief + Σ k_j·A_p·A_q        （夹紧 [0,1]）
τ_eff   = base_atmospheric_transmittance × (1 − A_total)
```

其中天气/海况为查表常数（云 0.05 / 雨 0.15 / 雾 0.20；海况 0.05/0.10/0.15），
湿度/能见度/温度为分段线性经验项。

**进入物理链路**：τ_eff 作为 `path_transmittance` 进入接收功率
`P = I_t·A_ap·τ_opt·τ_atm·η/d²`（`SbirsRadiometry.h:15-28`）直至 WFOV/NFOV SNR
门判决；SNR 门失败还有 `kAttenuationLimited` 归因
（`SbirsOutputTypes.h:134`）。

**注意点**：`base_atmospheric_transmittance` 注释写作 τ(λ,d)，实现是**标量常数**，
不随波长、路径长度/仰角变化。作为"简易模型"满足合同字面要求；若合同隐含
透过率随观测几何变化，需按 §8 阶段 5 精细化。

## 4. 指标四：当前时刻最大探测距离输出 — ❌ 不存在

- 全模块（含 docs）grep `max_detection_range` / `maximum_detection` /
  `detection_range` / `max_detect` **零命中**。
- 现有输出：逐检测记录仅方位/俯仰/IR SNR/观测阶段/detected 布尔
  （`SbirsOutputTypes.h:33-40`）；周期级输出仅扫描方位
  （`SbirsCycleResult.h`）；`estimated_range_m` 是内部 cue/诊断量且文档明确
  "不代表真实被动红外测距能力"（`SbirsOutputTypes.h:72-74`）。
- 距离门控 `min_range_m`/`max_range_m`（`SbirsMissionConfig.h:39-40`）是**静态配置门**，
  与 SNR 无关。

**可行性**：SNR 链路为 `SNR = I_t·A_ap·τ_opt·τ_eff·η·t_int/(NEP·d²)`，对检测门限
反解即得当前时刻最大探测距离：

```
d_max(t) = sqrt( I_t · A_ap · τ_opt · τ_eff(t) · η · t_int / (NEP · SNR_th) )
```

τ_eff(t) 随周期气象快照变化、I_t 逐目标不同，故该量是"当前时刻 × 逐目标"的
导出量，输出层设计见 §8 阶段 1。**当前实现完全没有该输出链路，若合同要求为
输出量，属明确缺口。**

## 5. 指标五：红外测角误差与安装矩阵误差 — ⚠️ 测角误差有，安装矩阵误差无

**红外测角误差：✅ 完整**（`src/sbirs_sensor/foundation/SbirsErrorModel.h`）：

5 类物理误差，配置系数在 `SbirsPolicyConfig.h:31-38`（`SbirsErrorModelConfig`）：

| 误差项 | 系数 | 性质 |
|--------|------|------|
| 卫星轨道误差 | `orbit_sigma_deg`（默认 0） | 高斯 1-σ，种子可注入（Box-Muller，replay 可复现） |
| 卫星姿态误差 | `attitude_sigma_deg`（默认 0.01°） | 高斯 1-σ，与轨道/视场项 RSS 合成 |
| 探测器视场（像元/畸变）误差 | `fov_sigma_deg`（默认 0） | 高斯 1-σ |
| 大气折射误差 | 确定性公式 `1.5e-6/(d·cosβ)` | 随距离/仰角 |
| 动态滞后误差 | 确定性公式 `ω/(2π·f_det)`，`detector_bandwidth_hz` | 随角速度（见 §2 卫星速度影响） |

另有距离乘法比例误差 `range_fraction_sigma`（1-σ 0.1%），以及作用于实际光轴的
时间相关指向扰动 `SbirsPointingDisturbanceConfig`（`SbirsPolicyConfig.h:45-53`：
共模姿态 GM + 逐 NFOV 通道 GM/确定性振动），与量测误差模型相互独立。

**安装矩阵误差：❌ 无**。因 §1 所述 SBIRS 不存在安装矩阵，误差模型中**没有专门的
安装失准角（misalignment）误差系数**。语义最接近的是 `attitude_sigma_deg`
（卫星姿态误差）与 `common_attitude_sigma_deg`（共模扰动），但二者均非"安装矩阵
误差"。若合同要求安装矩阵误差系数，其前提是先落地指标一的安装矩阵——
**与 §1 强耦合，应同批设计、先后实施**（§8 阶段 2 → 阶段 3）。

## 6. 指标六：扫描模式 — ⚠️ 三种中具备两种

**宽视场扫描探测（WFOV）：✅**

- 方位环形扫描：起点/跨度 (0,360°]/方向枚举/速率 + 帧率 + 矩形视场门（az/el 独立
  半宽），`SbirsMissionConfig.h:28-41`；相位按 `dt_sec` 推进
  （`SbirsPipeline.cpp:532-535`），门控 `InRectangularFov(目标 LOS, 实际扫描方位,
  scan_center_el, WFOV az/el)`（`SbirsPipeline.cpp:636-638`、:1058）。
- 工作模式 `kStandby / kWideSearch / kSearchAndStare`（`SbirsMissionConfig.h:15`）。

**窄视场跟踪探测（NFOV）：✅ 且链路完整**

WFOV cue → NFOV 多通道调度器（`max_concurrent_nfov_locks`，`SbirsPolicyConfig.h:61-63`）
→ ATP 限速转动/稳定（最大转速/稳定容差/指向稳定误差，`SbirsMissionConfig.h:43-45`）
→ 首次捕获（几何窗 + SNR 双门，`SbirsNfovAcquisition.h`）
→ 持续跟踪三种互斥模式（Estimated EKF/IMM、Strict/Sensor-like 真值辅助，
`SbirsPolicyConfig.h:66-92`），含 NIS 门/几何门丢锁与惯性维持
（`SbirsPipeline.cpp:641-680`、`SbirsOutputTypes.h:53-60`）。

**大幅面扫描：❌ 无独立模式**

- WFOV 是**单一固定俯仰上的方位一维环扫**：`scan_center_el_deg` 为标量
  （`SbirsMissionConfig.h:37`），无俯仰步进/行栅格（raster）/大画幅帧概念；
  俯仰向覆盖仅 WFOV 俯仰视场本身（默认 ±10°，:30）。
- 若合同"大幅面扫描"指二维大区域帧扫或大焦平面画幅成像式搜索，当前不满足；
  方位跨度可到 360° 只解决"宽"、不解决"大幅面（俯仰维度）"。实施见 §8 阶段 4。

## 7. 缺口汇总与依赖关系

```
指标1 安装矩阵+指向链 ──────────┐
        （架构级，最大改动）      ├─→ 指标5 安装矩阵误差（前提：矩阵存在）
                                │
指标2 卫星速度输入（独立，边界级） │
指标4 最大探测距离输出（独立，边界级）
指标6 大幅面扫描（半独立：可先做 ECI 系下俯仰栅格，
                  若需在安装系下定义则依赖指标1）
指标3 τ(λ,d) 精细化（独立，小改，可选）
```

## 8. 分阶段实施建议

原则：每阶段独立可交付、可回归（单测 + 契约测试 + replay 一致性 + 模块文档同步，
遵循库内既有惯例）；阶段内先设计登记（docs/sbirs_sensor/algorithms.md 算法登记表）
后实现。建议每阶段一个分支，命名沿用 `feature/sbirs-sensor-contract-alignment-phaseN`。

**replay 兼容性前提（2026-08-17 确认）**：项目未上线，replay 不存在新旧逻辑之分，
schema/codec 直接演进，无版本化回退设计。各阶段涉及 replay 的改动均为
"schema + codec + 既有回放测试同步更新"，不保留兼容分支逻辑。

### 阶段 1：输入/输出边界扩展（低耦合地基，先行）

> **实施状态（2026-08-17）**：已落地于分支 `feature/sbirs-sensor-contract-alignment-phase1`。
> 卫星速度必填（含 ECI 旋转变换/相对视线角速度/cue 延迟卫星位移/EKF R 阵生效）；d_max(t)
> 进归属层（attribution + DebugView + lifecycle + replay），SNR 门失败目标 issue 消息附带
> 数值；replay schema 直接演进（无兼容分支）。设计定界：速度必填、d_max 挂归属层、
> WFOV 门限反解（NFOV 版由消费方推导）。

1. `SbirsCycleInput` 增加卫星速度（`velocity_ecef_m_per_s` + `has_` 标志，对齐目标
   速度的字段风格与校验规则：false 时必须有限零向量）。
2. ECI 变换把卫星速度旋入 ECI（含 ω×r 输运项，与目标同法）；
   `SbirsPipeline.cpp:633` 处相对速度改为 `v_target − v_satellite`。
   生效范围：动态滞后误差、cue 延迟外推、EKF R 阵。速度字段是否必填
   （项目未上线，可直接收紧为必填）在阶段设计中定界。
3. 输入校验（`SbirsInputValidation`）、replay flatbuffer schema/codec、
   `SbirsExternalInputAdapter`/`SbirsCycleInputAdapter` 同步扩展（直接演进，
   无兼容分支）。
4. 新增输出：当前时刻最大探测距离 `d_max(t)`（§4 反解公式）。设计决策待定：
   逐检测记录携带（依赖该目标 I_t，建议）vs 周期级参考签名；进
   `SbirsDetectionRecord`（WFOV 用 `wide_min_snr_linear`、NFOV 用
   `narrow_min_snr_linear` 各反解一个）还是归属/调试层，需在阶段设计里定界。
5. 测试：卫星速度参与的角速度/滞后/cue 外推单测；d_max 反解与气象联动单测
   （τ_eff 变化 → d_max 变化）；replay codec 一致性测试同步更新。

### 阶段 2：安装矩阵与指向参数逻辑对齐 AR（架构级，最大工程）

1. 新增 SBIRS 安装指向配置（对齐 `ArOrientationConfig` 语义）：安装欧拉角
   （Body→Sensor）、扫描限位、稳定方式；决定是否复用
   `oneq/foundation/pose_types.h` 与 AR 的姿态合成工具
   （`ComposeAttitudeDeg`），按库内 COMMON 收敛惯例抽公共内核。
2. `SbirsCycleInput` 增加卫星姿态输入（欧拉角或四元数，含有效性校验）。
3. 指向合成链：ECI 扫描参考定义 → 卫星姿态 → 安装矩阵 → 实际光轴；
   WFOV 扫描门控、NFOV ATP 指向、扰动模型作用点全部迁移到新链路。
4. **关键设计决策**：输出 az/el 是否保持 ECI 极坐标参考（2026-08 正式变更的
   现行约定）——建议输出参考系不变（消费方兼容），安装矩阵只影响内部光轴几何。
5. replay flatbuffer schema/codec 直接扩展（新增姿态/安装字段）；
   `docs/sbirs_sensor/` 四份文档与 `docs/common/session_contract.md` 同步修订。
6. 测试：合成链单测（多组姿态×安装角×扫描中心的光轴真值）、WFOV/NFOV 门控
   回归、replay codec 一致性测试同步更新、契约测试更新。

### 阶段 3：安装矩阵误差模型（依赖阶段 2）

1. `SbirsErrorModelConfig` 增加安装失准角误差系数（与 `attitude_sigma_deg`
   的语义边界要写清：安装误差是常值失准 vs 姿态误差是时刻误差；建议支持
   常值偏置 + 随机微扰两部分）。
2. 与 `SbirsPointingDisturbanceConfig` 共模项的关系梳理，避免双重计模。
3. RSS 合成、随机流种子管理、replay 可复现性验证；误差链单测与真值对比测试。

### 阶段 4：大幅面扫描（俯仰向二维扫描）

1. `SbirsMissionConfig` 扩展：俯仰扫描范围/步进行数/行驻留（或大画幅视场
   配置），与既有方位环扫参数正交组合；扫描相位状态从 1-D 扩到 2-D
   （replay 快照字段同步扩展）。
2. 覆盖门控：逐行矩形 FOV 门 + 帧率预算；WFOV→NFOV cue 逻辑不变。
3. 若大幅面须定义在安装系下，则排到阶段 2 之后；若先在 ECI 系下实现俯仰栅格，
   可与阶段 2 并行（推荐先并行后收敛，降低串行风险）。
4. 测试：2-D 扫描覆盖/相位推进单测、跨行目标重访周期测试、快照恢复测试。

### 阶段 5：大气模型精细化与收尾（可选/低优先）

1. `base_atmospheric_transmittance` 从标量升级为随几何路径（仰角/距离）变化的
   简化模型（如分段指数路径长度修正），保持气象交互项框架不变。
2. 全量回归 + 契约测试固化六项指标 + 模块文档终版。

### 里程碑与验收

- 每阶段出口判据：新增行为有单测覆盖、replay codec 与 schema 一致性验证通过、
  `docs/sbirs_sensor/` 与实现一致、契约测试绿。
- 阶段 1/2 完成后，六项指标中 #1/#2/#4/#5 的合同判定应可翻绿；
  阶段 4 完成后 #6 翻绿；阶段 5 仅提升 #3 的物理保真度。

## 附：审计证据方法说明

- "零命中"结论来自对 `src/sbirs_sensor`、`include/1q/sbirs_sensor`、
  `docs/sbirs_sensor` 的全文 grep（关键词：`mount`、`installation`、`安装`、
  `max_detection_range`、`detection_range` 等）。
- AR 对照证据取自 `feature/remote-identification-radar-phase1` 分支当前实现，
  非历史版本。
- 本审计未运行任何仿真/测试，结论基于静态代码与文档核对。
