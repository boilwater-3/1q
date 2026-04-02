# 光学传感器算法需求 vs REOS 开源库 — 审查报告

> 审查基准: `docs/electro_optical_sensor_algorithms.md` (3 层 6 模块)
>
> 审查对象: [bgin/Radar_ElectroOptical_Simulation](https://github.com/bgin/Radar_ElectroOptical_Simulation) (REOS)
>
> 审查日期: 2026-04-02

---

## 一、总体结论

| 指标 | 结果 |
|------|------|
| 需求算法总数 | 28 项 |
| 已实现 (原子级可复用) | 15 项 (53.6%) |
| 部分实现 (需适配) | 5 项 (17.9%) |
| 未实现 (需全新开发) | 8 项 (28.6%) |

REOS 库在 **基础计算层** 覆盖度最高，尤其在光学系统特性（几何/成像质量）和辐射度量学方面有大量成熟实现；这些实现大多停留在**算法原子**层面。**核心处理层** 仅有局部公式或参数计算，**接口层** 基本无对应框架实现。

---

## 二、逐模块详细映射

### 2.1 接口层 — 传感器初始化与步进控制模块

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 说明 |
|----------|-----------|--------------------|------|
| 传感器配置加载 | **未实现** | **已实现** | EOS 已提供 `EosSessionConfig`、`EosCycleInput` 与基础输入校验，可承接配置注入 |
| 仿真步进调度 | **未实现** | **已实现** | EOS 已提供 `EosSession::Step/StepWithResult` 的单周期步进门面 |
| 环境参数解析 | **未实现** | **已实现** | EOS 已提供太阳辐照度、大气透明度、云量、昼夜类型等输入字段 |

**结论**: 接口层 3 项能力均需全新开发。REOS 是函数库而非仿真框架，不提供调度/配置能力。

---

### 2.2 核心处理层 — 扫描覆盖与视场感知模块

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 |
|----------|-----------|--------------------|----------|
| 扫描角度递推 | **未实现** | **已实现** | `src/electro_optical_sensor/core/pipeline/EosPipeline.cpp` 中 `AdvanceScan` |
| 角度偏差计算 | **未实现** | **已实现** | `src/electro_optical_sensor/core/pipeline/EosPipeline.cpp` 中 `NormalizeAngle180` + 方位/俯仰偏差判定 |
| 瞬时视场判定 | **部分实现** | **已实现** | `src/electro_optical_sensor/core/pipeline/EosPipeline.cpp` 中 `IsTargetInCurrentFov` |

**关键源文件**:
- `Infrared EOS/GMS_eos_sensor_sse.f90:4501` — `fov_x_axis_xmm4r4(H, delta, gamma)` 计算水平方向视场角
- `Infrared EOS/GMS_eos_sensor_types.f90:501-564` — `fov_x_axis_r4_t` / `fov_y_axis_r4_t` 数据结构

**结论**: REOS 仅提供视场角参数计算基础；当前项目 EOS 已在 `core/pipeline` 完成扫描角递推与目标落入判定策略下沉。

---

### 2.3 核心处理层 — 多谱段融合探测模块

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 说明 |
|----------|-----------|--------------------|------|
| 工作模式分发 | **未实现** | **已实现** | EOS 已提供 `EosWorkMode` 的 IR/Visible/Fused 模式选择 |
| 红外探测评估 | **部分实现** | **已实现** | EOS 已在 `core/pipeline` 贯通红外辐射、背景、功率与 NEP-SNR 评估链路 |
| 可见光探测评估 | **部分实现** | **已实现** | EOS 已在 `core/pipeline` 使用 `ComputeVisibleChannelResult` + NEP-SNR 形成完整可见光链路 |
| 等权重 SNR 融合 | **未实现** | **已实现** | EOS 已在融合模式下对两路 SNR 做等权重合并 |
| 探测结果组织 | **未实现** | **已实现** | EOS 已提供 `EosOutputFrame` / `EosDetectionRecord` 输出结构 |

**结论**: 有红外和可见光的**底层计算原子**，但缺少完整的探测评估流水线、工作模式分发和融合框架。

---

### 2.4 基础计算层 — 目标辐射亮度计算模块

#### 红外通道

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 普朗克热辐射建模 | **已实现** | **已实现** | `EosRadiometry.cpp` / `EosRadiometry.h` | `ComputePlanckRadiance`、`ComputeInfraredRadianceDelta` |
| 红外背景辐射估计 | **已实现** | **已实现** | `EosRadiometry.cpp` / `EosPipeline.cpp` | `ComputeInfraredRadianceDelta` + 背景温度输入 |
| 红外对比度计算 | **已实现** | **已实现** | `EosRadiometry.cpp` | `ComputeRelativeContrast` |

#### 可见光通道

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 朗伯漫反射模型 | **已实现** | **已实现** | `EosRadiometry.cpp` / `EosRadiometry.h` | `ComputeVisibleLambertianRadiance` |
| 可见光背景亮度估计 | **已实现** | **已实现** | `EosRadiometry.cpp` / `EosPipeline.cpp` | `VisibleChannelInputs` + `ComputeVisibleChannelResult` |
| 归一化对比度计算 | **已实现** | **已实现** | `EosRadiometry.cpp` | `ComputeRelativeContrast` |

---

### 2.5 基础计算层 — 光学系统特性计算模块

#### 几何特性()

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 地面投影距离计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` / `EosPipeline.cpp` | `ComputeGroundProjectionDistanceM` |
| 扫描镜偏转角计算 | **已实现** | **已实现** | `EosPipeline.cpp` | 扫描角递推与视场相关几何 |
| 瞬时视场角计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeInstantaneousFovDeg` |
| 地面扫描幅宽计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeGroundScanWidthM` |
| 最大/最小探测距离计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` / `EosPipeline.cpp` | `ComputeMinimumDetectionRangeM`、`ComputeMaximumDetectionRangeM` 参数化接口已接入管线 |
| 焦高比计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeFocalHeightRatio` |

#### 成像质量

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 离焦系数计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeDefocusCoefficient` |
| 弥散圆计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeCircleOfConfusionDiameterM` |
| 折射角位移计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeRefractiveShiftM` |

#### 空间分辨率

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 说明 |
|----------|-----------|--------------------|------|
| 衍射极限角分辨率 | **已实现** | **已实现** | `ComputeDiffractionLimitedAngularResolutionRad` |
| 地面空间分辨率 | **已实现** | **已实现** | `ComputeGroundSampleDistanceM` |

#### 扫描运动

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 轨迹扫描速率计算 | **已实现** | **部分实现** | `EosPipeline.cpp` | 已用于扫描推进与 NEP 积分时间/带宽派生，尚未独立暴露轨迹速率结果结构 |

---

### 2.6 基础计算层 — 大气传输与接收功率计算模块

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 大气透过率/衰减评估 | **已实现** | **已实现** | `EosPropagation.cpp` | `ComputeBeerLambertTransmittance`、`ComputeAtmosphericTransmittance` |
| 大气折射 (详细) | **已实现** | **未实现** | — | 当前项目尚未单独实现详细大气折射模型 |
| 接收功率计算 | **已实现** | **已实现** | `EosPropagation.cpp` / `EosPipeline.cpp` | `ComputeReceivedPowerW`、`ComputeBackgroundFluxW` |
| 信噪比 (SNR) 计算 | **已实现** | **已实现** | `EosPropagation.cpp` / `EosPipeline.cpp` | `ComputeSnrLinear`、`ComputeSnrDb`、`EvaluateSnrWithNep` |

**重要说明**: REOS 中的 `compute_SN` / `compute_SM` 仍是 **几何投影函数**（根据 Miroshenko 的传感器理论），而非信噪比计算。这里之所以将 SNR 标记为 `已实现`，是因为本报告对应的基础计算层已经补齐了真正的 NEP/SNR 接口，而不是因为 REOS 原函数本身就是 SNR。

---

## 三、REOS 额外能力（需求文档未覆盖）

以下 REOS 实现的能力在需求文档中未提及，但可作为有益参考：

| 能力 | 源文件 | 说明 |
|------|--------|------|
| 遮光罩滤波分析 | `GMS_eos_noise_immune.f90:569-734` | `background_flux`, `schema_K1A/K1B` — 遮光罩抗干扰分析 |
| 空间频率谱分析 | `GMS_eos_noise_immune.f90:110-358` | 矩形/圆盘/高斯目标的 2D 空间频率谱 |
| 背景噪声统计 | `GMS_eos_noise_immune.f90:361-438` | 高斯分布背景噪声脉冲幅度分布 |
| 行星辐射特性 | `GMS_eos_noise_immune.f90:83-88` | 地球/火星/金星/水星/木星/土星辐射参数 |
| 星际消光积分 | `GMS_eos_radiometry.f90:516-563` | `ie_integrand` / `interstellar_extinct` |
| 大气模型 | `Atmosphere_model/` | MSISE00、HWM14 大气模型 |
| 辐射传输 | `Radiative Transfer/` | BRDF、RRTMG 长/短波辐射传输 |
| 全 SIMD 向量化 | 整个库 | SSE/AVX/AVX512 多精度多展开因子实现 |

---

## 四、可行性建议

### 高置信可复用 (直接映射)

1. **普朗克辐射计算** — `eos_radiometry` 模块提供了完整的辐射出射度和黑体辐射亮度函数
2. **光学系统几何特性** — 焦高比、扫描镜偏转角、瞬时视场角、扫描幅宽均已实现
3. **成像质量评估** — 离焦系数、弥散圆（半径和直径）均已实现
4. **大气衰减** — Burger 定律衰减和湿度吸收系数
5. **大气折射位移** — 完整的折射状态模型

### 需适配封装 (部分实现，REOS 侧)

6. **IR 对比度** — REOS 侧已有解析和数值对比度，当前项目已封装进 IR 链路
7. **可见光背景** — REOS 侧有日光辐照度模型，当前项目已落地完整可见光链路
8. **接收功率** — REOS 侧有通量计算基础，当前项目已落地焦平面接收功率
9. **瞬时视场判定** — REOS 侧仅有视场角计算，当前项目已落地目标落入判定逻辑

### 需全新开发 (REOS 侧)

10. **仿真步进调度** — 框架级代码，REOS 不涉及（当前项目已实现）
11. **扫描角度递推** — 时序逻辑，REOS 为无状态函数（当前项目已在 `core/pipeline` 实现）
12. **多谱段融合流水线** — IR/Visible/Fused 模式分发和 SNR 融合（当前项目已实现）
13. **朗伯漫反射模型** — REOS 侧无完整现成接口（当前项目已实现）
14. **衍射极限角分辨率 / GSD** — REOS 侧需迁移封装（当前项目已实现）
15. **信噪比 (SNR) 计算** — 需基于 NEP 的 SNR 评估（当前项目已实现 NEP 细化接口）
16. **环境参数解析** — REOS 侧不涉及（当前项目已实现）
17. **探测结果组织** — REOS 侧不涉及（当前项目已实现）

---

## 五、技术栈差异

| 维度 | 需求项目 | REOS 库 |
|------|----------|---------|
| 语言 | C++ (基于现有工程) | Fortran 90 |
| 并行策略 | 待定 | SSE/AVX/AVX512 SIMD + OpenMP |
| 架构风格 | 面向对象 3 层架构 | 无状态函数库 |
| 数值精度 | 待定 | 同时支持 sp(单精度) 和 dp(双精度) |
| 依赖 | 待定 | quadpack (数值积分) |

**建议**: REOS 算法应作为 **数学公式参考和验证基准**，而非直接端到端代码复用。将 Fortran 实现的数学公式翻译为 C++ 实现，并对照 REOS 的单元测试用例进行验证。

---

## 六、源文件索引

| 文件 | 大小 | 核心内容 |
|------|------|----------|
| `Infrared EOS/GMS_eos_sensor_types.f90` | 67KB | 所有 EOS 数据类型定义 |
| `Infrared EOS/GMS_eos_sensor_sse.f90` | 384KB (9087行) | SSE 向量化传感器算法 |
| `Infrared EOS/GMS_eos_radiometry.f90` | 20KB | 辐射度量学（普朗克、对比度、日照） |
| `Infrared EOS/GMS_eos_noise_immune.f90` | 40KB | 噪声抗扰分析（衰减、背景、遮光罩） |
| `Radiolocation/GMS_atmos_refraction.f90` | 545KB | 大气折射（标量版） |
| `Radiolocation/GMS_atmos_refraction_xmm4r4.f90` | — | 大气折射（SIMD 版） |
| `REOS_UnitTests/test_eos_sensor_types.f90` | 70KB | 传感器类型单元测试 |
| `REOS_UnitTests/test_defocus_cof_*.f90` | — | 离焦系数测试 |
| `REOS_UnitTests/test_ratio_FH_*.f90` | — | 焦高比测试 |
| `REOS_UnitTests/test_scan_mirror_ang_*.f90` | — | 扫描镜角度测试 |
| `REOS_UnitTests/test_circle_dispersion_*.f90` | — | 弥散圆测试 |
| `REOS_UnitTests/test_compute_SM_*.f90` | — | SM 几何计算测试 |
| `REOS_UnitTests/test_compute_SN_*.f90` | — | SN 几何计算测试 |
| `REOS_UnitTests/test_alloc_atmos_refraction.f90` | 53KB | 大气折射分配测试 |
| `Data/Atmos76_LUT_1_20km_del_100.txt` | 108KB | 大气查表数据 (1-20km) |

---

## 七、建议增补能力与接入清单（面向当前 EOS 模块）

### 7.1 优先级与实施顺序

1. **背景噪声统计**（P0）
2. **遮光罩滤波分析**（P0）
3. **空间频率谱分析**（P1）
4. **辐射传输（BRDF/RRTMG）接口化接入**（P2）
5. **大气模型（MSISE00/HWM14）可选接入**（P2）

> 说明：P0 优先解决“探测稳定性与误警”；P1 提升“可分辨性判定”；P2 解决“环境真实性”。

### 7.2 文件级改造清单

#### A) 背景噪声统计（P0）

- 新增 `include/1q/electro_optical_sensor/foundation/EosNoiseModel.h`
  - 定义背景噪声模型输入/输出（均值、方差、等效噪声功率、分布参数）。
- 新增 `src/electro_optical_sensor/foundation/EosNoiseModel.cpp`
  - 实现背景噪声统计计算（先支持高斯近似，保留分布扩展位）。
- 修改 `src/electro_optical_sensor/core/pipeline/EosPipeline.cpp`
  - 将背景噪声统计结果并入 NEP/SNR 评估（替代固定扣背景比例常量）。
- 新增 `tests/electro_optical_sensor/eos_noise_model_test.cpp`
  - 覆盖单调性、边界值、参数敏感性。
- 修改 `tests/CMakeLists.txt`
  - 注册新测试目标。

#### B) 遮光罩滤波分析（P0）

- 新增 `include/1q/electro_optical_sensor/foundation/EosStrayLight.h`
  - 定义遮光罩几何与滤波参数接口。
- 新增 `src/electro_optical_sensor/foundation/EosStrayLight.cpp`
  - 计算杂散光抑制增益/惩罚系数。
- 修改 `include/1q/electro_optical_sensor/core/session/EosSession.h`
  - 在 `EosSessionConfig` 增加遮光罩配置项（开关、有效角域、抑制系数）。
- 修改 `src/electro_optical_sensor/core/session/EosSession.cpp`
  - 将新配置映射到 `EosPipelineConfig`。
- 修改 `src/electro_optical_sensor/core/pipeline/EosPipeline.h/.cpp`
  - 在探测链路中引入杂散光惩罚项。
- 新增 `tests/electro_optical_sensor/eos_straylight_test.cpp`
  - 验证抑制前后 SNR 与探测门限行为。

#### C) 空间频率谱分析（P1）

- 新增 `include/1q/electro_optical_sensor/foundation/EosSpatialSpectrum.h`
  - 定义目标频谱特征与系统 MTF 近似输入。
- 新增 `src/electro_optical_sensor/foundation/EosSpatialSpectrum.cpp`
  - 实现矩形/圆形目标的简化频谱指标与可分辨性评分。
- 修改 `src/electro_optical_sensor/core/pipeline/EosPipeline.cpp`
  - 将频谱评分合并到 `imaging_quality_gain`（替换纯 GSD 比例项）。
- 新增 `tests/electro_optical_sensor/eos_spatial_spectrum_test.cpp`
  - 覆盖尺寸、距离、FOV 变化下的可分辨性趋势。

#### D) 辐射传输接口化（P2）

- 新增 `include/1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h`
  - 抽象透过率/路径辐射接口，支持模型枚举与参数集。
- 新增 `src/electro_optical_sensor/foundation/EosRadiativeTransfer.cpp`
  - 先实现 lightweight 参数化模型（作为 BRDF/RRTMG 代理层）。
- 修改 `src/electro_optical_sensor/core/pipeline/EosPipeline.cpp`
  - 将 `ComputeDerivedAtmosphericTransmittance` 替换为可插拔调用。
- 新增 `tests/electro_optical_sensor/eos_radiative_transfer_test.cpp`
  - 校验与当前简化模型的一致性和可替换性。

#### E) 高级大气模型可选接入（P2）

- 修改 `include/1q/electro_optical_sensor/core/session/EosSession.h`
  - 新增环境模型选择枚举（简化模型 / 高级模型）。
- 修改 `src/electro_optical_sensor/core/session/EosSession.cpp`
  - 将环境模型选择下沉到 `EosPipelineConfig`。
- 修改 `src/electro_optical_sensor/core/pipeline/EosPipeline.h/.cpp`
  - 根据模型类型选择大气参数来源。
- 新增 `src/electro_optical_sensor/environment/` 子模块文件
  - 维护高度、湿度、风场等驱动参数与查表/拟合逻辑。
- 新增 `tests/electro_optical_sensor/eos_environment_model_test.cpp`
  - 验证不同高度/气象条件对透过率与 SNR 的影响方向正确。

### 7.3 实施约束（建议）

- 第一阶段（P0）仅做“可解释参数模型”，不引入外部大型依赖。
- 第二阶段（P1/P2）先接口抽象再替换实现，保持现有测试可回归。
- 所有新增能力必须在 `tests/electro_optical_sensor/` 增加同名测试文件，并保留基线用例。
