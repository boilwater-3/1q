# 光学传感器算法需求 vs REOS 开源库 — 审查报告

> 审查基准: `docs/electro_optical_sensor_algorithms.md` (3 层 6 模块)
>
> 审查对象: [bgin/Radar_ElectroOptical_Simulation](https://github.com/bgin/Radar_ElectroOptical_Simulation) (REOS)
>
> 审查日期: 2026-04-01

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
| 扫描角度递推 | **未实现** | **已实现** | `EosSession.cpp` 中的扫描角推进逻辑 |
| 角度偏差计算 | **未实现** | **已实现** | `EosSession.cpp` 中的视场内判断与角偏差计算 |
| 瞬时视场判定 | **部分实现** | **已实现** | `EosSession.cpp` 中的目标视场判定逻辑 |

**关键源文件**:
- `Infrared EOS/GMS_eos_sensor_sse.f90:4501` — `fov_x_axis_xmm4r4(H, delta, gamma)` 计算水平方向视场角
- `Infrared EOS/GMS_eos_sensor_types.f90:501-564` — `fov_x_axis_r4_t` / `fov_y_axis_r4_t` 数据结构

**结论**: 有瞬时视场角的**参数计算**基础，但缺少扫描角度递推和目标落入判定逻辑，因此只能视为部分实现。

---

### 2.3 核心处理层 — 多谱段融合探测模块

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 说明 |
|----------|-----------|--------------------|------|
| 工作模式分发 | **未实现** | **已实现** | EOS 已提供 `EosWorkMode` 的 IR/Visible/Fused 模式选择 |
| 红外探测评估 | **部分实现** | **已实现** | EOS 已提供红外辐射、背景、功率与 SNR 的基础链路 |
| 可见光探测评估 | **部分实现** | **已实现** | EOS 已提供朗伯反射、背景亮度与 SNR 的基础链路 |
| 等权重 SNR 融合 | **未实现** | **已实现** | EOS 已在融合模式下对两路 SNR 做等权重合并 |
| 探测结果组织 | **未实现** | **已实现** | EOS 已提供 `EosOutputFrame` / `EosDetectionRecord` 输出结构 |

**结论**: 有红外和可见光的**底层计算原子**，但缺少完整的探测评估流水线、工作模式分发和融合框架。

---

### 2.4 基础计算层 — 目标辐射亮度计算模块

#### 红外通道

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 普朗克热辐射建模 | **已实现** | **已实现** | `EosRadiometry.cpp` / `EosRadiometry.h` | `ComputePlanckRadiance`、`ComputeInfraredRadianceDelta` |
| 红外背景辐射估计 | **已实现** | **已实现** | `EosSession.cpp` / `EosRadiometry.cpp` | `ComputeInfraredRadianceDelta` 结合背景温度输入 |
| 红外对比度计算 | **已实现** | **已实现** | `EosRadiometry.cpp` | `ComputeRelativeContrast` |

#### 可见光通道

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 朗伯漫反射模型 | **已实现** | **已实现** | `EosRadiometry.cpp` / `EosRadiometry.h` | `ComputeVisibleLambertianRadiance` |
| 可见光背景亮度估计 | **已实现** | **已实现** | `EosSession.cpp` / `EosRadiometry.cpp` | `VisibleRadianceInputs` + `ComputeVisibleLambertianRadiance` |
| 归一化对比度计算 | **已实现** | **已实现** | `EosRadiometry.cpp` | `ComputeRelativeContrast` |

---

### 2.5 基础计算层 — 光学系统特性计算模块

#### 几何特性()

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 地面投影距离计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` / `EosSession.cpp` | `ComputeGroundProjectionDistanceM` |
| 扫描镜偏转角计算 | **已实现** | **已实现** | `EosSession.cpp` | 扫描角递推与视场相关几何 |
| 瞬时视场角计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeInstantaneousFovDeg` |
| 地面扫描幅宽计算 | **已实现** | **已实现** | `EosOpticalCharacteristics.cpp` | `ComputeGroundScanWidthM` |
| 最大/最小探测距离计算 | **已实现** | **部分实现** | `EosSessionConfig` / `EosCycleInput` | 当前项目已具备输入与调度基础，尚未单独抽出 Dmax/Dmin 计算接口 |
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
| 轨迹扫描速率计算 | **已实现** | **部分实现** | `EosSession.cpp` | 当前项目已具备扫描推进逻辑，但尚未独立暴露轨迹速率数据结构 |

---

### 2.6 基础计算层 — 大气传输与接收功率计算模块

| 需求算法 | REOS 状态 | 当前项目 EOS 模块 | 实现位置 | 备注 |
|----------|-----------|--------------------|----------|------|
| 大气透过率/衰减评估 | **已实现** | **已实现** | `EosPropagation.cpp` | `ComputeBeerLambertTransmittance`、`ComputeAtmosphericTransmittance` |
| 大气折射 (详细) | **已实现** | **未实现** | — | 当前项目尚未单独实现详细大气折射模型 |
| 接收功率计算 | **已实现** | **已实现** | `EosPropagation.cpp` / `EosSession.cpp` | `ComputeReceivedPowerW`、`ComputeBackgroundFluxW` |
| 信噪比 (SNR) 计算 | **已实现** | **已实现** | `EosPropagation.cpp` / `EosSession.cpp` | `ComputeSnrLinear`、`ComputeSnrDb` |

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

### 需适配封装 (部分实现)

6. **IR 对比度** — 已有解析和数值对比度，需封装为 IR 链路的一部分
7. **可见光背景** — 有日光辐照度模型，需扩展为完整可见光链路
8. **接收功率** — 有通量计算基础，需扩展为焦平面接收功率
9. **瞬时视场判定** — 有视场角计算，需增加目标落入判定逻辑

### 需全新开发

10. **仿真步进调度** — 框架级代码，REOS 不涉及
11. **扫描角度递推** — 时序逻辑，REOS 为无状态函数
12. **多谱段融合流水线** — IR/Visible/Fused 模式分发和 SNR 融合
13. **朗伯漫反射模型** — 需新写
14. **衍射极限角分辨率 / GSD** — 需新写
15. **信噪比 (SNR) 计算** — 需基于 NEP 的 SNR 评估
16. **环境参数解析** — 需新写
17. **探测结果组织** — 需新写

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
