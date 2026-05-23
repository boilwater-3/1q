# 统一大气模型模块设计

## 背景

当前项目存在 **三套碎片化的大气数据定义与消费模式**，各模块独立定义输入类型、独立填写 `AtmosphericPropagationInputs`、独立推导 `k_factor/day_of_year`。同时，内部物理层 [AtmospherePhysics.cpp](file:///Users/aurora/Code/1q/src/common/atmosphere/AtmospherePhysics.cpp) 的 GTD7 近似仅是 MSISE-00 的三段线性简化，缺乏物理精度。

项目已通过 `third_party/jsbsim` 集成了 JSBSim，其 `FGStandardAtmosphere` 实现了完整的 U.S. Standard Atmosphere 1976（ISA），包含 9 段温度断点、完整的气压层/递减层计算、湿度支持等。

**目标**：创建一个统一大气模型模块，基于 JSBSim ISA 提供精确的大气状态查询，统一各传感器模块的大气消费接口，消除重复代码。

---

## 现状诊断

### 三套碎片化的大气输入类型

| 层级 | 类型 | 字段 | 消费者 |
|------|------|------|--------|
| Foundation | [AtmosphericObservation](file:///Users/aurora/Code/1q/include/1q/foundation/atmospheric_types.h#L19-L24) | `enable_physical_model`, `pressure_hpa`, `temperature_k`, `relative_humidity` | AR, ESR (via alias) |
| Foundation | [SpaceWeatherContext](file:///Users/aurora/Code/1q/include/1q/foundation/atmospheric_types.h#L29-L37) | `k_factor`, `day_of_year`, `solar_flux_f107a/f107`, `geomagnetic_ap` | ESR (via alias) |
| AR 独有 | [AtmosphericDerivedContext](file:///Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentConfig.h#L95-L101) | `simulation_unix_seconds`, `solar_flux_f107a/f107`, `geomagnetic_ap` | AR only |
| ESR 独有 | [EsrAtmosphericObservation](file:///Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h#L65-L69) | `relative_humidity_ratio`, `precipitation_rate_mmph`, `visibility_km` | ESR only |
| EOS 独有 | [EosEnvironmentObservation](file:///Users/aurora/Code/1q/include/1q/electro_optical_sensor/environment/EosEnvironmentTypes.h#L27-L35) | `solar_altitude_deg`, `cloud_coverage_ratio`, `background_temperature_k` | EOS only |

### 各模块大气消费模式分析

#### Airborne Radar — 最重消费者

两处独立构造 `AtmosphericPropagationInputs`：

1. **[PropagationModel::Evaluate()](file:///Users/aurora/Code/1q/src/airborne_radar/environment/PropagationModel.cpp#L149-L190)** — 全局环境传播损耗。使用默认频率/路径参数，消费 `atmospheric_physics` + `atmospheric_context`。
2. **[DetectionExecution](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/DetectionExecution.cpp#L52-L77)** — 逐目标大气损耗。使用实际频率/距离/高度，消费相同的 `atmospheric_physics` + `atmospheric_context`。

两处都需要手工填充 14 个字段的 `AtmosphericPropagationInputs`（约 15 行重复代码），各自调用 `ResolveEffectiveKFactor()` 和 `ResolveEffectiveDayOfYear()`。

#### ESR — 中度消费者

**[EsrEnvironmentService](file:///Users/aurora/Code/1q/src/electronic_surveillance_radar/environment/EsrEnvironmentService.cpp#L105-L173)** 中一处构造，使用 **硬编码默认值**（10 GHz、10 km 路径、1 km 高度、5° 仰角），不依赖实际目标参数。

#### EOS — 隐式消费者

EOS 不直接消费 `AtmosphericPropagation*`，但 [EosRadiometry](file:///Users/aurora/Code/1q/src/electro_optical_sensor/foundation/EosRadiometry.cpp) 使用 `background_temperature_k` 作为黑体辐射的环境温度。当前由用户直接提供，未与大气模型关联。

#### Flight Dynamic — JSBSim 内置

飞行动力学模块通过 `FGFDMExec` 隐式使用 JSBSim 的 `FGStandardAtmosphere`，不显式访问大气 API。JSBSim 内部自动根据飞行高度计算温度/气压/密度。

### 内部物理层现状

[AtmospherePhysics.cpp](file:///Users/aurora/Code/1q/src/common/atmosphere/AtmospherePhysics.cpp) 包含：

- **Blake 大气损耗** — 简化指数气压模型 + 氧吸收/水汽吸收经验公式 ✅ 合理
- **折射率** — 基于 Magnus/Clausius-Clapeyron 的水汽压估算 ✅ 合理
- **GTD7 近似** — 三段线性温度 + 指数密度衰减 ⚠️ 过于简化
- **EvaluateAtmosphericPropagation** — 聚合入口 ✅ 保留改造

### JSBSim FGStandardAtmosphere API

> [!IMPORTANT]
> `FGStandardAtmosphere` 构造函数需要 `FGFDMExec*`，不能直接独立使用。但核心 ISA 数学（温度/气压/密度查表 + 递减率计算）不依赖 `FGFDMExec`，可以提取。

关键 API（均使用英制单位）：

| 方法 | 输入 | 输出 | 内部单位 |
|------|------|------|----------|
| `GetTemperature(alt)` | ft ASL | °R (Rankine) | 分段递减率查表 |
| `GetPressure(alt)` | ft ASL | PSF (lb/ft²) | 气压方程 33a/33b |
| `GetDensity(alt)` | ft ASL | slug/ft³ | P/(R·T) |
| `GetSoundSpeed(alt)` | ft ASL | ft/s | √(γRT) |
| `GetStd*(alt)` | ft ASL | — | 无偏差的纯 ISA 值 |
| `Set*Bias/Delta` | — | — | 可定制偏差/梯度 |
| `SetRelativeHumidity(RH)` | RH (%) | — | 修改水汽分压 |

ISA 1976 温度断点（9 层）：

```
海平面     →  11 km:  -6.5 °C/km (对流层)
11 km      →  20 km:  等温层
20 km      →  32 km:  +1.0 °C/km
32 km      →  47 km:  +2.8 °C/km
47 km      →  51 km:  等温层
51 km      →  71 km:  -2.8 °C/km
71 km      → ~85 km:  -2.0 °C/km
```

---

## User Review Required

> [!IMPORTANT]
> **JSBSim ISA 数学提取 vs. 自实现 ISA**
>
> 方案 A：从 `FGStandardAtmosphere` 提取纯数学实现到独立类（无 `FGFDMExec` 依赖）。优点是与 JSBSim 飞行动力学的大气完全一致；缺点是 ISA 数学部分散落在多个 protected 方法中，提取需要理解内部状态。
>
> 方案 B：基于 ISA 1976 文档自行实现纯 C++ ISA（无 JSBSim 依赖）。优点是完全自主、零外部依赖、代码更精简；缺点是需要与 JSBSim 做交叉验证。
>
> **建议采用方案 B**——自实现 ISA 1976。理由：
> 1. ISA 1976 是开放标准，数学明确（温度查表 + 两种气压公式），自实现约 150-200 行 C++。
> 2. 避免对 JSBSim 内部实现的紧耦合（Rankine/PSF 单位转换、FGTable 依赖等）。
> 3. 可以直接使用 SI 单位，与项目 convention 一致。
> 4. 测试中与 JSBSim 的 `GetStdTemperature`/`GetStdPressure` 做交叉验证，确保一致性。

> [!WARNING]
> **公共 API 破坏性变更决策**
>
> 各模块的公共输入类型（`AtmosphericObservation`、`EsrAtmosphericObservation`、`EosEnvironmentObservation`）已经暴露在 `include/1q/` 下，且被 FlatBuffers schema 序列化。
>
> 建议本次 **不修改现有公共输入类型**，仅在内部统一消费层引入 `AtmosphericState` 输出。各模块现有的 `environment` 层继续接受模块特定的输入，但内部通过统一的 `IAtmosphereProvider` 获取精确的大气状态，替代各自手工填充 `AtmosphericPropagationInputs` 的重复代码。

---

## Open Questions

> [!IMPORTANT]
> **Q1: 大气状态输出是否需要包含声速？**
>
> 当前雷达模块不需要声速，但 EOS 的气溶胶散射模型可能间接需要。声速计算量极小（`sqrt(γRT)`），建议默认包含。

> [!NOTE]
> **Q2: 高海拔扩展（>86 km）是否需要？**
>
> ISA 1976 在 86 km 以上需要不同的压力计算方法。当前项目的传感器工作高度范围通常在 0-30 km。建议首版仅覆盖 0-86 km，高海拔留 stub 接口。

> [!NOTE]
> **Q3: 是否需要大气偏差/自定义能力？**
>
> JSBSim 的 `SetTemperatureBias`/`SetSLTemperatureGradedDelta` 允许修改标准大气。建议首版仅实现标准 ISA，偏差注入作为 Phase 2 扩展。

---

## Proposed Changes

### Component 1: Foundation — 统一大气状态类型

新增统一输出结构体，放置在 `include/1q/foundation/` 下，供所有模块消费。

#### [NEW] [atmosphere_state.h](file:///Users/aurora/Code/1q/include/1q/foundation/atmosphere_state.h)

```cpp
namespace oneq { namespace foundation {

/// 统一大气状态查询结果（SI 单位制）。
struct ONEQ_API AtmosphericState {
  float altitude_m{0.0f};          ///< 查询高度（输入回显）
  float temperature_k{288.15f};    ///< 温度（K）
  float pressure_pa{101325.0f};    ///< 气压（Pa）
  float density_kg_m3{1.225f};     ///< 密度（kg/m³）
  float speed_of_sound_mps{340.29f}; ///< 声速（m/s）
  float pressure_hpa{1013.25f};    ///< 气压（hPa，便利字段 = pressure_pa/100）
};

}}
```

#### [NEW] [atmosphere_provider.h](file:///Users/aurora/Code/1q/include/1q/foundation/atmosphere_provider.h)

```cpp
namespace oneq { namespace foundation {

/// 大气模型抽象接口。
class ONEQ_API IAtmosphereProvider {
 public:
  virtual ~IAtmosphereProvider() = default;

  /// 查询指定几何高度处的大气状态。
  virtual AtmosphericState GetState(float altitude_m) const = 0;

  /// 查询海平面大气状态。
  virtual AtmosphericState GetSeaLevelState() const = 0;
};

}}
```

---

### Component 2: 共享实现 — ISA 1976 标准大气

在 `src/common/atmosphere/` 下新增 ISA 实现，替代现有 GTD7 近似。

#### [NEW] [StandardAtmosphere.h](file:///Users/aurora/Code/1q/src/common/atmosphere/StandardAtmosphere.h)

ISA 1976 实现类，实现 `IAtmosphereProvider` 接口。

- 9 段温度断点表（对流层到中间层顶）
- 梯度层：`P = P_b * (T_b / (T_b + L*ΔH))^(g0*M/(R*L))`
- 等温层：`P = P_b * exp(-g0*M*ΔH/(R*T_b))`
- 密度：`ρ = P / (R_specific * T)`
- 声速：`a = sqrt(γ * R_specific * T)`
- 全 SI 单位（m, K, Pa, kg/m³, m/s）

#### [NEW] [StandardAtmosphere.cpp](file:///Users/aurora/Code/1q/src/common/atmosphere/StandardAtmosphere.cpp)

实现细节：
- 几何高度 → 位势高度转换：`H_geopotential = r_earth * h / (r_earth + h)`
- 查表确定所在层，计算温度、气压
- 缓存海平面状态避免重复计算

#### [MODIFY] [AtmospherePhysics.h](file:///Users/aurora/Code/1q/src/common/atmosphere/AtmospherePhysics.h)

- **保留**：`blake_atmos_loss_*`、`refractivity_index_*`、`EvaluateAtmosphericPropagation` 及其输入输出结构体
- **重构 `GTD7()` 函数**：内部实现改为委托到 `StandardAtmosphere::GetState()`，返回值不变（保持 ABI 兼容），但精度大幅提升
- **新增便利入口**：`AtmosphericPropagationInputs BuildPropagationInputs(const AtmosphericState& state, float frequency_hz, float path_length_m, ...)` — 消除各模块手工填充 14 字段的重复代码

---

### Component 3: Airborne Radar — 消费层重构

#### [MODIFY] [PropagationModel.cpp](file:///Users/aurora/Code/1q/src/airborne_radar/environment/PropagationModel.cpp)

- `Evaluate()` 中 15 行手工填充 `AtmosphericPropagationInputs` → 调用 `BuildPropagationInputs()` 一行完成
- 移除对 `ResolveEffectiveKFactor` / `ResolveEffectiveDayOfYear` 的直接调用（由 `BuildPropagationInputs` 内部处理）

#### [MODIFY] [DetectionExecution.cpp](file:///Users/aurora/Code/1q/src/airborne_radar/signal/pipeline/DetectionExecution.cpp)

- `ComputeTargetSpecificAtmosphericLossDb()` 中类似重构，使用统一入口

---

### Component 4: ESR — 消费层重构

#### [MODIFY] [EsrEnvironmentService.cpp](file:///Users/aurora/Code/1q/src/electronic_surveillance_radar/environment/EsrEnvironmentService.cpp)

- `BuildSnapshot()` 中 18 行 `AtmosphericPropagationInputs` 构造 → 使用 `BuildPropagationInputs()`
- 移除硬编码的 `kDefaultAtmosphere*` 常量（由调用者或配置决定）

---

### Component 5: 构建系统

#### [MODIFY] [src/common/CMakeLists.txt](file:///Users/aurora/Code/1q/src/common/CMakeLists.txt)

- 新增 `common/atmosphere/StandardAtmosphere.cpp` 到 common OBJECT 库源文件列表

**无需新增 JSBSim 链接依赖** — 自实现 ISA 无外部依赖。

---

### Component 6: 测试

#### [NEW] [tests/unit/common_standard_atmosphere_test.cpp](file:///Users/aurora/Code/1q/tests/unit/common_standard_atmosphere_test.cpp)

测试矩阵：

| 测试项 | 验证内容 |
|--------|----------|
| ISA 断点精确值 | 海平面 T=288.15K, P=101325Pa, ρ=1.225kg/m³ |
| 对流层递减率 | 11km 处 T=216.65K |
| 等温层 | 11-20km 温度恒定 216.65K |
| 气压公式 | 与 ISA 1976 文档 Table 4 对比 |
| 密度计算 | ρ=P/(R_specific*T) 一致性 |
| 声速计算 | a=sqrt(γ*R*T) 一致性 |
| 边界条件 | 负高度钳位、极高高度处理 |
| JSBSim 交叉验证 | 与 `FGStandardAtmosphere::GetStd*()` 在 0-86km 范围内误差 < 0.01% |

#### [MODIFY] [tests/unit/ar_atmosphere_physics_test.cpp](file:///Users/aurora/Code/1q/tests/unit/ar_atmosphere_physics_test.cpp)

- 新增 `BuildPropagationInputs()` 便利函数测试
- 验证重构后的 `GTD7()` 与新 ISA 实现的一致性

---

## 文件变更总览

```
include/1q/foundation/
├── [NEW] atmosphere_state.h              统一大气状态输出
├── [NEW] atmosphere_provider.h           抽象接口
└── atmospheric_types.h                   (不修改)

src/common/atmosphere/
├── [NEW] StandardAtmosphere.h            ISA 1976 实现头文件
├── [NEW] StandardAtmosphere.cpp          ISA 1976 实现
├── [MODIFY] AtmospherePhysics.h          新增便利入口声明
└── [MODIFY] AtmospherePhysics.cpp        GTD7→ISA 委托 + 便利函数实现

src/common/
└── [MODIFY] CMakeLists.txt               新增源文件

src/airborne_radar/
├── environment/
│   └── [MODIFY] PropagationModel.cpp     消费层简化
└── signal/pipeline/
    └── [MODIFY] DetectionExecution.cpp   消费层简化

src/electronic_surveillance_radar/
└── environment/
    └── [MODIFY] EsrEnvironmentService.cpp 消费层简化

tests/unit/
├── [NEW] common_standard_atmosphere_test.cpp ISA + 交叉验证
└── [MODIFY] ar_atmosphere_physics_test.cpp   便利函数测试
```

---

## 不变更的部分

- **公共输入类型不变**：`AtmosphericObservation`、`SpaceWeatherContext`、`EsrAtmosphericObservation`、`EosEnvironmentObservation` 均保持现有 API
- **FlatBuffers schema 不变**：序列化格式保持向后兼容
- **EOS 模块暂不修改**：EOS 的 `background_temperature_k` 是辐射学参数而非大气模型参数，不在本次范围内
- **Flight Dynamic 模块不修改**：继续使用 JSBSim 内置大气

---

## Verification Plan

### Automated Tests

```bash
# 1. Configure + Build
cmake --preset llvm-ninja-debug-local >/tmp/1q-cmake.log 2>&1
cmake --build --preset llvm-ninja-debug-local >/tmp/1q-build.log 2>&1

# 2. Run atmosphere tests
ctest --preset llvm-ninja-debug-local --output-on-failure -j 4 \
  -R "common_standard_atmosphere_test|ar_atmosphere_physics_test"

# 3. Run full test suite (regression)
ctest --preset llvm-ninja-debug-local --output-on-failure -j 4
```

### Manual Verification

1. ISA 断点精确值与 [NASA TM-X-74335](https://ntrs.nasa.gov/citations/19770009539) Table 4 对比
2. 与 JSBSim `FGStandardAtmosphere::GetStdTemperature/GetStdPressure` 在 0-86km 做交叉验证（编写专用测试用例）
3. 确认 AR/ESR 现有测试在重构后无回归
