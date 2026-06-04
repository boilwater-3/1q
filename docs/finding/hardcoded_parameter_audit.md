# 硬编码参数审计报告：可 XML 化 vs 物理推导 vs 保留常量

## 审计日期

2026-06-04

## 摘要

对 `flight_dynamic` 模块中所有硬编码参数进行全面审计，交叉对照 JSBSim 属性树中可用的运行时/初始化属性，判断每项参数是否可以：
- **A 类**：从 JSBSim 属性直接读取（property 已存在）
- **B 类**：从 JSBSim 属性间接推导（通过物理公式/统计关系）
- **C 类**：保留为命名常量（通用控制律参数，非 aircraft-specific）
- **D 类**：迁移到 profile + XML override（aircraft-specific capability）

---

## 一、JSBSim 属性可用性总览

### 1.1 所有机型普遍可用的属性（初始化和运行时）

| 属性路径 | 类型 | 说明 |
|----------|------|------|
| `metrics/Sw-sqft` | 初始化 | 翼面积 (ft²) |
| `metrics/bw-ft` | 初始化 | 翼展 (ft) |
| `inertia/weight-lbs` | 运行时 | 当前总重 (lbs) |
| `inertia/iyy-slugs_ft2` | 初始化 | 俯仰惯性矩 (slug·ft²) |
| `atmosphere/rho-slugs_ft3` | 运行时 | 当前空气密度 |
| `velocities/vc-kts` | 运行时 | 校准空速 (kts) |
| `velocities/vtrue-fps` | 运行时 | 真实空速 (fps) |
| `propulsion/engine[n]/thrust-lbs` | 运行时 | 当前推力 |

### 1.2 只有部分机型的属性

| 属性路径 | 有机型 | 用途 |
|----------|--------|------|
| `aero/alpha-max-rad` | **无** — 全 31 个机型均未定义 | 失速迎角 |
| `limits/vne-kts` | **无** — 全 31 个机型均未定义 | 最大速度 |
| `propulsion/magneto_cmd` | 仅活塞机型 (FGPiston) | 磁电机（已用于 has_mixture 检测） |

### 1.3 只有 3 个机型 XML 显式写了 guidance 属性

| 机型 | guidance 属性数 | 覆盖内容 |
|------|----------------|---------|
| B747 | 12 | landing 全部参数 + heavy_flare |
| c172x | 2 | roll-angle-limit, roll-rate-limit |
| global5000 | 2 | roll-angle-limit, roll-rate-limit |

**结论**：JSBSim 的 aircraft XML **不包含**任何可以直接读取的性能参数（Vne、失速迎角、升限等）。所有性能参数必须从 weight/wing_area/engine_type/Iyy 这四个基本属性**间接推导**。

---

## 二、逐参数审计

### 2.1 Autopilot.cpp — ApplyEnergyDefaults

| 参数 | 当前值 | 当前推导方式 | 可改进方案 | 分类 |
|------|--------|-------------|-----------|------|
| `stall_margin` | FBW:1.25, 重型:1.20, 中型:1.25, 活塞:1.30, 其他:1.30 | 类别表 | 可用 wing_loading 微调（高翼载→低裕度） | **B** |
| `cruise_factor` | FBW:3.5, 重型:3.5, 中型:3.2, 活塞:2.8, 其他:3.0 | 类别表 | 引擎类型+翼载的影响已有 V_stall 承载，factor 差异来自推进效率 | **C** — 通用气动常数 |
| `max_factor` | FBW:5.5, 重型:4.2, 中型:4.0, 活塞:3.5, 其他:3.8 | 类别表 | 同上 | **C** |
| `max_pitch_command_deg` | FBW:15, 重型:8, 中型:10, 轻活塞:12, 重活塞:10, 其他:20 | 类别表 | 无可用属性直接推导 | **D** — 可加 XML override |
| `max_roll_angle_deg` | struct:45°, 按类别:25-45° | 类别表，后续 XML roll-limit 覆盖 | c172x/global5000 已有 XML roll-limit | **已有 XML 路径** ✅ |
| `min_throttle` | FBW:0.35, 重型:0.55, 中型:0.40, 活塞:0.20-0.40, 其他:0.15 | 类别表 | 可用 engine_count 改进（多发→高 idle） | **D** — 可加 XML override |
| `speed_energy_priority` | FBW:true, 重型:true, 中型:true, 重活塞:true, 轻活塞:false, 其他:false | 类别表 | 推重比 > 0.5 → true | **B** — 可用 wing_loading 反推 |
| `ceiling_m` | FBW:15200, 重型:13700, 中型:12500, 活塞:4300, 其他:7600 | 类别表 | 无推力数据可用 | **D** — 已有 XML override |

### 2.2 Autopilot.cpp — ApplyRotationDefaults

| 参数 | 当前值 | 当前推导方式 | 可改进方案 | 分类 |
|------|--------|-------------|-----------|------|
| `rotation_ramp_sec` | Iyy>1e7:6s, >1e6:4s, default:3s | `log10(Iyy)` 连续值分档 | 可以 `ramp_sec = a + b * log10(Iyy)` 连续化 | **B** |
| `rotation_climb_rate_mps` | Iyy>1e7:3.0, >1e6:4.0, default:5.0 | `log10(Iyy)` 分档 | 可以用 V_stall 和 engine_type 调整 | **B** |
| `landing_heavy_flare` | 默认 false，B747 XML 显式开启 | **已解耦** ✅ | — | **已修复** ✅ |

### 2.3 EngineManager.cpp

| 参数 | 当前值 | 当前推导方式 | 可改进方案 | 分类 |
|------|--------|-------------|-----------|------|
| `CLmax` | piston:1.6, turboprop:2.0, delta_wing(AR<2.5):2.5 | 引擎类型 + AR 检测 | 无气动数据可用 | **C** — 气动分类常量 |
| `AR 阈值 2.5` | 固定 | 气动学定义 | — | **C** |
| `Vr_factor` | heavy_turbine:1.08, medium_turbine:1.15, light_turbine:1.20, piston:1.10, default:1.15 | 引擎类型 × Iyy 分档 | Iyy 已有，但 factor 本身是经验值 | **C** — 通用安全系数 |
| `Iyy 阈值` | 1e6, 1e7 | 重量级分类 | — | **C** |
| `Vr floor` | 40 kts | 安全常数 | — | **C** |
| `approach_speed fallback` | piston:28, turboprop:41, turbine:62, rocket:80, default:36 m/s | 引擎类型表 | 可改用 V_stall × 1.3 统一推导 | **B** |
| `climb_pitch` | piston:10°, turbine:15°, turboprop:10°, default:10° | 引擎类型表 | 无可用属性 | **D** — 可加 XML override |

### 2.4 Maneuver.cpp — Landing 控制律常量（已命名化，阶段 12b）

| 参数 | 值 | 分类 |
|------|----|------|
| `kLandingHighSpeedOrbitFactor` | 1.35 | **C** — 控制律参数 |
| `kLandingHighAltitudeOrbitMinSpeedMps` | 120.0 | **C** |
| `kLandingHighAltitudeOrbitMinRadiusM` | 3000.0 | **C** |
| `kLandingPatternOrbitMinRadiusM` | 2000.0 | **C** |
| `kLandingApproachEntrySpeedFactor` | 1.15 | **C** |
| `kLandingOrbitDisabledHighSpeedMps` | 200.0 | **C** — 旧代码遗留，被 profile 取代 |
| `kLandingFlareMinAltitudeM` | 15.0 | **C** |
| `kLandingFlareAltitudePerSpeed` | 0.60 | **C** |
| `kLandingHeavyFlareAltitudePerSpeed` | 0.25 | **C** |
| `kLandingFinalSinkTargetMps` | -3.0 | **C** |
| `kLandingFinalThrottleBase` | 0.25 | **C** |
| `kLandingFinalThrottleSinkGain` | 0.05 | **C** |
| `kLandingFinalThrottleCapSpeedFactor` | 1.05 | **C** |
| `kLandingHeavyFlareScaleM` | 30.0 | **C** |
| `kLandingHeavyFlareBaseElevator` | -0.08 | **C** |
| `kLandingStandardFlareInitialElevator` | -0.25 | **C** |
| `kLandingHeavyFlareInitialElevator` | -0.08 | **C** |
| bounce/float recovery thresholds | 0.10, 0.05, 10.0m, 5.0s | **C** |

### 2.5 Maneuver.cpp — Takeoff 控制律常量

| 参数 | 当前推导方式 | 分类 |
|------|-------------|------|
| `TakeoffIdleThrottle()` | 引擎类型表 | **C** |
| `TakeoffPowerThrottle()` | 引擎类型表 | **C** |
| `EngineStartDurationSec()` | 引擎类型表 | **C** |
| Takeoff pre-rotation elevator | `0.8 × rotation_max_elevator` at 80% Vr | **C** |
| Uncommanded liftoff AGL/vc 阈值 | 10m, 25kts | **C** |

---

## 三、关键发现

### 3.1 JSBSim 属性树可直接读取的属性极少

全 31 个 JSBSim 机型 XML **均未定义** `aero/alpha-max-rad` 和 `limits/vne-kts` 为 JSBSim 属性。这两项是推导完整飞行包线最需要的属性，但 JSBSim 模型将它们编码在气动系数表中，不暴露为独立属性。

**但是！** 引擎 XML 文件中包含大量的推力/功率数据。虽然这些数据不直接在 JSBSim 属性树中暴露，但它们存在于文件系统上，可以在初始化时解析。

### 3.2 引擎推力数据（间接可获取）

| 机型 | 引擎类型 | 数量 | 单发推力/功率 | 推重比估算 |
|------|----------|------|-------------|-----------|
| c172x | piston | 1 | 160 hp | 0.11 hp/lb |
| c172r | piston | 1 | 180 hp | 0.11 hp/lb |
| c182 | piston | 1 | 230 hp | 0.14 hp/lb |
| c310 | piston | 2 | 2×260 hp | 0.18 hp/lb |
| 737 | turbine | 2 | 2×20000 lbs | 0.48 |
| B747 | turbine | 4 | 4×58000 lbs | 0.44 |
| MD11 | turbine | 3 | 3×60200 lbs | 0.47 |
| f16 | turbine | 1 | 29000 lbs (AB) | 1.67 |
| f15 | turbine | 2 | 2×29000 lbs (AB) | 2.07 |
| A4 | turbine | 1 | 11200 lbs | 1.09 |
| DHC6 | turboprop | 2 | 2×680 hp | 0.17 hp/lb |
| Concorde | turbine | 4 | 4×37080 lbs (AB) | 0.86 |
| C130 | turbine | 4 | 4×10080 lbs | 0.38 |
| B17 | piston | 4 | 4×1200 hp | 0.13 hp/lb |

**这意味着推重比可以从 XML 数据间接推导**——对于喷气机用 `milthrust/weight`，对于活塞用 `hp/weight`。

### 3.3 只有 4 个属性可可靠用于物理推导

```
weight_lbs  +  wing_area_ft2  +  iyy_slugs_ft2  +  engine_type
     │                │                  │               │
     └──── V_stall ───┘                  │               │
           │                             │               │
    [速度包线基准]                  [重量级分类]    [推进类型]
```

**当前 V_stall 推导已经在用 weight + wing_area + CLmax，这是最合理的物理推导路径。**

### 3.3 当前"最硬"的硬编码

按严重程度排序：

1. **CLmax (1.6/2.0/2.5)** — 影响 V_stall → 影响整个速度包线。但无可用属性替代。**保留，但可加 XML override**。

2. **cruise_factor / max_factor** — 影响包线上限。类别表合理但仍有改进空间：可用 wing_loading 做微调（高翼载 = 高因子）。

3. **rotation_ramp_sec 的 log10(Iyy) 分段** — 可以连续化（`a + b * log10(Iyy)`），避免硬阈值。

4. **ceiling_m** — 无推力数据，只能走类别 + XML override。**已有 XML override 路径**。

5. **climb_pitch 和 approach_speed fallback** — 影响较小，可加 XML override。

### 3.4 "间接推导"的可行方案

#### 方案 1：wing_loading 微调 cruise_factor

```cpp
// 当前
cruise_factor = 2.8;  // 所有活塞一样

// 改进
double wl = profile->wing_loading_lbs_ft2;
cruise_factor = 2.5 + 0.02 * wl;  // c172x(14):2.78, c310(31):3.12
```

#### 方案 2：Iyy 连续化 rotation_ramp_sec

```cpp
// 当前
if (log_moi > 7.0) ramp = 6.0;
else if (log_moi > 6.0) ramp = 4.0;
else ramp = 3.0;

// 改进
ramp = std::clamp(1.5 + 0.6 * log_moi, 3.0, 6.0);
// c172x(3.13):3.4s, 737(6.18):5.2s, B747(7.52):6.0s
```

#### 方案 3：从 wing_loading 推 speed_energy_priority

```cpp
// 高翼载（>50 lbs/ft²）→ 速度优先（动能大，速度保护更重要）
speed_energy_priority = (wing_loading_lbs_ft2 > 50.0);
```

---

## 四、推荐行动

### 第一批：低风险，现有行为不变（建议立即执行）

| 改动 | 效果 | 风险 |
|------|------|------|
| `rotation_ramp_sec` 连续化 | 每架飞机不同 ramp，消除硬阈值 | 低 |
| `speed_energy_priority` 用 wing_loading 推导 | 替代类别布尔值 | 低 |
| `approach_speed fallback` 改用 V_stall×1.3 | 消除引擎类型表 | 低 |

### 第二批：基于引擎推力数据（中风险，需验证）

| 改动 | 效果 | 风险 |
|------|------|------|
| **从引擎 XML 读取 thrust/hp** → 计算推重比 | 获取每架飞机的真实推重比 | 中 — 引擎 XML 格式不统一 |
| **推重比微调 cruise_factor** | 高推重比→高巡航因子 | 中 — 需全 16 机验证 |
| **推重比 + wing_loading → ceiling_m** | 替代类别默认值 | 中 — 活塞和喷气需要不同公式 |

关于引擎推力数据：引擎 XML 中 `<milthrust>` 和 `<maxhp>` 等标签不在 JSBSim 属性树中暴露，但可以从文件系统读取。JSBSim 的 `FGEngine` 类内部持有这些值，但不通过 `GetProperty()` 暴露。

**替代方案**：在 Autopilot 初始化后的第一个 `Update()` 中，读取 `propulsion/engine[n]/thrust-lbs`（此时 throttle=1，地面静态），记录为 `max_static_thrust_lbs`。这是一个运行时测量值，无需解析 XML。

### 第三批：加 XML override（低风险）

| 改动 | 说明 |
|------|------|
| CLmax `guidance/takeoff-cl-max` | 允许特殊机型覆盖 |
| climb_pitch `guidance/climb-pitch-deg` | 允许覆盖 |
| Vr_factor `guidance/vr-factor` | 允许覆盖 |

### 第四批：保持不变

| 参数 | 原因 |
|------|------|
| 所有 kLanding* 控制律常量 | 已命名化 |
| AR 阈值 2.5 | 气动学定义 |
| Vr floor 40 kts | 安全硬约束 |

---

## 五、附录：全 31 机型属性清单

| 机型 | 引擎类型 | 翼面积(ft²) | 翼展(ft) | Iyy(slug·ft²) | alpha-max | Vne | guidance属性 |
|------|----------|------------|---------|---------------|-----------|-----|-------------|
| c172x | piston | 174 | 36 | 1.35e3 | ❌ | ❌ | roll-limit ×2 |
| c172p | piston | 174 | 35.8 | 1.35e3 | ❌ | ❌ | 无 |
| c172r | piston | 174 | 36.1 | 1.35e3 | ❌ | ❌ | 无 |
| c182 | piston | 174 | 35.8 | 1.35e3 | ❌ | ❌ | 无 |
| c310 | piston | 175 | 36.5 | 1.94e3 | ❌ | ❌ | 无 |
| 737 | turbine | 1171 | 94.7 | 1.47e6 | ❌ | ❌ | 无 |
| B747 | turbine | 5648 | 211.5 | 3.31e7 | ❌ | ❌ | landing ×12 |
| MD11 | turbine | 3648 | 169.5 | 3.84e7 | ❌ | ❌ | 无 |
| f16 | turbine(FBW) | 300 | 30 | 5.58e4 | ❌ | ❌ | 无 |
| f15 | turbine | 608 | 42.8 | 1.65e5 | ❌ | ❌ | 无 |
| A4 | turbine | 260 | 27.5 | 2.59e4 | ❌ | ❌ | 无 |
| F4N | turbine | 530 | 38.4 | 1.32e5 | ❌ | ❌ | 无 |
| T38 | turbine | 170 | 25.3 | 3.61e4 | ❌ | ❌ | 无 |
| DHC6 | turboprop | 422.5 | 65 | 2.47e4 | ❌ | ❌ | 无 |
| OV10 | turbine* | 291 | 40 | 3.56e4 | ❌ | ❌ | 无 |
| F80C | turbine | 237 | 38.8 | 2.85e4 | ❌ | ❌ | 无 |
| Boeing314 | piston | 2867 | 152 | 1.35 | ❌ | ❌ | 无 |
| Concorde | turbine | 3856 | 83.8 | 1.90e7 | ❌ | ❌ | 无 |
| C130 | turbine | 3070 | 132.5 | 2.39e6 | ❌ | ❌ | 无 |
| L410 | turboprop | 376.8 | 64.6 | 4.20e4 | ❌ | ❌ | 无 |
| B17 | piston | 1420 | 103.8 | 2.50e5 | ❌ | ❌ | 无 |
| F450 | electric | 0.016 | 0.127 | — | ❌ | ❌ | 无 |
| XB-70 | turbine(delta) | 6298 | 105 | 1.60e7 | ❌ | ❌ | 无 |
| global5000 | turbine | — | — | — | ❌ | ❌ | roll-limit ×2 |

> *OV10 的 JSBSim 引擎 XML 用 `<turbine_engine>` 但实际是涡桨

### 关键统计

- **所有 31 机型**: `alpha-max-rad` = 无, `vne-kts` = 无
- **有 guidance 属性**: 3/31 (B747, c172x, global5000)
- **可用的唯一物理属性**: weight, wing_area, wingspan, Iyy, engine_type/count
