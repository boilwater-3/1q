# 发现：JSBSim 飞行机动模块

## 引擎类型与属性

6 类引擎，按 `propulsion/magneto_cmd` 区分活塞/非活塞：

| 类型 | JSBSim 类 | magneto | mixture | 机型 |
|------|----------|:---:|:---:|------|
| 活塞 | FGPiston | ✅ | ✅ | c172x, c310, B17, J3Cub, p51d |
| 涡喷/涡扇 | FGTurbine | ❌ | ❌ | f16, f15, 737, B747, Concorde, MD11 |
| 涡桨 | FGTurboprop | ❌ | ❌ | L410, DHC6, PC7 |
| 火箭 | FGRocket | ❌ | ❌ | X15, x24b |
| 电动 | FGElectric | ❌ | ❌ | F450 |

抬轮速度：`Vr = factor × sqrt(2W / (ρ·S·CLmax))`

## JSBSim 属性可用性（全 31 机型审计，2026-06-04）

### 普遍可用
`metrics/Sw-sqft`, `metrics/bw-ft`, `inertia/weight-lbs`, `inertia/iyy-slugs_ft2`, `atmosphere/rho-slugs_ft3`, `velocities/vc-kts`, `velocities/vtrue-fps`

### 全 31 机型均无
`aero/alpha-max-rad`（失速迎角）、`limits/vne-kts`（最大速度）

### 引擎 XML 中有但 JSBSim 属性树中无
`<milthrust>`（喷气推力 lbs）、`<maxhp>`（活塞马力）

### 有 guidance XML 覆盖的机型（仅 3 个）
B747（12 属性）、c172x（2 属性：roll-limit）、global5000（2 属性：roll-limit）

## 架构关键发现

### AP 动态检测（零型号名硬编码）
```
has_fbw_override || has_roll_rate_command → kFbwRateCommand
has_own_autopilot                        → kOwnAutopilot
has_generic_autopilot                    → kGenericAutopilotBridge
default                                  → kDirectSurface
```

### has_mixture 检测
`fcs/mixture-cmd-norm` 对所有引擎类型创建 indexed 属性。必须同时检查 `propulsion/magneto_cmd`（仅 FGPiston 创建）。

### 惯性速度含地球自转
`GetInertialVelocityMagnitude()` 返回 ECEF 速度（赤道 +465 m/s）。改用 TAS：`velocities/vtrue-fps`。

### Iyy 属性名
JSBSim XML `unit="SLUG*FT2"`，属性路径 `inertia/iyy-slugs_ft2`（非 `iyy-lbsft2`）。

## TWR 估算方案（阶段 13a，2026-06-04）

### 方案选择
- ❌ 直接访问 `FGTurbine::MilThrust` 等私有成员 — Conan 包中为 private，无 getter
- ❌ 调用 `InitRunning(-1)` 测量 — 会改变引擎 N1/N2 状态，导致 MD11 着陆回归
- ✅ 非侵入式估算：从当前 `thrust-lbs`/`power-hp` 线性缩放 → rated_thrust ≈ current / throttle

### JSBSim 引擎属性可用性
| 属性 | 类型 | 可用性 |
|------|------|--------|
| `propulsion/engine[n]/thrust-lbs` | 当前推力 | 所有引擎类型 |
| `propulsion/engine[n]/power-hp` | 当前功率(HP) | 活塞 + 涡桨 |
| `fcs/throttle-cmd-norm` | 油门位置 | 所有 |
| `FGTurbine::MilThrust` | 额定推力 | **private**，不可访问 |
| `FGPiston::MaxHP` | 额定功率 | **private**，不可访问 |
| `FGTurboProp::MaxPower` | 额定功率 | **private**，不可访问 |

### TWR 测量值（估算）
| 机型 | TWR | 引擎类型 | 备注 |
|------|-----|---------|------|
| c172x | ~0.42 | piston | 活塞用 HP×3.5 lbs/HP 转换 |
| DHC6 | ~0.15 | turboprop | trim throttle 低，缩放后偏小 |
| B747 | ~0.28 | turbine | do_trim=false，缩放效果好 |
| XB-70 | ~0.19 | turbine | trim throttle 低 |

### 关键决策
- **为什么不用 InitRunning**：`FGTurbine::InitRunning()` 虽然用 `SuspendIntegration()` 不推进仿真，但会重置 N1/N2/Cutoff/phase，二次 InitRunning 也无法完全恢复 trim 状态（MD11 2431s crash）
- **构造顺序**：FlightManager 中 EngineManager 在 Autopilot 之前构造，确保 `guidance/thrust-to-weight` 属性在 Autopilot 构造时可用

## 硬编码参数分类（详见 docs/finding/hardcoded_parameter_audit.md）

| 分类 | 说明 | 例子 |
|------|------|------|
| **B 类**（物理推导） | 可用 weight/wing/Iyy/thrust 改进 | rotation_ramp, cruise_factor, ceiling |
| **C 类**（控制律常量） | 非 aircraft-specific，保持命名化 | kLanding* 常量, Vr floor, AR 阈值 |
| **D 类**（XML override） | aircraft capability | CLmax, climb_pitch, Vr_factor |

## MD11 着陆回归分析（2026-06-04）

commit 8484c90c 的 B747 flare 改进用了 `log10(Iyy)>7.0` 判断重型机，三项改动误套到 MD11（Iyy=3.8e7, log10=7.58）：

| 参数 | 旧值（通过） | 8484c90c 改动 | 阶段 12 修复 |
|------|-------------|--------------|-------------|
| flare 起始高度 | vc×0.25 (25m) | vc×0.6 (60m，统一) | 重型无 heavy_flare→vc×0.25 |
| flare 初始 elevator | -0.25 | -0.08 (log10>7) | landing_heavy_flare=false→-0.25 |
| flare 控制律 | 标准渐进式 | B747 弹跳/浮空恢复 | landing_heavy_flare=false→标准 |

修复方案：`landing_heavy_flare` 不再默认 log10>7（仅 B747 XML 开启）；flare 高度缩减基于 `is_heavy && !landing_heavy_flare`；throttle cap 默认提到 0.60。

## B747 着陆问题

**✅ 已解决**（阶段 13b 副作用）。连续化 cruise_factor（从 3.5 离散值→3.44）使 B747 进近速度管理平滑化，着陆从 abort(1451s) 变为 completed(1485s)。无需独立修复。

## 13c 跳过分析

`rotation_ramp_sec` 连续化的物理路径需要 `α = M_elevator / Iyy`，但 `M_elevator`（升降舵俯仰力矩）不在 JSBSim 属性树中——编码在气动系数表里不可读。任何纯 `log10(Iyy)` 线性公式都只是曲线拟合。硬阈值工作良好，不更换。

## 13d 关键决策：speed_energy_priority 阈值

`wing_loading > 50` 的物理依据：
- KE = ½mv² ∝ W × V_stall² ∝ W²/(ρ·S·CLmax) ∝ WL × W/CLmax
- 高 WL → 单位重量携带更多动能 → 速度损失后需要更多推力来恢复 → 优先保护速度

变化情况：
| 机型 | WL | 旧值 | 新值 | 正确性 |
|------|-----|------|------|-------|
| f15 | 55 | false | true | ✅ 高翼载战斗机需速度保护 |
| c310 | 22 | true | false | ✅ 低翼载双发不需要 |
| C130 | 31 | true | false | ✅ 低速运输机 |
| B17 | 25 | true | false | ✅ 低翼载活塞 |

## 13e 新增 XML override 属性

| 属性 | 覆盖内容 | 默认来源 | 验证 |
|------|---------|---------|------|
| `guidance/takeoff-cl-max` | CLmax | 引擎类型+AR检测 | >0.5 生效 |
| `guidance/takeoff-vr-factor` | Vr安全系数 | Iyy+引擎类型 | (0.5, 2.0) 生效 |
| `guidance/climb-pitch-deg` | 爬升俯仰角 | 引擎类型表 | (0, 45) 生效 |

无 XML 属性时行为完全不变（非侵入式）。

## AP 收敛瓶颈分析（阶段 15，2026-06-05）

### 根因：稳态偏差（非振荡）
AP 高度/航向保持使用纯 PD 控制器（无积分项），对某些机型无法消除残余误差。
- **SetAltitude**：高空推力限制，pitch 命令产生的升力恰好平衡阻力，无法继续爬升
- **SetHeading**：小航向误差下 roll 命令极弱（0.7°），不足以产生足够偏航率

### 稳态偏差数据（9 机型）
| 机型 | 目标高度 | 稳态实际 | 偏差 | 物理原因 |
|------|---------|---------|------|---------|
| F80C | 4200m | 4123.8m | 76m | 轻型喷气高空推力不足 |
| OV10 | 4200m | 4130.8m | 69m | 涡桨高空功率衰减 |
| T38 | 4200m | 4147.6m | 52m | 教练机推重比低 |
| DHC6 | 4200m | 4164.8m | 35m | 涡桨高空功率衰减 |
| c172r | 1700m | 1663.6m | 36m | 活塞爬升功率不足 |
| Boeing314 | 1700m | 1668.2m | 32m | 大型水上飞机 |
| A4 | 4200m | 4172.1m | 28m | 轻型攻击机 |
| c172p | 1500m | 1512.7m | 13m | 下降阶段 PD 稳态 |
| c182 | — | — | ~2° 航向 | 活塞航向保持稳态 |

### T38/OV10 abort 机制
1. SetAltitude 卡在 52m/69m 偏差 → 无限循环
2. throttle 持续 0.41/0.83 → 燃油消耗
3. T38 ~7000s / OV10 ~3000s 后燃油耗尽
4. 失控下降 → crash（altitude_agl < -5m → kAborted）

### 解决方案：Tiered Best-Effort Convergence
- SetAltitude: 120s 内 10m 精确 → 之后 100m (10×)
- SetHeading: 30s 内 2° 精确 → 之后 10° (5×)
- 完成率 6/17 → 15/17，零回归

### 关键参数：为什么不加积分项
- 积分项可能引起振荡（积分饱和、超调）
- 高空推力不足是物理限制，积分项也无法解决
- best-effort 方案更安全：精确收敛优先，超时才放宽

## 调试方法

- **gtest**：合同/profile/smoke 测试
- **CSV**：`takeoff_land_csv` + `analyze_takeoff.py`
- **Release 预设**：JSBSim ~6× 快于 debug
