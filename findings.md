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

## JSBSim 控制接口分析（阶段 15，2026-06-05）

### SetRoll — 模式开关（非目标角度）
- `SetRoll(0)` = wings level：PD 控制器驱动 aileron 归零 roll 角
- `SetRoll(1)` = heading-based roll：roll 角正比于航向误差
- **不是"设坡度到 X°"**，JSBSim 中没有对应的通用概念
- 所有 17 机型 heading-alt 测试已间接覆盖，不需要单独测试

### SetPitch — 固定俯仰角目标（有风险）
- 使用与 altitude hold 相同的 PD 控制器：`elevator = -(2.0·err - 0.2·q)`
- **JSBSim 没有通用高层 pitch 目标 API**（仅 X15 有 `ap/pitch-target-deg`）
- 我们通过 `fcs/elevator-cmd-norm`（或 FBW 的 `fcs/pitch-trim-cmd-norm`）直接驱动

### 控制接口分类（17 机型）

| 接口类型 | 机型 | 特征 |
|---------|------|------|
| direct surface | c172p/r, c182, c310, 737, B747, MD11, f15, A4, F4N, T38, DHC6, OV10, F80C, Boeing314 | `elevator-cmd-norm` → `elevator-pos-rad` |
| native AP bridge | c172x, c310, global5000 | XML AP `ap/attitude_hold` → wing leveler PID |
| FBW rate command | **f16** | `aileron-cmd-norm` → roll-rate PID → `aileron-pos-rad` |
| FBW pitch | **f16** | `pitch-trim-cmd-norm` → elevator-cmd-limiter → alpha scheduler → `elevator-pos-rad` |

### f16 FBW 保护机制（影响 SetPitch）
- **elevator-cmd-limiter**：clip [-1, 0.44]， Nose-down 权限大于 nose-up
- **alpha limiter**：高迎角时 elevator 权限降到 0.11（几乎无效）
- **elevator scheduler**：按 AoA 调度 elevator 增益（0°→1.0，±30°→0.0）
- **SetPitch(20°)** 在高迎角时会被 FBW 限制住，物理上到不了目标

### 17 机型 elevator/aileron 物理限制

| 机型 | elevator 物理范围 | aileron 物理范围 | 备注 |
|------|:-:|:-:|------|
| c310 | ±25/35° | — | — |
| 737 | ±0.3 rad | — | — |
| OV10 | ±0.35 rad | — | — |
| F80C | ±0.35 rad | — | — |
| f16 | clip [-1, 0.44] + alpha 调度 | ±0.375 rad | FBW 保护 |
| 其余 | JSBSim 默认 | JSBSim 默认 | — |

### SetPitch 风险评估
- PD 增益固定（2.0/0.2），对 Iyy 差 4 数量级的机型可能过激或不足
- altitude hold 已验证（heading-alt 测试），但 SetPitch 目标是固定角度而非高度
- 高俯仰角目标（>15°）可能在某些机型上引起振荡或失速
- **需要全机型验证**，特别是 f16（FBW 限制）和 B747（高 Iyy）

### SetPitch 全机型验证结果（15d，2026-06-05）

使用 `--pitch-test` 模式：takeoff → SetPitch(+5°,10s) → SetPitch(-5°,10s) → SetPitch(+15°,10s) → SetPitch(0°,5s) → land

#### 温和目标 (+5°/-5°/0°)：15/17 安全
所有机型在 ±5° pitch 目标下稳定响应，无 crash，无振荡。

#### 激进目标 (+15°)：全部机型未达标

| 机型 | +15° pitch_max | 实际行为 |
|------|:-:|------|
| f16 | 0.7° | FBW alpha 保护几乎完全阻止 pitch-up |
| f15 | 2.0° | 先升后降，振荡→nose-dive 至 -22° |
| A4 | 0.9° | 高速下 pitch 响应极慢 |
| F4N | 1.3° | 同 A4 |
| T38 | 3.0° | 缓慢上升后回落 |
| B747 | 3.8° | 上升后反转负 pitch，不稳定 |
| c172x | 7.3° | 最接近目标，但速度从 67→52 kts（接近失速）|
| DHC6 | 9.0° | 先升后降，pitch 反转至 -10.7° |
| 737 | 7.4° | 温和响应，未达标 |

#### 根因
- `ExecuteSetPitch()` 只设 `pitch_hold=true`，不设 `speed_hold`
- `UpdateEnergyManagement()` 在 `altitude_hold=false && speed_hold=false` 时退出
- **速度无保护**：大 pitch 角 → 阻力增大 → 速度骤降 → 升力不足 → 失速 → nose-dive
- f16 的 FBW alpha limiter 反而是保护机制（阻止了失速）

#### 结论
- SetPitch 限于 ±5° 安全范围
- +15° 需要：速度保护 + pitch 角限幅 + 更长的持续时间
- 当前 PD 控制器增益对温和目标足够，激进目标需要额外安全机制
- 积分项可能引起振荡（积分饱和、超调）
- 高空推力不足是物理限制，积分项也无法解决
- best-effort 方案更安全：精确收敛优先，超时才放宽

### SetPitch 速度保护机制（阶段 15e，2026-06-05）

#### 根因分析
`ExecuteSetPitch()` 只设 `pitch_hold`，不设 `speed_hold`。`UpdateEnergyManagement():755` 守卫条件 `if (!altitude_hold_ && !speed_hold_) return;` 导致 pitch_hold 期间能量管理完全不执行。大 pitch 角 → 阻力增大 → 无油门补偿 → 速度衰减 → 失速。

额外发现：`ExecuteSetPitch()` 未清除 takeoff 遗留的 `altitude_hold`，导致后续 pitch-only 油门覆盖条件 `pitch_hold_ && !altitude_hold_` 不成立。

#### 双层保护架构
- **L1（油门）**：能量管理增加渐进式 pitch 偏置
  - 守卫条件增加 `pitch_hold_`
  - `energy_err += 0.40 × (pitch_target / 25°)` — 与 pitch 角度成正比
  - +5°→+0.08, +15°→+0.24, +25°→+0.40
  - 不 return，速度/超速保护始终在线
- **L2（俯仰退让）**：`UpdatePitchChannel()` pitch_hold 分支
  - speed < min_speed × 1.10 时线性缩减 pitch 目标
  - 均衡点 = `thrust_max ⇌ drag` = 机型物理 pitch 上限

#### v1→v2 改进关键
v1 使用全有/全无油门覆盖（pitch>0→thr=1.0, early return），被指出"偏激"——+5°也全油门，速度/超速保护被绕过。

v2 改为渐进式偏置融入现有能量公式，效果：
| pitch 目标 | v1 throttle | v2 throttle | 评价 |
|-----------|------------|------------|------|
| +5° | 1.00 | 0.80 | v2 更温和 |
| +15° | 1.00 | 0.95→1.00 | 几乎持平 |
| +25° | 1.00 | 1.00 | 持平 |
| -5° | 0.15(min) | 0.62 | v2 更合理 |

#### 改善数据（全部 17 机型 vs 15d 初始测试）
| 机型 | 15d +15° | 15e +15° | 15e +25° | 改善倍数 |
|------|---------|----------|----------|---------|
| f16 | 0.69° | 15.3° | 25.1° ✓ | **22×** |
| F4N | 1.3° | 15.4° | 25.3° ✓ | **12×** |
| f15 | 2.0° | 14.5° | 24.2° | 7× |
| c172x | 7.3°(失速) | 20.7° | 14.7° | 2.8× |
| A4 | 0.9° | 14.0° | 23.7° | 15× |
| T38 | 3.0° | 12.7° | 22.5° | 4.2× |
| DHC6 | 9.0°(振荡) | 16.0° | 20.7° | 1.8× |
| B747 | 3.8° | 10.9° | 19.0° | 2.9× |

#### 关键认知
- **"物理上限"有两种**：瞬间气动极限（elevator 能拉到的 max pitch）vs 推力可持续极限（max thrust 能维持的稳态 pitch）。c172x 瞬间 20.7° 但稳态只有 8-12°。
- **FBW 不是障碍**：f16 从 0.69° 跃升到 15.3° 证明之前的限制是能量不足而非 FBW 锁定。满油门 + 高速度下 AoA 低于 alpha limiter 阈值。
- **渐进式 > 全有/全无**：小 pitch 不浪费油门，大 pitch 渐进满推力，速度/超速保护始终在线。
- **Clear altitude_hold 是关键 bugfix**：takeoff 后 altitude_hold 仍为 true，导致 pitch-only 油门覆盖不触达。这个问题隐藏了多轮测试。

## 新机动行为架构（阶段 16，2026-06-05）

### 核心设计原则
所有复杂机动（Racetrack、Figure-8、S-Turn）复用 Orbit 的核心模式：
**每帧计算航向目标 → 通过 AP 航向 PD 跟踪**，不引入新的 `LateralGuidanceMode`。

```
ManeuverExecutor::Update()
  └─ switch (current_maneuver_.type)
       ├─ kOrbit:       ComputeClockwiseOrbitHeadingRad()
       ├─ kRacetrack:   ComputeRacetrackHeadingRad() + FSM
       ├─ kFigure8:     ComputeFigure8HeadingRad() + FSM
       └─ kSTurn:       ComputeSTurnHeadingRad()  (纯时间驱动)
```

### Racetrack 四阶段 FSM
```
LEG1 ──(直边长度到达)──→ CW_TURN1 ──(180° 转弯完成)──→ LEG2 ──(直边长度到达)──→ CW_TURN2 ──(180° 转弯完成)──→ LEG1 [repeat]
```
- 转弯段复用 `ComputeClockwiseOrbitHeadingRad()`，圆心位于直边末端的垂直偏移
- 每次环绕回到 LEG1 时 `lap++`，达到目标圈数后 `kComplete`
- **关键几何**：圆心1 = 起点偏移 r×90°(朝向 ψ+90°)，圆心2 = 起点+L 沿 ψ + 偏移 r×(-90°)

### Figure-8 双阶段 FSM
```
CW(360°) ──(方位角累积 2π)──→ CCW(360°) ──(方位角累积 -2π)──→ CW [cycle++]
```
- 与 Orbit 的 `ComputeClockwiseOrbitHeadingRad()` 共享几何
- 增加 `is_cw` 参数：CW 用 `radial_angle + π/2`，CCW 用 `radial_angle - π/2`
- 截距修正 `atan2(radial_error, radius_m)` 在两种方向下都有效

### S-Turn 纯时间驱动
```cpp
heading(t) = ψ_base + A·sin(2πt/T)
```
- 无状态机，AP 持续跟踪正弦航向
- `IsManeuverComplete()` = `elapsed_sec >= duration_sec`

### 字段复用约定（ManeuverCommand 无新增字段）

| 机动 | target | value | duration_sec | heading_tolerance_rad | altitude_tolerance_m |
|------|--------|-------|-------------|----------------------|----------------------|
| Racetrack | 起点(lat/lon/alt) | 直边航向(rad) | 直边长度(m) | 转弯半径(m) | 圈数 |
| Figure-8 | 中心(lat/lon/alt) | 半径(m) | 轴航向(rad) | 循环次数 | — |
| STurn | — | 基准航向(rad) | 持续时间(s) | 振幅(deg) | 周期(s) |

### 与 Orbit 实现的对比分析

| 特性 | Orbit | Racetrack | Figure-8 | S-Turn |
|------|-------|-----------|----------|--------|
| 航向计算 | 圆切线+截距修正 | 直边固定+半圆切线 | 交替切线方向 | 正弦调制 |
| 状态机 | 无（单圆） | 4 阶段 | 2 阶段 | 无 |
| 复用 ComputeClockwiseOrbitHeadingRad | — | ✅ 转弯段 | ⚠️ 带 is_cw 参数 | ❌ |
| 完成判定 | duration_sec | lap数+phase | cycle数+phase | elapsed_sec |
| 几何复杂度 | 低 | 高（4段衔接） | 中（方向切换） | 低（连续函数） |

### 测试策略
- **单元测试**：每种机动独立验证几何收敛性、阶段切换正确性、完成判定
- **长时间测试**：Racetrack 10 圈验证无累积误差，Figure-8 10 循环验证交替稳定性
- **边界测试**：极小转弯半径（clamped）、零振幅 S-Turn（= 直飞）、零圈数 Racetrack（立即完成）
- **队列测试**：Racetrack → Orbit → SetHeading 多级切换

## Racetrack 转弯精度分析（阶段 16g，2026-06-05）

### 问题描述
Racetrack 转弯使用 `ComputeClockwiseOrbitHeadingRad()` 的默认 lookahead（`max(2000, speed×10)`），对于小转弯半径（r≈1000m）导致 carrot 落在 121° 前方——半圆弧才 180°。

### 切弯机制
- **Carrot 角**：law of cosines 计算 `theta = acos((d²+r²-l²)/(2dr))`
- 对于 d≈r, l=2000, r=1147：`cos_theta = 2r²-4×10⁶/2r² ≈ -0.52` → `theta = 121°`
- 飞机始终瞄准 121° 前的点 → 切弯 → 沿内圈飞行 → Leg2 起飞点偏离

### 修正方案
```cpp
// 老：lookahead = max(2000, speed×10)       → 121° 超前（r=1147）
// 新：lookahead = max(200, min(speed×5, π/3·r))  → 60° 超前
```
- π/3·r ≈ 60° 弧长，半圆弧 180°，carrot 不超出圆弧端点
- `speed×5` 作为高速保底（vs 原来的 `speed×10`）

### Leg2 交叉修正
老方案用 `CrossTrackDistanceM(loc, leg2_entry, heading_π)` 从预计算入口测横向偏差。但 Turn1 结束时机位置 ≠ 预计算入口点：

```cpp
// 老：x_track = CrossTrackDistanceM(loc, leg2_entry, π)  → 错位
// 新：x_track = x_prime - 2r                            → 精确
```

### Speed 失配模式

| 模式 | 原因 | 修复 |
|------|------|------|
| Profile cruise ≠ actual | FBW 飞机不尊崇 AP speed hold | 预热 2000 步测峰值速度 |
| feasible_r 用 spec speed 计算 | spec=200 但实际 311 m/s | 用实测速度 vtrue 计算 |
| ExecuteRacetrack 速度 clamp 失效 | FBW 覆盖 AP speed target | 无代码修复，仅测试端适配 |

### 验证结果
- 60 次测试，**0 BAD**，48/60 (80%) GOOD
- 剩余 WARN/CRASHED 均为模型限制（B17/FBW/Concorde）

## Racetrack 进场阶段分析（阶段 16h，2026-06-05）

### 问题描述
初始 Racetrack FSM 从 `kLeg1` 开始，假设飞机在起点位置且航向对齐。当飞机从任意位置/方向进入时：
- 后方5km：需飞额外距离，误差 5× 恶化
- 侧方5km：45° 截角限制收敛极慢
- 对角线5km：ratio > 1.0（BAD），飞机永远无法加入航线
- 远距15km：ratio > 8.0，完全不收敛

### 根因
FSM 无进场阶段，`kLeg1` 的引导逻辑（航向 ψ + 横距修正）只在飞机靠近 Leg1 轨迹时有效。远距离时：
1. 航向目标固定为 ψ，不管飞机朝哪
2. 横距修正被 ±45° clamp 限制，收敛极慢
3. `y_prime >= leg_len` 阶段转换可能在错误位置触发

### 解决方案：kApproach 阶段

FSM 从 4 阶段扩展为 5 阶段：`kApproach → kLeg1 → kTurn1 → kLeg2 → kTurn2`

核心算法：
1. **最近点搜索**：遍历 racetrack 四段几何体（Leg1/Leg2/Turn1/Turn2），找到距飞机最近的跑道巡逻航线上的点
2. **前瞻点导航**：从最近点沿航线前进方向前进 `max(1000, speed×5)` 米，飞向前瞻点而非最近点
   - Leg1: 沿 +y 方向前进
   - Leg2: 沿 -y 方向前进
   - Turn: 沿 CW 方向在圆弧上前进 `la/r` 弧度
3. **前瞻导航的作用**：飞机自然从航线后方接近，到达时航向已对齐
4. **捕获条件**：`best_dist <= max(r×1.5, 1000m)` 时切入对应 FSM 阶段

### 关键发现

#### 前瞻 vs 直接飞向最近点
直接飞向最近点时，飞机到达最近点航向可能不对（如从东侧到达 Leg2 但朝西飞，而 Leg2 航向朝南）。前瞻点让飞机瞄准航线前进方向的某个点，到达时航向自然对齐。

#### 收敛一致性
所有场景（对齐/后方/侧方/对角线/远距）收敛后平均误差均为 128-160m（r=1000m），方差极小。证明 approach 正确引导后，正常 FSM 接管效果一致。

#### 零成本
对齐场景下 approach 阶段立即过渡（dist=0），不增加任何延迟。

#### JSBSim Trim 限制
非北向初始航向导致 JSBSim `DoTrim(0)` 失败。这是 JSBSim 的已知限制——trim 假设北向飞行。解决方法：始终北向配平，用 SetHeading 转向后再开始机动。

### 诊断工具
- `racetrack_approach_diag`：8 场景进场质量测试（位置偏移 + 航向偏移）
- `racetrack_approach_trace`：进场轨迹 CSV 导出（x_prime/y_prime 坐标）
- `tools/plot_racetrack_approach.py`：进场轨迹可视化（全局 + 航线区域放大双视图）

## 调试方法

- **gtest**：合同/profile/smoke 测试
- **CSV**：`takeoff_land_csv` + `analyze_takeoff.py`
- **Release 预设**：JSBSim ~6× 快于 debug
