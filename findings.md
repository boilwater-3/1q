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

当前 crash at 1451s。B747.xml 是 git-ignored 文件，guidance 属性是本地修改。需阶段 13f 排查。

## 调试方法

- **gtest**：合同/profile/smoke 测试
- **CSV**：`takeoff_land_csv` + `analyze_takeoff.py`
- **Release 预设**：JSBSim ~6× 快于 debug
