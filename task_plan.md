# 任务计划：飞行控制阶段修复

## 目标

完成全机型起飞→巡航→着陆自动化飞行。消除硬编码分类，用物理推导替代。

## 背景

分支 `refactor/jsbsim-integration`。17 可用机型（含 B747）全任务通过。

## 已完成阶段

### 阶段 1–7 — 初始修复 ✅
地面弹跳、reset XML、横侧控制、旋转渐进、硬编码清理

### 阶段 8 — CRASH 修复 ✅
8.1 重型机起飞、8.3a MD11 fly-to/landing、8.3b 非指令升空+CLmax+DHC6、8.3c OV10 巡航高度

### 阶段 9 — 航路点速度与飞行包线 ✅
Waypoint.speed_mps、速度包线 6 档分类、能量管理超速保护、F80C fly-past detection

### 阶段 10 — 配置驱动进近重构 ✅
XML 属性驱动方案、B747 着陆进近重构

### 阶段 11 — XML 配置契约 ✅
非 B747 XML override 测试、guidance 属性表、硬编码分层清单

### 阶段 12 — 硬编码清理第一批 ✅
12a: landing/flare capability 迁移到 profile
12b: landing 控制律常量命名化
12c: EngineManager 参数边界确认 + contract 测试
12d: 示例程序策略外移（ScenarioConfig）
12e: 物理推导速度包线 + 升限 clamp + MD11 修复

## 当前阶段

### 阶段 13 — 基于物理推导的硬编码消减 — `complete` ✅

依据：`docs/finding/hardcoded_parameter_audit.md`（全 31 机型 JSBSim XML 审计）

核心发现：
- JSBSim 属性树只有 4 个物理属性可直接读取：weight、wing_area、wingspan、Iyy
- 引擎推力/功率在 XML 中存在但不在属性树中
- 31 机型全无 alpha-max-rad 和 vne-kts
- 当前速度包线仍用 5 档分类乘数，所有同类别飞机一样

#### 13a：获取引擎推力数据（推重比）— `complete` ✅
非侵入式方案：从当前运行状态（thrust-lbs / power-hp）线性缩放估算额定推力。
低 throttle 时用翼载分类 fallback。发布到 `guidance/thrust-to-weight` 属性。
FlightManager 构造顺序：EngineManager → Autopilot（确保 TWR 属性可用）。

#### 13b：推重比 + 翼载 → 连续化速度包线 — `complete` ✅
cruise_factor 从翼载连续计算，替代 5 档离散分类：
- 活塞：CF = 2.8（不变，WL 范围窄）
- 非活塞：CF = 2.89 + 0.00455 × WL（线性拟合 17 机型数据）
- FBW 补偿：+0.25
max_factor 从 cruise_factor + 类别 delta 推导，安全参数保持离散分类。
副作用：B747 着陆从 abort(1451s) 变为 completed(1485s)。

#### 13c：rotation_ramp_sec + rotation_climb_rate 连续化 — `skipped` ⏭️
`log10(Iyy)` 线性公式替代 3 档硬阈值。**跳过原因**：升降舵力矩 M_elevator 不在 JSBSim 属性树中，Iyy 是唯一可用的物理量，任何连续公式都只是曲线拟合而非物理推导。硬阈值工作良好。

#### 13d：speed_energy_priority + approach_speed fallback — `complete` ✅
- `speed_energy_priority`: `wing_loading > 50` → true，替代 5 类别布尔值。
  物理基础：KE ∝ WL × W/CLmax，高翼载→高动能→速度恢复更难→优先保护速度。
  f15(WL=55) 获优先级，c310(WL=22)/C130(WL=31)/B17(WL=25) 放弃优先级。
- `GetDefaultApproachSpeedMps()`: `V_stall × 1.3` 替代引擎类型查找表。
  正常路径走 `Vr × 1.3`，此 fallback 是死代码但语义更物理。

#### 13e：CLmax / climb_pitch / Vr_factor 加 XML override — `complete` ✅
新增 3 个可选 guidance 属性（不写 XML 时行为不变）：
- `guidance/takeoff-cl-max` → 覆盖 CLmax
- `guidance/climb-pitch-deg` → 覆盖爬升俯仰角
- `guidance/takeoff-vr-factor` → 覆盖 Vr 安全系数

#### 13f：B747 着陆回归修复 — `complete` ✅
**13b 副作用解决**：连续化 cruise_factor 使 B747 进近速度管理平滑化，着陆从 abort(1451s) 变为 completed(1485s)。无需独立修复。

### 变更总结

消除的硬编码分类：
- `cruise_factor`: 5 档离散表 → `2.89 + 0.00455 × WL` 连续函数
- `speed_energy_priority`: 5 类布尔值 → `WL > 50 || FBW`
- `approach_speed fallback`: 引擎类型表 → `V_stall × 1.3`
- `CLmax / Vr_factor / climb_pitch`: 加 XML override 路径

保留不变的 C 类参数（控制律常量，非 aircraft-specific）：
- kLanding* 常量、Vr floor(40 kts)、AR 阈值(2.5)、rotation 阈值(log10>6/7)

### 执行顺序
```
13a（推重比估算）→ 13b（翼载连续化速度包线）→ 13d（spd_prio+approach）→ 13e（XML override）
13c 跳过（无物理推导路径）
13f 已被 13b 副作用解决
```

### 待处理（非当前阶段）

| 任务 | 阶段 | 优先级 |
|------|------|--------|
| Concorde/C130/L410 引擎兼容性 | 8.2 | 低 |
| XB-70 delta wing 不稳定 | JSBSim 模型限制 | 不修 |

## 工具

- `cmake --preset llvm-ninja-release-local` 构建
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4`
- `build/llvm-ninja-release-local/bin/takeoff_land_csv <model> <out.csv>`
