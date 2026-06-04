# 任务计划：飞行控制阶段修复

## 目标

完成全机型起飞→巡航→着陆自动化飞行。消除硬编码分类，用物理推导替代。

## 背景

分支 `refactor/jsbsim-integration`。16 可用机型全任务通过。

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

### 阶段 13 — 基于物理推导的硬编码消减 — `pending`

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
`cruise_factor = f(T/W, wing_loading)` 替代 5 档硬编码分类。每架飞机独立计算。

#### 13c：rotation_ramp_sec + rotation_climb_rate 连续化 — `pending`
`log10(Iyy)` 线性公式替代 3 档硬阈值。

#### 13d：speed_energy_priority + approach_speed fallback 替代 — `pending`
`wing_loading > 50` → speed_energy_priority；`V_stall × 1.3` → approach fallback。

#### 13e：CLmax / climb_pitch / Vr_factor 加 XML override — `pending`
`guidance/takeoff-cl-max`、`guidance/climb-pitch-deg`、`guidance/takeoff-vr-factor`。

#### 13f：B747 着陆回归修复 — `pending`
B747 crash at 1451s，依赖 git-ignored XML guidance 属性，需排查。

### 执行顺序
```
第一批（低风险）: 13a → 13c → 13d
第二批（核心）:   13b（需全 16 机回归）
第三批（低风险）: 13e
第四批（独立）:   13f
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
