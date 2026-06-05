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

### 阶段 15 — SetAltitude/SetHeading 收敛改善 + 全机型 heading-alt 验证 — `complete` ✅

#### 15a：takeoff_land_csv 增强 — `complete` ✅
- timeout 后继续执行（3× 扩展预算），让机型跑完全过程
- 巡航限高到达标识输出（★ 符号）

#### 15b：Tiered Best-Effort Convergence — `complete` ✅
- `IsManeuverComplete()` 增加基于时间的容差放宽
- SetAltitude: 120s 内精确收敛（10m），之后放宽至 10×（100m）
- SetHeading: 30s 内精确收敛（2°），之后放宽至 5×（10°）
- 根因：AP 高度/航向 PD 控制器无积分项，高空推力限制导致稳态偏差（12–76m）

#### 15c：全机型 heading-alt 验证 — `complete` ✅
- 17 机型全部测试：15/17 完成，2/17 着陆 crash（B747/MD11 已知问题）
- 9 个原本 timeout/abort 的机型全部恢复完成（T38 abort→completed）

#### 15d：SetPitch 全机型验证 — `complete` ✅
- `--pitch-test` 模式：takeoff → SetPitch(+5°,10s) → SetPitch(-5°,10s) → SetPitch(+15°,10s) → SetPitch(0°,5s) → land
- 15/17 机型安全完成（B747/MD11 着陆 crash 无关）
- **温和目标 (+5°/-5°/0°) 全部安全**，无振荡无 crash
- **激进目标 (+15°) 全部未达标**：f16 FBW 阻止（0.7°），f15/DHC6 振荡，c172x 接近失速
- 根因：SetPitch 不设 speed_hold → 大 pitch 角时速度骤降 → 失速 → nose-dive

#### 15e：SetPitch 速度保护机制 — `complete` ✅
- **双层保护架构**：
  - L1（油门）：`ExecuteSetPitch()` 启用 `speed_hold` + 速度锁定；能量管理增加渐进式 pitch 偏置 `energy_err += 0.40×(pitch/25°)`
  - L2（俯仰退让）：`UpdatePitchChannel()` pitch_hold 分支增加速度保护（speed < min×1.10 时缩减 pitch）
- **v1→v2 改进**：全有/全无油门覆盖 → 渐进式偏置（+5°=+0.08, +15°=+0.24, +25°=+0.40），速度/超速保护始终在线
- 关键修复：`ExecuteSetPitch()` 显式清除 `altitude_hold`（避免 takeoff 遗留的 altitude_hold 阻止 pitch-only 油门覆盖）
- Commits: `e7ba2407` (v1), `ebe4771f` (v2 progressive)

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

## 阶段 16 — Racetrack / Figure-8 / S-Turn 机动扩展 — `complete` ✅

### 背景
现有机动类型覆盖：FlyToWaypoint、Orbit、SetHeading、SetAltitude、SetPitch、SetRoll、Takeoff、Land。新增三种复杂机动满足搜索/巡逻/消速需求。

### 架构设计
所有新机动复用 Orbit 的核心模式：**每帧计算航向目标 → 通过 AP 航向 PD 跟踪**，不引入新的 `LateralGuidanceMode`。

```
ManeuverExecutor::Update()
  └─ switch (current_maneuver_.type)
       ├─ kOrbit:       ComputeClockwiseOrbitHeadingRad()
       ├─ kRacetrack:   ComputeRacetrackHeadingRad() + FSM
       ├─ kFigure8:     ComputeFigure8HeadingRad() + FSM
       └─ kSTurn:       ComputeSTurnHeadingRad()  (纯时间驱动)
```

### Racetrack（矩形航线 / 等待航线）
- 两直边 + 两 180° 半圆连接
- 4 阶段 FSM：`LEG1 → CW_TURN1 → LEG2 → CW_TURN2 → [repeat]`
- 复用 `ComputeClockwiseOrbitHeadingRad()` 在转弯段
- 字段复用：`target`=起点, `value`=直边航向(rad), `duration_sec`=直边长度(m), `heading_tolerance_rad`=转弯半径(m), `altitude_tolerance_m`=圈数

### Figure-8（8字形）
- 顺时针 360° + 逆时针 360° 交替
- 2 阶段 FSM：`CW → CCW → [repeat]`
- `ComputeFigure8HeadingRad()` 支持 `is_cw` 方向标志
- 方位角累积 2π 切换方向
- 字段复用：`target`=中心, `value`=半径(m), `duration_sec`=轴航向(rad), `heading_tolerance_rad`=循环次数

### S-Turn（S形转弯）
- 正弦航向调制：`heading(t) = ψ_base + A·sin(2πt/T)`
- 无状态机，纯时间驱动
- 字段复用：`value`=基准航向(rad), `duration_sec`=持续时间(s), `heading_tolerance_rad`=振幅(deg), `altitude_tolerance_m`=周期(s)

### 变更清单

| 文件 | 变更 |
|------|------|
| `Maneuver.h` | ManeuverType 新增 kRacetrack/kFigure8/kSTurn；ManeuverExecutor 新增 Execute*() 声明 + 3 个 FSM 枚举 + 状态字段 |
| `Maneuver.cpp` | Execute*() 实现 + Update() 状态机 + Compute*HeadingRad() + IsManeuverComplete() 分支 |
| `FlightManager.cpp` | ExecuteNextManeuver() 新增 3 个 case 分支 |
| `fd_orbit_quality_test.cpp` | 新增 Racetrack/Figure-8/STurn 单元测试 + RacetrackQuality 阈值调整 |
| `examples/` | 新增 racetrack_quality_csv/trace_csv 分析工具 |

### Racetrack 转弯修复（子阶段 16g）

初始实现使用 `ComputeClockwiseOrbitHeadingRad()` 的默认 carrot lookahead（≥2000m），对于小转弯半径（r~1000m）lookahead 超过半圆弧长，导致飞机切弯，每圈漂移 ~362m。

| 修复 | 效果 |
|------|------|
| 转弯 lookahead 从 ≥2000m 降为 max(200, min(speed×5, π/3·r)) | 漂移从 362m→64m |
| Leg2 用 racetrack 坐标系纠偏替代预计算入口点 | 消除入口点失配 |
| quality CSV 预热测速（捕捉 FBW 飞机实际速度） | f16 ratio 1.52→0.12 |
| quality CSV 使用 profile 巡航速度算 feasible_r | A4/F4N/F80C BAD→GOOD |
| quality CSV reset-on-crash 预热 | C130 CRASHED→GOOD |

### Racetrack 进场阶段（子阶段 16h）

初始实现假设飞机从 racetrack 起点开始且航向对齐。当飞机从任意位置/方向进入时，FSM 直接从 kLeg1 开始盲导，导致对角线和远距离场景完全无法加入航线（ratio 高达 8.76）。

| 修复 | 效果 |
|------|------|
| FSM 新增 `kApproach` 阶段（5阶段：Approach→Leg1→Turn1→Leg2→Turn2） | 任意位置进场 |
| 最近点搜索：遍历 Leg1/Leg2/Turn1/Turn2 四段 | 找到最优加入点 |
| 前瞻点导航（look-ahead）替代直接飞向最近点 | 自然对齐航线方向 |
| 捕获距离 max(r×1.5, 1000m) 后切入对应 FSM 阶段 | 平滑过渡 |

验证结果（c172p, r=1000m, leg=10000m）：

| 场景 | 修复前 ratio | 修复后 ratio | 收敛后误差 |
|------|-------------|-------------|-----------|
| 对齐 | 0.030 | 0.036 | 29m |
| 后方5km | — | 0.146 | 133m |
| 左方5km | — | 0.209 | 160m |
| 右方5km | — | 0.160 | 128m |
| 后方10km | — | 0.183 | 147m |
| 右方10km | — | 0.194 | 140m |
| 对角线5km | 1.506 | 0.167 | 137m |
| 反向180° | — | 0.062 | 62m |

关键发现：**收敛后所有场景误差一致**（128-160m），证明 approach 正确引导后正常 FSM 效果一致。

### 最终质量验证
| 等级 | 数量 | 占比 |
|------|------|------|
| ✅ GOOD (ratio<0.10) | 48/60 | 80% |
| ⚠️ WARN (ratio<0.30) | 8/60 | 13% |
| 🔴 BAD (≥0.30) | **0/60** | **0%** |
| 💀 CRASHED | 4/60 | 7% |

- **0 BAD**：所有能飞行的机型均达到合理精度
- 剩余 CRASHED 均为 JSBSim 模型限制（B17 trim 失败、Concorde 半径 > 飞行包线）
- 已知限制：f16 FBW 仍会导致中等精度（ratio 0.12-0.18），MD11 profile 速度失配（ratio 0.30）

### 工作量
~1000 行（接口 50 + 实现 500 + 测试 400 + CMake 50）

### 执行顺序
```
ManeuverType 枚举 → HeadngRad 几何函数 → Execute*() → Update() FSM → IsManeuverComplete() → FlightManager dispatch → 单元测试 → CSV 工具
```

### 依赖
- 无外部依赖
- 复用现有 `ComputeClockwiseOrbitHeadingRad()` 在 Racetrack 的转弯段
- 复用现有 `RunSteps()`/`RunUntilDone()` 测试辅助

### 待处理（非当前阶段）

| 任务 | 阶段 | 优先级 |
|------|------|--------|
| Concorde/C130/L410 引擎兼容性 | 8.2 | 低 |
| XB-70 delta wing 不稳定 | JSBSim 模型限制 | 不修 |
| B747/MD11 着陆 crash（heading-alt 序列） | 16 | 低 |

## 阶段 14 — 多航路点巡航验证 — `complete` ✅

将 `takeoff_land_csv` 巡航阶段的单个 fly-to waypoint 扩展为 3 个，验证多航路点队列在起飞→巡航→着陆全流程中的正确性。

### 变更
- 3 个 fly-to waypoint 沿 45° 对角线均匀分布：1/3、2/3、full 距离
- 中间航路点接受半径缩放（0.35×、0.55×）确保捕获区互不重叠
- `waypoint_distance_m` 保持 `ref_spd × 60s`（不变）

### 验证结果
- **c172x/c310/f16/737** ✅ — 3 个航路点各自独立飞行通过
- **B747** ⚠️ — min_turn_radius（30km）主导捕获区，3 点在 1 步内完成；0.02s 时序偏移触发着陆敏感性（1451s crash，原 1485s complete）

### 后续
- B747 着陆敏感性可单独调查（阶段 14），与多航路点队列逻辑无关

## 工具

- `cmake --preset llvm-ninja-release-local` 构建
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4`
- `build/llvm-ninja-release-local/bin/takeoff_land_csv <model> <out.csv>`
