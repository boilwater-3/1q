# 任务计划：飞行控制阶段修复

## 目标

完成全机型起飞→巡航→着陆自动化飞行。修复 CRASH/TIMEOUT 机型。

## 背景

分支 `refactor/jsbsim-integration`，fd_ci 6/6 绿。15/18 可用机型全任务通过。

## 阶段

### 阶段 1–7 ✅ — 初始修复和基线
- 地面弹跳修复、reset XML、横侧控制、旋转渐进、硬编码清理

### 阶段 8 — 剩余 CRASH 修复

#### 8.1 重型机起飞稳定化 ✅ — `commit 06b37c3f`
#### 8.2 引擎/燃油兼容性（Concorde, C130, L410）— `pending`
#### 8.3a：MD11 fly-to/landing ✅ — `commit 5d5f7ac5`
#### 8.3b：非指令升空 + CLmax + DHC6 ✅ — `commit 55c935f8`
#### 8.3c：OV10 重量分类巡航高度 ✅ — `commit 2efd8cb9`
#### 8.4 着陆 crash 自行修复 ✅

### 阶段 9 — 航路点速度与飞行包线 ✅ — `commits cd8900e9 + 8484c90c`

#### 9a：航路点速度 + 速度包线核心 ✅
- `Waypoint.speed_mps` 字段
- `ApplyEnergyDefaults` 6 档分类速度包线
- ExecuteFlyTo/ExecuteOrbit 速度管理
- 能量管理超速保护
- kDecelerate→kApproach AP altitude hold 修复

#### 9b：回归修复 ✅
- **F80C**: fly-past detection + 航路点距离增大 → 1776s completed
- **B747**: 部分改进（bounce recovery, agl<3m touchdown, 进近速度管理）— 仍有进近阶段物理限制

### 阶段 10 — 配置驱动进近重构 ✅

- 10a XML 属性驱动的配置方案（替代硬编码 MOI 阈值）— `completed`
- 10b B747 着陆进近阶段重构（高空减速、盘旋下降、襟翼管理）— `completed`
- 8.2 引擎兼容性（Concorde 燃油 cross-feed、C130 gearratio、L410 cutoff-cmd）

### 阶段 11 — XML 配置契约与硬编码清理规划 ✅

#### 11a：XML 配置契约验证 — `completed`
- 扫描并记录现有 aircraft XML 的 `guidance/*` 用法。
- 建立至少一个非 B747 的 XML override 回归测试，优先使用已有 `c172x` / `global5000` 的 roll limit 配置或补充一个轻量测试 fixture。
- 验证 XML 值优先级：XML > profile 动态默认 > struct fallback。
- 实际落地：新增 `AutopilotPreservesC172xXmlRollGuidanceOverrides`；确认 adapter 保留 `c172x` XML roll limit/rate limit，且 profile 中只对非 FBW、非重型、非 adapter fallback 的 structural roll limit 做 sustained-turn 映射。

#### 11b：flight_dynamic 硬编码分层清单 — `completed`
- 将硬编码分为三类：物理常量/单位换算、通用控制律参数、aircraft capability 参数。
- 只迁移第三类，第二类先命名化并加测试保护，避免把算法常数无边界地塞进 XML。
- 重点清单：`Autopilot.cpp` energy/rotation defaults、`Maneuver.cpp` takeoff/approach/final/flare 参数、`EngineManager.cpp` CLmax/Vr/approach defaults、`takeoff_land_csv.cpp` 场景参数。

#### 11c：配置 schema/文档化 — `completed`
- 定义 `guidance/*` 属性表：单位、默认语义、适用阶段、是否 aircraft-specific。
- 明确 landing orbit 默认开启，XML 仅用于禁用或调整。
- 为新增属性补充测试可读性，避免仅靠 B747 单点验证。
- 实际落地：更新 `docs/finding/jsbsim_aircraft_control_contract.md` 阶段 11 guidance profile 合同表。

#### 11d：分批迁移执行建议 — `completed`
- 第一批：profile 已有字段的契约测试和文档，不改飞行行为。
- 第二批：landing/approach 剩余 aircraft capability 常量迁移。
- 第三批：takeoff/Vr/CLmax 能力参数迁移。
- 第四批：示例程序任务参数抽象，避免 `takeoff_land_csv` 继续承载型号策略。

### 阶段 12 — flight_dynamic 硬编码清理第一批 ✅ — 全部完成

#### 12a Maneuver landing/flare capability ✅
#### 12b Maneuver approach/final 控制律常量命名化 ✅
#### 12c EngineManager 参数边界确认 ✅
#### 12d 示例程序策略外移 ✅
#### 12e 物理推导速度包线 + 升限 clamp ✅

#### 12a：Maneuver landing/flare capability 参数迁移 — `completed`
- 优先清理 `Maneuver.cpp` 中仍按 `log10(Iyy)>7` 或固定重型假设控制 flare/touchdown 的参数。
- 只迁移 aircraft capability：flare 初始电梯、touchdown AGL、final throttle cap、landing flap schedule 已有字段继续复用；若发现缺字段，再先补 `AircraftControlProfile` + XML override + snapshot/contract 测试。
- 验收：B747 `takeoff_land_csv` 仍完成，`1q_fd_tests` 全量通过；非 B747 landing/invalid orbit snapshot 不发生非预期变化。
- 实际落地：新增 `landing_heavy_flare` profile capability 和 XML override `guidance/landing-heavy-flare`；`Maneuver.cpp` flare 分支不再直接读取 `pitch_moi/log10`，B747 XML 显式声明该能力。

#### 12b：Maneuver approach/final 控制律常量命名化 — `completed`
- 对 `1.35/1.15/1.3/1.05` 这类速度窗口、`30m` flare scale、sink-rate 控制参数先命名化并加注释，不急于 XML 化。
- 验收：纯重命名/提取不改变行为；通过 focused landing/fly-to/orbit 测试。
- 实际落地：`Maneuver.cpp` landing 段新增命名常量，覆盖 decelerate/approach/final/flare/rollout 速度窗口、FPA、sink-rate throttle、flare scale 和 bounce/float recovery 阈值；未改公式和行为。

#### 12c：EngineManager aircraft capability 参数边界确认 — `completed`
- 梳理 CLmax、delta-wing AR 阈值、Vr factor、approach speed、climb pitch 的输入来源。
- 先补 contract 测试，确认哪些是模型能力参数，哪些是控制律 fallback；避免为了”去硬编码”把物理估算常量移进 XML。
- 实际落地：`EngineManager.cpp` 新增 20+ 命名常量；`spdlog::debug` 诊断日志 + `spdlog::warn` 边界告警；`EngineManagerContractTest` 12 个参数化测试（4 机型 × 3 方法）；`FlightManager.h` 新增 `GetEngineManager()` 访问器。
- 分类结论：CLmax/Vr factor/climb pitch = aircraft capability（可迁移 profile，保持 engine type fallback）；AR 阈值/Iyy 阈值/Vr floor = 通用分类常量（仅命名化）；Approach speed = last-resort fallback（保留）。
- 验证：build 通过，`EngineManager*` 12/12 passed，回归 11/11 passed，full 171/3 passed/skipped，B747 端到端起降无回归。

#### 12d：示例程序策略外移规划 — `completed`
- `takeoff_land_csv.cpp` 的 cruise altitude、waypoint distance、landing target、skip/model 分支属于场景策略，不应混入核心 `flight_dynamic`。
- 下一批只形成清单和小型结构化配置入口，不在阶段 12a/12b 中同时改。
- 实际落地：
  - 新增 `ScenarioConfig` 结构体（cruise_altitude_m, waypoint_distance_m, landing_lat/lon_rad, max_steps）和 `MakeScenario()` 工厂函数——零型号名硬编码，全部通过 profile 布尔值推导。
  - `CheckSkip()` 改用属性树检测（bw-ft < 1.0 → 多旋翼），替代型号名字符串匹配。
  - Landing target 改为独立的场景位置参数（可以是同机场或不同机场），不再从航路点几何推导。
  - 巡航高度从 profile 撤回，保留在示例程序的场景工厂中——确认其为场景参数而非飞机能力。

#### 12e：飞机升限（service ceiling）— `completed`
- 问题：所有机动（takeoff, fly-to, orbit）的目标高度可能超过飞机实际升限 → 无限爬升 → TIMEOUT。
- 方案：profile 新增 `ceiling_m`，按 6 档分类设默认值（FBW 15200m, 重型涡扇 13700m, 中型涡扇 12500m, 涡桨/轻涡 7600m, 活塞 4300m），支持 XML `guidance/ceiling-m` 覆盖。
- `ExecuteTakeoff`、`ExecuteFlyTo`、`ExecuteOrbit` 三入口统一 clamp 目标高度 ≤ ceiling_m。

### 阶段 13 — 基于物理推导的硬编码消减 — `pending`

基于 `docs/finding/hardcoded_parameter_audit.md` 调研结论。核心原则：
- **A 类**（直接读取）— JSBSim 属性树已有 → 直接读
- **B 类**（间接推导）— 可从 weight/wing_area/Iyy/engine_type/thrust 物理推导 → 连续公式替代硬编码分类
- **C 类**（控制律常量）— 通用参数，非 aircraft-specific → 保持命名常量
- **D 类**（XML override）— aircraft capability → 加 profile 字段 + XML 覆盖路径

#### 13a：获取引擎推力数据（推重比）— `pending`

**目标**：在 Autopilot 初始化时获取每架飞机的实际推重比，替代当前"FBW/重型/中型/活塞"的粗粒度分类。

**方案**：JSBSim 引擎 XML 中包含 `<milthrust>`（喷气推力 lbs）和 `<maxhp>`（活塞马力），但这些值不通过属性树暴露。

**方案 A（推荐）**：运行时测量。在 `JsbsimAdapter` 初始化完成后、`FlightManager` 首次 `Step()` 前，短暂运行若干帧（throttle=1, brakes=on, 地面静态），读取 `propulsion/engine[n]/thrust-lbs` 累计值作为 max_static_thrust。推重比 = total_thrust / weight。

**方案 B**：解析引擎 XML。实现轻量 XML 解析器读取 `<milthrust>` / `<maxhp>` 标签。风险是各机型 XML 格式不统一。

**验收**：
- 16 机型均能获取到非零推重比
- 推重比与机型实际性能一致（f16≈1.6, B747≈0.44, c172x≈0.11 hp/lb）
- 不影响现有 `1q_fd_tests` 和 `takeoff_land_csv` 结果

#### 13b：推重比 + 翼载 → 连续化速度包线 — `pending`

**目标**：用连续公式替代当前 5 档硬编码的 `stall_margin` / `cruise_factor` / `max_factor`。

**当前问题**：
```
活塞 ×2.8, 涡桨/轻涡 ×3.0, 中型涡扇 ×3.2, 重型涡扇 ×3.5, FBW ×3.5
```
所有活塞都一样，所有重型涡扇都一样——忽略了同类别内的性能差异。

**方案**：
```
cruise_factor = base + kTW * (T/W) + kWL * (wing_loading / 100)
max_factor    = cruise_factor * (1.2 ~ 1.5, 取决于引擎类型)

其中 base, kTW, kWL 由引擎类型决定：
  活塞:     base=2.3, kTW=4.0,  kWL=0.3
  涡桨:     base=2.5, kTW=3.0,  kWL=0.4
  涡扇(非FBW): base=2.8, kTW=1.5, kWL=0.5
  涡扇(FBW):   base=3.0, kTW=1.0, kWL=0.6
```

**预期效果**：
| 机型 | 当前 cruise | 推导 cruise | 推重比 | 翼载 |
|------|------------|------------|--------|------|
| c172x | 72 m/s | ~68 m/s | 0.11 | 14 |
| c310 | 98 m/s | ~95 m/s | 0.18 | 31 |
| 737 | ~232 m/s | ~240 m/s | 0.48 | 111 |
| B747 | 201 m/s | ~220 m/s | 0.44 | 106 |
| f16 | 203 m/s | ~280 m/s | 1.67 | 83 |

**验收**：16 机型 `takeoff_land_csv` 全部完成，与旧包线结果无 crash 回归。

#### 13c：rotation_ramp_sec + rotation_climb_rate 连续化 — `pending`

**目标**：用 `log10(Iyy)` 线性公式替代 3 档硬阈值。

**当前**：
```
if log10(Iyy) > 7.0: ramp=6.0s, climb=3.0m/s
if log10(Iyy) > 6.0: ramp=4.0s, climb=4.0m/s
else:                ramp=3.0s, climb=5.0m/s
```

**方案**：
```
ramp_sec = clamp(1.5 + 0.6 * log10(Iyy), 3.0, 6.0)
climb_mps = clamp(7.0 - 0.5 * log10(Iyy), 3.0, 5.0)
```

**预期效果**：
| 机型 | log10(Iyy) | 旧 ramp | 新 ramp | 旧 climb | 新 climb |
|------|-----------|---------|---------|---------|---------|
| c172x | 3.13 | 3.0s | 3.4s | 5.0 | 5.0 |
| 737 | 6.18 | 4.0s | 5.2s | 4.0 | 3.9 |
| B747 | 7.52 | 6.0s | 6.0s | 3.0 | 3.2 |
| f16 | 4.75 | 3.0s | 4.4s | 5.0 | 4.6 |

**验收**：同 13b。

#### 13d：speed_energy_priority + approach_speed fallback 替代 — `pending`

- `speed_energy_priority`：用 wing_loading > 50 lbs/ft² 替代 5 档布尔值。高翼载飞机动能大，速度保护更重要。
- `approach_speed fallback`：统一用 `V_stall × 1.3` 替代引擎类型表。当前极少触发（profile 优先 → Vr×1.3 优先 → 才到这里）。

**验收**：同 13b。

#### 13e：CLmax / climb_pitch / Vr_factor 加 XML override — `pending`

- `guidance/takeoff-cl-max` → 覆盖 CLmax（默认 1.6/2.0/2.5 auto-detect）
- `guidance/takeoff-vr-factor` → 覆盖 Vr factor（默认按引擎+Iyy 推导）
- `guidance/climb-pitch-deg` → 覆盖爬升俯仰角（默认按引擎类型）
- 补充 snapshot 测试 + contract 文档

**验收**：新 XML 属性在 B747（或专用 fixture XML）上通过测试验证优先级。

#### 13f：B747 着陆回归修复 — `pending`

B747 当前 crash at 1451s。依赖 git-ignored B747.xml guidance 属性。需要排查 guidance 属性是否正确加载，并确保 B747 端到端通过。

### 批次执行顺序

```
第一批（13a+13c+13d）：低风险，不影响飞行行为或影响可预测
第二批（13b）：中风险，速度包线核心变动，需全 16 机回归
第三批（13e）：低风险，加 XML override 路径
第四批（13f）：B747 修复，独立于前三批
```

## 工具

- `cmake --preset llvm-ninja-release-local` 构建
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4`
- `build/llvm-ninja-release-local/bin/takeoff_land_csv <model> <out.csv>`
- `python3 tools/analyze_takeoff.py [--plot] <csv...>`
