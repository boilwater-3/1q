# 任务计划：起飞控制器修复（CRASH 机型）

## 目标

修复 7 个 CRASH 机型（L410, OV10, F450, B747, MD11, XB-70, T38）的起飞/着陆问题。

## 背景

分支 `refactor/jsbsim-integration`，fd_ci 3/3 绿。16/20 机型全任务通过（含 OV10），F450 跳过（多旋翼）。

## 根因分析（最终确认）

1. **RunIC 地面弹跳**：`SuspendIntegration()` → dt=0 → 地面力不积分 → 首个 Run(dt>0) 弹跳
   - 涡桨（L410, OV10）静态拉力极大 → 初始 vc 20.8 kts → 弹跳 + 无控翻转
   - 涡扇（737, f16）静态拉力低 → 自然阻尼 → 不受影响
2. **reset XML 高度**：每个机型的 reset00.xml 定义了正确 AGL → 加载后 gear 在跑道面
3. **阶跃 elevator**：Vr 时 elevator 0→-0.3 阶跃 → 大推重比机型（B747）产生灾难性滚转
4. **AP lateral detector 误判**：`ap/autopilot-roll-on` 从共享 Autopilot.xml 泄漏 → 误选 `kGenericAutopilotBridge` → 无副翼控制

## 阶段

### 阶段 1 ✅ — 初始速度修复
- velocity=0 for ground start
- 结果：RunIC 后 UVW=0 正确，但首个 Run() 仍弹跳（非速度问题，是力平衡问题）

### 阶段 2 ✅ — 滑跑横侧控制
- StartEngine: roll_mode=0, roll_ap_on=true（wings-level during ground roll）

### 阶段 3 ✅ — 渐进式旋转
- elevator 3s ramp: `el = -0.3 * min(elapsed/3.0, 1.0)`
- 去掉 ConfigureForClimb 中的阶跃 elevator

### 阶段 4 ✅ — F450 跳过
- 标记为 multirotor → skipped

### 阶段 5 ✅ — 地面弹跳修复
- 加载 reset00.xml → 正确 AGL（gear 在跑道面）
- SettleInitialGroundState（HoldDown 5 帧 + 刹车 + idle 油门）
- VehicleStateMapper：altitude=0 时保留 XML 值
- InitRunning throttle reset + 涡桨 prop-advance 设置

### 阶段 6 ✅ — F4N 回归修复
- 根因：reset AGL 1.14m 暴露了 AP kOwnAutopilot 缺少直接 lateral 控制
- 修复：kOwnAutopilot 路径增加 UpdateDirectHeadingLateral 安全网
- 辅助：altitude=0 override（reset 高度导致 nose-only 接地）

### 阶段 7 ✅ — 硬编码清理
- 删除 Tier 1 显式 profile 表（7 个机型硬编码 → 0 个）
- lateral_interface 全动态检测：FBW > OwnAP > GenericAP > DirectSurface
- OV10 guard：单属性泄漏不配 kGenericAutopilotBridge
- indexed_throttle：`engine_count > 1` 动态
- 能量管理：按属性分类（FBW/发数/mixture）替代机型名字符串
- 删除 F4N 硬编码 altitude override

## 🆕 阶段 8 — 剩余 CRASH 修复

### 8.1 重型机起飞稳定化 ✅ — `commit 06b37c3f`
- 三个根因修复：
  1. **地面滚转发散**：延迟 heading hold 到离地后（WOW 清除 + AGL > 5m）
  2. **Elevator 阶跃**：rotation ramp 持续到完成（不再在 AGL=10m 中断），Iyy 分档（重型 6s / 中型 4s / 轻型 3s）
  3. **爬升振荡**：用 AP pitch hold（PD + pitch rate 阻尼）替代 climb rate P 控制器
- Vr 按 Iyy 分级：重型 1.08×V_stall、中型 1.15×、轻型 1.20×
- 结果：B747 完整完成 takeoff→fly→land（985s），MD11 起飞成功爬升到 10346m
- 文件：`Maneuver.cpp`, `Autopilot.cpp`, `Autopilot.h`, `EngineManager.cpp`

### 8.2 引擎/燃油兼容性（Concorde, C130, L410）— `pending`
- **Concorde**：Collector tanks (13-16) 仅 46 lbs，满推力 4s 耗尽。需要 cross-feed 从主油箱补油。注释："LP valve to emulate cross feed... can feed 75000 kg/h per engine"
  - 方案：起飞期间每帧通过 `propulsion/tank[n]/contents-lbs` 属性补充 collector tanks
- **C130**：t56_prop 缺少 `gearratio`（应为 ~13.5:1），涡轮转速 ~13800 RPM vs 螺旋桨 maxrpm=1400，导致推力仅 1328 lbs（应为 ~40000 lbs）
  - 方案：在 EngineManager 中通过 property 设置 gearratio，或接受为模型缺陷
- **L410**：engTM601 有 `cutoff-cmd` 属性（可能默认=1 燃油切断）+ `betarangeend=64`（油门<64%仅怠速）。两个引擎交替正负振荡
  - 方案：启动时显式设置 `cutoff-cmd=0`；研究 betarangeend 对 throttle ramp 的影响

### 8.3 飞行阶段问题 ✅

#### 8.3a：MD11 fly-to/landing ✅ — `commit 5d5f7ac5`
- 四个根因：Iyy 属性名、has_mixture 误判、惯性速度含地球自转、高空着陆 throttle
- MD11: 750s crash → 2389s completed
- 副作用：F80C (TIMEOUT→1740s)、T38 (crash→572s)

#### 8.3b：非指令升空 + CLmax + DHC6 ✅ — `commit 55c935f8`
- **非指令升空检测**：`!WOW && AGL>10m && vc>25kts` → 自动进入 kRotateAndClimb，跳过 ramp
- **Heading hold 速度门槛**：正常 85% Vr / 非指令升空 100% Vr
- **CLmax 分档**：涡桨 2.0、delta wing (AR<2.5) 2.5、默认 1.6
- **Iyy 属性名修复**：EngineManager 中 `iyy-lbsft2` → `iyy-slugs_ft2`
- **高空着陆下降油门分档**：涡桨 0.50、涡扇 0.70
- DHC6: TIMEOUT → 3031s completed

#### 8.3c：OV10 重量分类巡航高度 ✅ — `commit 2efd8cb9`
- **根因**：OV10 T76 引擎 XML 使用 `<turbine_engine>` 而非 `<turboprop_engine>`，JSBSim 报告 `etTurbine`，测试程序分配 8000m 巡航高度。OV10 推力不足到达，持续爬升到 6988m 超时。
- **修复**：turbine 巡航高度按 `inertia/weight-lbs` 动态分类替代型号名硬编码：
  - < 15k lbs → 4000m | < 100k lbs → 8000m | ≥ 100k lbs → 10000m
- 同时移除 B17/C130 型号名硬编码
- OV10: TIMEOUT → 1991s completed

### 8.3d 已知限制

| 机型 | 状态 | 根因 | 决策 |
|------|------|------|------|
| **XB-70** | 💥 CRASH | JSBSim delta wing 模型俯仰不稳定，Vr 可达但旋转后 pitch 69°+ 不可控 | 不修 |

### 8.4 着陆 crash（T38, A4）— ✅ `已自动解决`
- **T38** ✅：404s completed（has_mixture 修正 → 能量分类正确）
- **A4** ✅：166s completed（阶段 8.1 或更早已修复）

## 🆕 阶段 9 — 航路点速度与飞行包线 — `pending`

### 问题
- `Waypoint` 缺少 `speed_mps` 字段，航路点不携带目标速度
- `ExecuteFlyTo()` 用 `GetTrueSpeedMps()`（惯性速度含地球自转 ~465 m/s）作速度目标
- 对 `ref_speed_mps = 0` 的机型无 cap，能量管理失效
- JSBSim 不提供飞行包线数据结构（`aero/alpha-max-rad` 大部分机型未定义）

### JSBSim 可用运行时属性
- `forces/fwx-aero-lbs`（气动阻力）、`forces/fwz-aero-lbs`（气动升力）
- `aero/qbar-area`（动压×翼面积）、`forces/lod-norm`（升阻比）
- `propulsion/engine[n]/thrust-lbs`（当前推力）
- `atmosphere/rho-slugs_ft3`（空气密度，随高度变化）

### 拟定步骤
1. `Waypoint` 加 `speed_mps` 字段（0.0 = 使用机型默认巡航速度）
2. 新增机型性能分类：基于引擎类型+重量计算默认巡航/失速速度
3. `ExecuteFlyTo()`/`ExecuteOrbit()` 使用 waypoint 速度替代 `GetTrueSpeedMps()`
4. 能量管理增加 Vmin/Vmax 约束（运行时从力平衡推导速度边界）
5. 测试程序传入合理速度值

### 速度包线策略
- 速度在 [Vmin, Vmax] 内 → 正常保持
- 速度 > Vmax → 限速（减小油门/减速板）
- 速度 < Vmin → 安全策略（俯冲增速/减小爬升角）
- 未指定速度 → 机型分类默认值

## 关键决策

| 决策 | 理由 |
|------|------|
| 全动态检测 | 无机型名硬编码，靠属性存在性判断 |
| reset XML 加载 | 每机型 AGL 正确，非手工维护映射表 |
| 渐进旋转 3s | 模拟飞行员柔和操作 |
| OV10 guard 规则 | `ap/autopilot-roll-on` 单属性泄漏，需额外证据 |
| FBW 优先 | f16 有 ap/heading_hold（FBW 暴露）但需 kFbwRateCommand |
| 重量分类巡航高度 | JSBSim 引擎 XML 类型不一定反映真实引擎类型 |

## 工具

- `cmake --preset llvm-ninja-release-local` 构建
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4`
- `build/llvm-ninja-release-local/bin/takeoff_land_csv <model> <out.csv>`
- `python3 tools/analyze_takeoff.py [--plot] <csv...>`
