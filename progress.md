# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`，fd_ci 3/3 绿。commit `8484c90c`。

**阶段 1-9 核心实现完成，F80C 回归已修复。**

## 全机型 takeoff_land_csv 测试结果（2026-06-04，commit 8484c90c 后）

### ✅ 完成 OK（15 机）
| 机型 | 时间 | 备注 |
|------|------|------|
| c172x | 1344s | |
| c172p | 867s | |
| c172r | 876s | |
| c182 | 817s | |
| c310 | 686s | |
| 737 | 1926s | |
| **MD11** | **2516s** | |
| f16 | 132s | |
| f15 | 116s | |
| A4 | 166s | |
| F4N | 120s | |
| **T38** | **406s** | |
| **DHC6** | **3053s** | |
| **OV10** | **2014s** | |
| **F80C** | **1776s** ✅ | fly-past detection + 航路点距离增大 |
| Boeing314 | 565s | |

### ⚠️ 已知回归（1 机）
| 机型 | 状态 | 根因 | 来源 |
|------|------|------|------|
| **B747** | 💥 abort 2683s | 进近速度过高 (150 kts) + 全襟翼升力≈重量 → flare 浮空 20s+ → 滚转发散。bounce/float recovery 已加入但不够突破物理极限。高空空气密度低致减速不足。 | cd8900e9 |

### 💥 CRASH（1 机）
- **XB-70**: JSBSim delta wing 模型俯仰不稳定。**已知模型限制。**

### ⏰ TIMEOUT（3 机）— 引擎兼容性
| 机型 | 根因 |
|------|------|
| **Concorde** | 引擎 4s 后熄火 — collector tanks 仅 46 lbs |
| **C130** | 螺旋桨缺 gearratio，推力仅 1328 lbs |
| **L410** | 引擎推力交替正负振荡 |

### ⏭️ SKIP（2 机）
- B17: Vr unreachable at MTOW
- F450: multirotor

## 已完成的阶段

### 阶段 1–7 ✓（之前完成）
### 阶段 8.1：重型机起飞稳定化 ✓ — `commit 06b37c3f`
### 阶段 8.3a：MD11 fly-to/landing ✓ — `commit 5d5f7ac5`
### 阶段 8.3b：非指令升空 + CLmax + DHC6 ✓ — `commit 55c935f8`
### 阶段 8.3c：OV10 重量分类巡航高度 ✓ — `commit 2efd8cb9`
### 阶段 9：航路点速度 + 飞行包线 — `commits cd8900e9 + 8484c90c` — 核心完成

#### 阶段 9a：航路点速度字段 + 速度包线 ✅ — `cd8900e9`
- **`GetTrueSpeedMps()` → TAS**：修复惯性速度含地球自转 (~465 m/s) 的 bug。
- **`Waypoint.speed_mps`**：航路点新增目标速度字段 (0 = 使用默认巡航速度)
- **速度包线**：profile 新增 `cruise_speed_mps` 和 `max_speed_mps`，按机型 6 档分类
  - FBW 战斗机 (200/140/350 m/s)、重型涡扇 (250/130/300)、中型涡扇 (200/100/260)
  - 重型活塞 (75/65/100)、GA 活塞 (50-60/40-55/80-90)、轻型涡扇兜底 (80/50/200)
- **中型机分类**：log10(Iyy) 6-7 为中型运输机（737 类），获得独立能量管理参数
- ExecuteFlyTo/ExecuteOrbit 速度目标优先级：waypoint > cruise_speed > 当前 TAS，clamp 到 [min, max]
- 能量管理增加 max_speed 超速保护 (>95% 时降低 throttle)
- kDecelerate→kApproach 时关闭 AP altitude hold（防止与着陆 pitch 冲突）

#### 阶段 9b：两个回归修复 ✅ — `8484c90c`
**F80C TIMEOUT 修复：**
- **Fly-past detection**：`IsManeuverComplete()` 的 kFlyToWaypoint 分支新增后方航路点检测：如果航路点 >115° 偏机头且距离 >3× 捕获半径，且飞行时间 >10s，则标记为完成。
  - 解决 `HasPassedActiveWaypoint()` 在飞机起点已超过航路点时无法检测的缺陷
- **航路点距离增大**：`takeoff_land_csv.cpp` 中，max_speed > 150 m/s 的快速飞机使用 `max_speed × 80` 替代 `ref_speed × 45`

**B747 着陆改进（不完全修复）：**
- **重型机 flare 下沉率控制器**：改为基于高度的柔和电梯 + 弹跳恢复 + 浮空恢复
  - 弹跳恢复：sink_rate > 0 且 AGL < 10m 时立即推杆 (+0.10 nose-down)
  - 浮空恢复：在 AGL < 10m 悬浮 >5s 后逐渐推杆
- **AGL < 3m 接地检测**：kFlare→kTouchdown 转换要求 `WOW && agl < 3m`，防止高起落架飞机（B747 主轮在 ~5m 触地）提前进入 kTouchdown 导致弹跳
- **Flare 起始高度恢复**：移除重型机的降低系数 (`vc×0.25`)，回退到统一 `vc×0.6`
- **kFinalDescent 速度管理**：当速度 > 进近速度 ×1.05 时限制油门 ≤0.15，允许减速

**B747 剩余问题：**
- 高空段（10000→3000m）空气密度低，减速极慢（`descent_thr=0.70` 维持速度不变）
- 即使油门降至 0.30 或 0.15，高空减速效果有限
- 进近速度过高（150 kts/290 km/h），全襟翼升力≈重量 → 无法下降
- **修复方向**：进近阶段重构——增加盘旋减速、降低进近襟翼设置、或引入减速板

## 待处理

| 子任务 | 机型 | 类型 | 优先级 |
|--------|------|------|--------|
| 8.2 引擎/燃油兼容性 | Concorde, C130, L410 | 模型缺陷 workaround | 中 |
| B747 着陆 | B747 | 回归修复（需进近阶段重构） | 低 |
| — | XB-70 | JSBSim 模型限制 | 不修 |

## 2026-06-04 09:20 CST — 阶段 10 优先级调整

用户要求推迟 8.2 引擎/燃油兼容性，优先处理：
1. XML 属性驱动的配置方案（替代硬编码 MOI 阈值）
2. B747 着陆进近阶段重构（高空减速、盘旋下降、襟翼管理）

已更新 `task_plan.md`：阶段 10a XML 配置驱动为 `in_progress`，阶段 10b B747 进近重构为 `pending`。

初步代码观察：
- `Autopilot.cpp` 的能量和旋转默认值仍主要依赖 `engine_count`、`has_mixture`、`log10(Iyy)` 分类。
- `Maneuver.cpp` 的着陆参数仍有多处固定值：高空下降门槛 3000m、中间高度 2000/3000m、进近/最终襟翼 0.5/1.0、重型 flare 初始电梯由 `log10(Iyy)>7` 判断。
- `JsbsimAdapter::ConfigureIntegrators()` 当前会把已有 XML `guidance/roll-angle-limit` 覆盖为 45deg，这与 XML 优先配置目标冲突，需要先修正为“XML 存在则保留，缺省才创建默认值”。

## 2026-06-04 09:45 CST — 阶段 10 完成：XML 配置驱动 + B747 进近重构

完成内容：
- `AircraftControlProfile` 新增 landing/approach XML override 字段：进近速度、高空下降门槛、staging/pattern 高度、默认盘旋下降开关、下降油门、进近/最终襟翼、最终油门上限、flare 初始电梯、touchdown AGL。
- `Autopilot.cpp` 新增 `ApplyXmlProfileOverrides()`，优先读取 `guidance/*` 属性；不存在时保留动态检测/MOI 分类默认值。
- `JsbsimAdapter::ConfigureIntegrators()` 改为 XML 已存在 `guidance/roll-angle-limit` / `guidance/roll-rate-limit` 时不覆盖。
- B747 XML 写入 landing guidance 参数；`landing-high-descent-orbit` 不显式写入，因为默认值已改为开启。
- B747 decelerate 阶段修复：速度高于 `approach_speed * 1.35` 时继续 orbit 并保持 pattern altitude，避免在 200kt+ 时贴地/WOW。
- Landing 入口释放遗留 AP hold，flare/touchdown/rollout 期间中和横侧输入。

验证：
- `cmake --build --preset llvm-ninja-release-local --target 1q_fd_tests takeoff_land_csv` 通过。
- `build/llvm-ninja-release-local/bin/1q_fd_tests --gtest_filter='FlightDynamicTest.AutopilotReadsB747LandingGuidanceOverrides:AircraftProfiles/ProfileSnapshotTest.MatchesExpectedProfile/*'`：8/8 passed。
- `build/llvm-ninja-release-local/bin/1q_fd_tests`：158 passed, 3 skipped。
- `build/llvm-ninja-release-local/bin/takeoff_land_csv B747 /tmp/1q_fd/b747_stage10_mid_speed_orbit.csv`：B747 completed at 1475.0s。
- Landing 诊断：`outcome=completed | alt_min=4.10202m | spd_min=2.97916m/s | roll_max=54.72deg | pitch_max=11.058deg | crashed=no`。
- CSV 末段：`vc≈5.8kt`、`roll=0`、`elevator=0`、`aileron=0`、`WOW=1`，完成前无穿地 crash。

## git 历史

```
8484c90c fix: F80C fly-to timeout and B747 flare improvements
cd8900e9 feat: waypoint speed field and speed envelope management
f16848f8 docs: update planning files for speed envelope and OV10 fix
2efd8cb9 fix: OV10 TIMEOUT — weight-based cruise altitude for turbine aircraft
55c935f8 fix: uncommanded liftoff, CLmax, DHC6 landing — three aircraft resolved
5d5f7ac5 fix: MD11 fly-to/landing crash — four root causes
06b37c3f fix: stabilize heavy aircraft takeout (B747, MD11)
```
