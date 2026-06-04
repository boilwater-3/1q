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
