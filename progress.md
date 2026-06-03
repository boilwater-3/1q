# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`，fd_ci 3/3 绿。commit `cd8900e9`。

**阶段 1-8.3 完成，阶段 9（航路点速度+飞行包线）核心实现完成。**

## 全机型 takeoff_land_csv 测试结果（2026-06-03，commit cd8900e9 后）

### ✅ 完成 OK（14 机）
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
| **T38** | **406s** | 8.3 副作用修复 |
| **DHC6** | **3053s** | 8.3 修复 |
| **OV10** | **2014s** | 8.3c 重量分类修复 |
| Boeing314 | 565s | |

### ⚠️ 回归（2 机）
| 机型 | 状态 | 根因 | 来源 |
|------|------|------|------|
| **B747** | 💥 abort 2683s | 着陆 flare 阶段浮力过长，低速滚转发散。GetTrueSpeedMps→TAS 后 AP 速度保护行为变化 | cd8900e9 |
| **F80C** | ⏰ TIMEOUT | fly-to 航路点仅 9km，takeoff 1740s 后飞机已飞过 410km | 2efd8cb9 |

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
### 阶段 9：航路点速度 + 飞行包线 — `commit cd8900e9` — 核心完成

#### 核心修复
- **`GetTrueSpeedMps()` → TAS**：修复惯性速度含地球自转 (~465 m/s) 的 bug。影响所有速度相关计算。
- **`Waypoint.speed_mps`**：航路点新增目标速度字段 (0 = 使用默认巡航速度)
- **速度包线**：profile 新增 `cruise_speed_mps`（默认巡航速度）和 `max_speed_mps`（最大速度），按机型分类
- **中型机分类**：log10(Iyy) 6-7 为中型运输机（737 类），获得独立能量管理参数
- **轻型机兜底**：非活塞非 FBW 非重型的涡扇/涡桨（OV10, f15 等）获得合理默认速度参数

#### ExecuteFlyTo/ExecuteOrbit 修复
- 速度目标优先级：waypoint.speed_mps > profile.cruise_speed_mps > 当前 TAS
- 速度 clamp 到 [min_speed, max_speed] 范围
- 替代了旧的 `GetTrueSpeedMps()` (惯性速度) hack

#### 能量管理增强
- max_speed 超速保护：速度 > 95% max_speed 时主动降低 throttle
- min_speed 失速保护：已存在，不变
- ref_speed fallback 链：ref_speed > cruise_speed > current_speed

#### 着陆改进
- kDecelerate→kApproach 时关闭 AP altitude hold（防止与着陆 pitch 冲突）
- 重型机 flare 起始高度降低：vc×0.25 vs vc×0.6

## 待处理

| 子任务 | 机型 | 类型 | 优先级 |
|--------|------|------|--------|
| 8.2 引擎/燃油兼容性 | Concorde, C130, L410 | 模型缺陷 workaround | 中 |
| B747 着陆 flare | B747 | 回归修复 | 高 |
| F80C fly-to 航路点距离 | F80C | 回归修复 | 高 |
| — | XB-70 | JSBSim 模型限制 | 不修 |

## git 历史

```
cd8900e9 feat: waypoint speed field and speed envelope management
2efd8cb9 fix: OV10 TIMEOUT — weight-based cruise altitude for turbine aircraft
55c935f8 fix: uncommanded liftoff, CLmax, DHC6 landing — three aircraft resolved
5d5f7ac5 fix: MD11 fly-to/landing crash — four root causes
06b37c3f fix: stabilize heavy aircraft takeoff (B747, MD11)
```
