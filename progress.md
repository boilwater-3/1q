# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`，18/18 测试全绿。commit `06b37c3f`。

**阶段 1-7 完成，阶段 8.1 完成，阶段 8.2-8.4 待开始。**

## 全机型 takeoff_land_csv 测试结果（2026-06-03，阶段 8.3 MD11 修复后）

### ✅ 完成 OK（12 机）
| 机型 | 时间 | max_agl |
|------|------|---------|
| c172x | 1311s | — |
| c172p | 680s | 1449m |
| c172r | 781s | 1434m |
| c182 | 532s | 1437m |
| c310 | 536s | — |
| 737 | 1242s | — |
| B747 | 2436s | — |
| **MD11** | **2389s** | **~10965m** |
| f16 | 131s | 1047m |
| f15 | 154s | 492m |
| A4 | 166s | — |
| F4N | 120s | — |
| Boeing314 | 566s | 1467m |

### 💥 CRASH（1 机）
- **XB-70** (44s): delta wing pitch 爆发到 68°

### ⏰ TIMEOUT（5 机）
- **Concorde** (2500s, max_agl=3m): 引擎 4s 后熄火 — collector tanks 仅 46 lbs
- **C130** (2500s, max_agl=3m): 螺旋桨缺 gearratio，推力仅 1328 lbs
- **L410** (2500s, max_agl=2m): 引擎推力交替正负振荡，卡在 22 kts
- DHC6 (2500s, max_agl=9180m): 飞到高空但未完成
- OV10 (2500s, max_agl=6542m): 飞到高空但未完成
- F80C (2500s, max_agl=475m): 未完成

### ⏭️ SKIP（1 机）
- B17: Vr unreachable at MTOW

## 已完成的阶段

### 阶段 1：初始速度 → velocity=0 ✓
### 阶段 2：滑跑横侧 → wings-level during ground roll ✓
### 阶段 3：渐进旋转 → 3s elevator ramp ✓
### 阶段 4：F450 skip ✓
### 阶段 5：地面弹跳修复 ✓
### 阶段 6：F4N 回归修复 ✓
### 阶段 7：硬编码清理 ✓
### 阶段 8.1：重型机起飞稳定化 ✓
- 三个根因：地面 heading hold 导致滚转发散、elevator ramp 中断、climb rate P 控制器振荡
- B747 从 70s 坠毁 → 完整完成（986s）
- MD11 从 42s 坠毁 → 起飞成功（10346m）

### 阶段 8.3：部分飞行阶段修复 ✓（MD11 fly-to/landing）
- 四个根因：
  1. **Iyy 属性名错误**：`inertia/iyy-lbsft2` 不存在 → 改为 `inertia/iyy-slugs_ft2`
  2. **has_mixture 误判**：`fcs/mixture-cmd-norm` 对所有引擎类型创建 → 同时检查 `propulsion/magneto_cmd`
  3. **惯性速度含地球自转**：`GetInertialVelocityMagnitude()` 含 ~465m/s → 改用 TAS 计算转弯半径
  4. **高空着陆 throttle 骤降**：kDecelerate throttle=0.1 → AGL>3000m 用 AP altitude hold 下降
- MD11 从 750s crash → 完整完成（2389s）

## 待处理

| 子任务 | 机型 | 类型 | 优先级 |
|--------|------|------|--------|
| 8.2 引擎/燃油兼容性 | Concorde, C130, L410 | 模型缺陷 workaround | 中 |
| 8.3 剩余飞行阶段问题 | XB-70, DHC6, OV10, F80C | 控制逻辑 | 中 |
| 8.4 着陆 crash | T38, A4 | 控制逻辑 | 低 |

## git 历史

```
(uncommitted) fix: MD11 fly-to/landing crash — Iyy property name, mixture detection, TAS for turn radius, high-altitude descent
06b37c3f fix: stabilize heavy aircraft takeoff (B747, MD11)
2d599d3f docs: update plan files after autopilot hardcoding removal
ec1c4449 refactor: remove model-name hardcoding from autopilot profile detection
```
