# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`，fd 6/6 绿（release ~9s）。

## 本次会话（2026-05-31，续）

### 航路点自适应捕获半径
- 修复：`IsManeuverComplete()` 中用 `max(user_radius, V²/(g×tan(bank_max))×1.5)` 自动扩大捕获半径
- 快飞机不再因转弯半径大于航路点距离而永远盘旋

### 着陆控制器重写
- **起落架 Bug 修复**：爬升阶段收轮后，着陆时 `ConfigureForApproach` 和 `ConfigureForLanding` 都要放轮
- **减速段** (`kDecelerate`)：着陆开始时收油门、固定 elevator=-0.05，等速度降到 approach_speed×1.15
- **高空下降段** (`kApproach` 高高度)：gentle descent，elevator 限制在 -0.15/+0.1
- **低空进场段** (`kApproach` 低高度)：固定 -3° 下滑角，油门控速
- **最终下降** (`kFinalDescent`)：同上 + 超速时主动收油抬头
- **渐进式 Flare** (`kFlare`)：elevator 从 -0.15 到 -0.40 渐进增加
- **着陆接地检测**：WOW 传感器优先，`agl<0.1m` 备份，起落架压缩容差 -0.5m

### 进场速度计算
- `GetRotationSpeedKts()` 加 40kt 下限
- `GetDefaultApproachSpeedMps()`：按引擎类型返回默认值（piston=28, turboprop=41, turbine=62 m/s）
- 进场速度上限 75% 当前速度（去掉了 55% 下限）

### 起降实测（最新）

```
c172x  ✅✅✅  全任务完成 (1224s)
c310   ✅✅✅  全任务完成 (518s)
737    ✅✅✅  全任务完成 (2301s)
f16    ✅❌—   起飞+巡航完成，着陆 crash (117s, 432m/s 超音速无法减速)
```

### f16/f22 待处理
- f16: 低空超音速巡航 (432m/s @ 500m)，着陆无法在短距离内减速
- f22: FBW rate-integrator 限制，6 种方案均未解决
- 两者都需要 FBW 兼容的着陆控制策略

### 本次修改的文件
- `src/flight_dynamic/guidance/Maneuver.cpp` — 着陆控制器重写、起落架修复、渐进 flare
- `src/flight_dynamic/propulsion/EngineManager.cpp` — Vr 下限、默认进场速度
- `src/flight_dynamic/propulsion/EngineManager.h` — 新增 `GetDefaultApproachSpeedMps()`
- `src/flight_dynamic/FlightManager.cpp` — crash 检测容忍起落架压缩、着陆阶段豁免
- `include/1q/flight_dynamic/guidance/Maneuver.h` — 新增 `kDecelerate`、`IsTouchingGround()`
