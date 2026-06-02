# 任务计划：起飞控制器修复（CRASH 机型）

## 目标

修复 7 个 CRASH 机型（L410, OV10, F450, B747, MD11, XB-70, T38）的起飞/着陆问题，使起飞控制逻辑接近现实程序。

## 背景

已完成的阶段 0-13 交付了适配层、控制接口、机动语义、测试体系等基础设施。当前 10/28 机型全任务通过（c172x, c172p, c310, 737, f16, f15, F4N, F80C, Boeing314, pc7）。

## 根因分析（更新于 2026-06-02 第二次会话）

### 原始根因（有偏差）
1. **初始弹跳**：velocity=1.0 m/s → RunIC 后某些机型速度异常
2. **阶跃旋转**：Vr 时 elevator 0→-0.3 瞬变
3. **无横侧控制**：kEngineStart/kTakeoffRoll 阶段 AP 不激活

### 深层根因（经 LLDB + 源码 + 多轮实验确认）

**核心问题：JSBSim RunIC 的 `SuspendIntegration()` 设计缺陷 + 涡桨发动机高静态推力**

```
RunIC 流程：
  ① SuspendIntegration() → dt=0
  ② SetInitialState → aircraft at AGL=0, gear extends below ground
  ③ Run() → ground force computed, NOT integrated (dt=0)
  ④ Run() → same, no integration
  ⑤ ResumeIntegration() → dt restored

InitRunning(-1) → throttle=1.0, GetSteadyState() → engine at full thrust

第一次 Step() → Run(dt=0.01) → 满推力 + 地面穿透力 → 弹跳
```

**为什么只有部分机型崩溃？**

| 机型 | 初始 vc | 发动机类型 | 结果 |
|------|--------|-----------|------|
| 737 | 2.2 kts | 双发涡扇 | ✅ PASS — 涡扇静态拉力小 |
| f16 | 5.0 kts | 单发涡扇 | ✅ PASS |
| c172x | 7.7 kts | 单发活塞 | ✅ PASS — 活塞推力适中 |
| **L410** | **20.8 kts** | 双发涡桨 | 💥 CRASH — 涡桨静态拉力极大 |
| **OV10** | **8.1 kts** | 双发涡桨 | 💥 CRASH — 涡桨+轻机身加速快 |
| **B747** | ? | 四发涡扇 | 💥 CRASH — 阶跃elevator问题 |
| **MD11** | 13.2 kts | 三发涡扇 | 💥 CRASH — 弹跳 |

涡桨发动机在 V=0 时产生最大静态拉力（propeller 效率在低速最高），而涡扇在低速时拉力最小。

## 阶段

### 阶段 1：初始速度修复 — `complete`
- **目标**：消除 initial_velocity=1.0 导致的弹跳离地
- **方案**：地面 altitude=0 时 velocity=0
- **结果**：✅ 已修改 `takeoff_land_csv.cpp` line 76 (0.0)
- **问题**：velocity=0 不解决问题——RunIC 后 UVW 确实为 0，但第一个 Run() 积分产生非零速度

### 阶段 2：滑跑横侧控制 — `complete`
- **目标**：起飞滑跑阶段激活 wings-level 保持
- **文件**：`src/flight_dynamic/guidance/Maneuver.cpp`
- **结果**：✅ 已修改 `StartEngine()` — 增加 roll_mode=0, roll_ap_on=true

### 阶段 3：渐进式旋转 — `complete`
- **目标**：消除阶跃 elevator 导致的急剧翻滚
- **文件**：`src/flight_dynamic/guidance/Maneuver.cpp` ConfigureForClimb + kRotateAndClimb
- **结果**：✅ 已修改 — 3s ramp: `el = -0.3 * min(elapsed/3.0, 1.0)`

### 阶段 4：F450 跳过 — `complete`
- **目标**：排除非固定翼机型
- **文件**：`examples/flight_dynamic/takeoff_land_csv.cpp`
- **结果**：✅ F450 标记为 skipped (multirotor)

### 阶段 5：全机型回归测试 — `pending`
- **阻塞原因**：弹跳根因未修复，crash 机型仍 crash

### 阶段 6：T38 着陆调试 — `pending`

### 🆕 阶段 7：RunIC 地面弹跳修复 — `partial`
- **根因**：RunIC SuspendIntegration 导致地面力在 dt=0 时不积分；InitRunning(-1) 设 throttle=1.0；涡桨静态拉力极大的机型在第一个 Run() 弹跳
- **已尝试的失败方案**：
  1. 高度偏移 0.001m / 0.5m — 不够，齿轮支柱 2-3m
  2. InitRunning 移到 RunIC 之前 — 弹跳是地面力，不是推力
  3. 多帧沉降循环 — 产生发散振荡，越弹越高
  4. InitRunning 后重置 throttle=0 — StartEngine 又设回 1.0
  5. JSBSim RunIC 加真实 dt Run() — release build 用预编译库
  6. 源码取消 RunIC SuspendIntegration — L410 首帧 vc 20.8→41.5 kts，弹跳提前/放大
  7. RunIC 内按起落架压缩量抬升 AGL — L410 被抬到空中后 0.4~1.8s crash，OV10 只能推迟到 ~62s
  8. RunIC 内按地面反力/重量估计静态压缩 — L410 仍 0.4s crash，说明仅修正 RunIC 几何不够
- **待探索方案**：
- **已验证可行的部分方案**：
  - A. 保持 JSBSim RunIC 设计不变，在 adapter 层执行 `SetHoldDown(true)` 初始 settle，期间 throttle=0、brakes=1，再释放 hold-down。
  - B. `InitRunning(-1)` 后必须同时清理 FCS indexed/unindexed throttle 和 `FGPropulsion::in.ThrottleCmd/ThrottlePos`，只清 property tree 不够。
  - C. takeoff state machine 已改为 idle/start → optional static runup → brake release → throttle ramp/takeoff roll → rotate。
- **实测结果**：
  - `fd_ci` 3/3 passed。
  - c172x 完整起降 completed at 1144.6s。
  - L410 首帧 vc 20.8→0.0 kts，但 1.03s 仍 WOW=1 下沉到 h-agl=-5.02m hard crash。
  - OV10 首帧 vc 8.1→0.0 kts，但 4.01s 翻转 crash。
- **待探索方案**：
  - A. L410：地面接触/支撑已分离。需要继续定位涡桨/螺旋桨零 RPM spool-up，避免继续把它归因于首帧弹跳。
  - B. OV10：独立检查 XML/发动机类型、低速 roll/yaw/rotate 控制链和推力线。
  - C. 若继续源码线，应实现真实静态支柱平衡或正式 hold-down/ground-initialization 语义，而不是取消 SuspendIntegration 或简单抬高 AGL。

## 关键决策

| 决策 | 理由 |
|------|------|
| 地面 v=0，空中 v=1.0 | 分离 NaN 问题（高空）和弹跳问题（地面）— 但未解决弹跳 |
| 渐进旋转 3s | 现实飞行员 2-3°/s，重型机需要柔和 |
| F450 跳过 | 多旋翼模型，固定翼逻辑不适用 |
| 阶段 7 分层处理 | adapter HoldDown settle 已消除首帧速度跃迁；L410/OV10 剩余失败需分别定位 |
| 起飞逻辑下一步转向 state machine 重构 | 现实程序强调平滑加油、方向/横侧控制随速度建立、Vr 附近平滑抬轮；当前满油门阶跃不符合 |

## 遇到的错误

| 错误 | 尝试次数 | 解决方案 |
|------|---------|---------|
| v=1.0 导致弹跳 | 1 | 改为地面 v=0 — 无效 |
| 高度偏移 | 1 | 0.001~0.5m 不够 |
| InitRunning 时序 | 1 | 前后移都无效 |
| 沉降循环 | 2 | 发散振荡 |
| throttle 重置 | 1 | StartEngine 覆盖 |
| JSBSim 源码修改 | 1 | release 用预编译库 |
| 取消 RunIC SuspendIntegration | 1 | 否定：L410 首帧速度放大到 41.5 kts |
| RunIC 起落架压缩几何修正 | 3 | 否定：L410 更早 crash，OV10 只能推迟失败 |
| adapter HoldDown settle | 1 | 部分成功：L410/OV10 首帧 vc 均降为 0.0 kts；剩余失败另行分层 |
| L410 gear input + 起始高度 | 1 | 部分成功：同步 `/controls/gear/gear-down-cond` 并使用 7.1 ft 起始高度后，不再穿地/弹跳 crash |
| L410 propeller 初始 RPM=1600 | 1 | 否定：首帧约 32M lbs 推力，0.08s crash |

## 工具

- `cmake --preset llvm-ninja-release-local` 构建（预编译 JSBSim）
- `cmake --preset jsbsim-source-debug-local -DENABLE_EXAMPLES=ON` 构建（源码 JSBSim，可 LLDB）
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4` CI 测试
- `build/llvm-ninja-release-local/bin/takeoff_land_csv <model> <out.csv>` 起降测试
- `python3 tools/analyze_takeoff.py [--plot] <csv...>` CSV 分析
