# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`，fd_ci 3/3 绿（release ~0.5s）。

**暂停中**：阶段 1-4 代码修改已完成但无法解决弹跳根因，阶段 7 暂停等待新方案。

## 全机型测试结果（2026-06-02）

### ✅ PASS（10 机）
c172x, c172p, c310, 737, f16, f15, F4N, F80C, Boeing314, pc7

### 💥 CRASH（6 机）— 本次修复目标
- L410 (3.2s): 涡桨静态拉力 → 首帧 vc=20.8kts → 弹跳离地
- OV10 (3.9s): 同上，双发涡桨
- F450: ⏭️ SKIP（多旋翼）
- B747 (47.5s): 阶跃 elevator → roll departure
- MD11 (8.9s): 弹跳离地
- XB-70 (77.5s): 弹跳

### 🛑 GROUND（3 机）
C130, Concorde, p51d

### ⚠️ DIVERGE/NO_LAND（7 机）
A4, DHC6, global5000, t6texan2, T37, c172r, 787-8

### ⏭️ SKIP（2 机）
B17 (Vr 不可达), F450 (多旋翼)

## 会话 2026-06-02（续）：RunIC 弹跳修复

### 阶段 1 ✅ — 初始速度修复
- takeoff_land_csv.cpp: velocity=1.0 → 0.0
- 结果：RunIC 后 UVW=0 正确，但第一个 Run() 积分产生弹跳

### 阶段 2 ✅ — 滑跑横侧控制
- Maneuver.cpp StartEngine(): 增加 roll_mode=0, roll_ap_on=true

### 阶段 3 ✅ — 渐进式旋转
- Maneuver.cpp ConfigureForClimb(): 去掉阶跃 elevator=-0.3
- Maneuver.cpp kRotateAndClimb: 3s ramp (el = -0.3 * progress)

### 阶段 4 ✅ — F450 跳过
- takeoff_land_csv.cpp: F450 → skipped (multirotor)

### 阶段 7 ❌ — RunIC 弹跳（源码实验继续失败）

| 尝试 | 方案 | 结果 |
|------|------|------|
| 1 | altitude=0.001m | 太小，齿轮支柱 2-5m |
| 2 | altitude=0.5m | 不够 |
| 3 | InitRunning 移到 RunIC 前 | 无效 |
| 4 | 50帧沉降循环 | 发散振荡 |
| 5 | InitRunning 后 throttle=0 | StartEngine 覆盖 |
| 6 | JSBSim RunIC 加 dt>0 Run | release 用预编译库 |
| 7 | `jsbsim-source-debug-local -DENABLE_EXAMPLES=ON` 构建源码版 `takeoff_land_csv` | 成功构建，源码版复现 L410 3.16s / OV10 3.86s crash |
| 8 | 取消 `RunIC()` 内 `SuspendIntegration()/ResumeIntegration()` | 否定：L410 首帧 41.5 kts，OV10 首帧 17.4 kts，弹跳提前/放大 |
| 9 | RunIC 后按最大起落架压缩量抬升 AGL | 否定：L410 AGL=2.27m、WOW=0，0.4s crash；OV10 推迟到 61.9s |
| 10 | RunIC 后按 50%/25% 压缩释放 | 否定：L410 1.2s/1.8s crash；OV10 63.5s/3.3s crash |
| 11 | RunIC 后按地面反力/重量估计释放压缩 | 否定：L410 仍 0.4s crash；OV10 仍约 61.9s crash |

### 关键发现
- RunIC `SuspendIntegration()` → dt=0 → 地面力不积分
- 涡桨发动机静态拉力极大（V=0 时 propeller 效率最高）
- 涡扇（737: 2.2kts）和活塞（c172x: 7.7kts）不受影响
- 弹跳根本原因是起落架穿透+发动机满推力在第一个 Run() 中被同时积分
- 源码层“直接让 RunIC 积分”会放大首帧冲击；“简单抬高 AGL”会让 L410 悬空再坠地
- 当前更可信的下一步是适配器/机动层地面启动策略，或源码层真实静态支柱平衡/hold-down 语义，而不是继续调压缩比例

## 会话 2026-06-02（现实起飞程序对照）

### 已核实
- FAA 正常起飞程序：松刹车后 throttle smooth/continuous 到 takeoff power；rudder/aileron 随速度建立方向/横侧控制；到合适速度平滑 back elevator 建立起飞姿态；正爬升后收轮/襟翼。
- T-38 公开 AFMAN：static takeoff 是 brakes hold + MIL runup + 检查稳定，松刹车后 MAX；约 135 KIAS 平滑后杆，145-150 KIAS 前轮离地，约 160 KIAS 飞离，正爬升后收轮。

### 当前实现偏差
- `StartEngine()` 直接刹车+满油门+启动，缺少 idle/runup/engine-check 与 throttle ramp。
- `ConfigureForTakeoffRoll()` 和 `kTakeoffRoll` 每帧满油门，缺少按机型/发动机类型的 takeoff power schedule。
- `EngineManager::SetThrottle()` 同时写 throttle cmd 和 pos，等价于无油门执行机构滞后；涡桨静态推力被瞬时施加。
- JSBSim `InitRunning()` 会内部置满油门并稳态发动机，虽然 adapter 有 throttle reset，但 maneuver 首帧又覆盖为满油门。

### 评估
- 当前优先级应从“改 RunIC 积分”转向“按现实起飞阶段重构 1q takeoff state machine”：idle/start -> optional static runup -> brake release -> throttle ramp/takeoff roll -> rotate -> positive climb cleanup。
- 对涡桨机型先做保守油门 ramp 和地面低速稳定；对喷气/T-38/F-16 保留 static runup 但分离 runup thrust 与 release 后 thrust。

### 当前 git 状态
```
M examples/flight_dynamic/takeoff_land_csv.cpp  (v=0, F450 skip)
M src/flight_dynamic/guidance/Maneuver.cpp       (roll hold, gradual rotation)
M src/flight_dynamic/adapter/JsbsimAdapter.cpp   (throttle reset after InitRunning)
M progress.md
M task_plan.md
M tools/analyze_takeoff.py                       (math.isfinite fix)
```

## 会话 2026-06-02 14:36 CST（适配器 HoldDown settle 实验）

### 已修改
- `JsbsimAdapter.cpp`
  - `InitRunning(-1)` 后重置 indexed/unindexed FCS throttle cmd/pos。
  - 同步清理 `FGPropulsion::in.ThrottleCmd/ThrottlePos`，避免只改 property tree。
  - 地面初始状态执行 5 帧 `SetHoldDown(true)` settle，期间 throttle=0、brakes=1，然后释放 hold-down。
- `EngineManager.cpp`
  - 发动机类型识别改为读取 JSBSim `FGEngine::GetType()`，保留 legacy property fallback。
- `Maneuver.cpp/.h`
  - takeoff 增加 idle/start、static runup、takeoff roll throttle ramp。
  - 起飞地面阶段 `WOW=1` 时允许接地，避免把起落架压缩误判为空中 crash。

### 验证结果
- `cmake --build --preset llvm-ninja-release-local --target takeoff_land_csv 1q_unit_tests -j 4` ✅
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4` ✅ 3/3 passed
- `takeoff_land_csv c172x /tmp/1q_c172x_touching.csv` ✅ completed at 1144.6s
- `takeoff_land_csv L410 /tmp/1q_L410_touching.csv` ❌ aborted at 1.03s
  - 首帧 vc 从 20.8 kts 降为 0.0 kts，说明初始化弹跳已被 HoldDown settle 消除。
  - 后续仍以 WOW=1 下沉到 h-agl=-5.02m，属于起落架/地面反力静态平衡或模型几何问题。
- `takeoff_land_csv OV10 /tmp/1q_OV10_touching.csv` ❌ aborted at 4.01s
  - 首帧 vc 从 8.1 kts 降为 0.0 kts。
  - 约 4s 时 roll≈176.6deg，WOW=0，属于后续姿态/控制稳定性问题，不再是首帧初始化弹跳。

### 当前判断
- “RunIC dt=0 不积分地面力”导致的首帧速度跃迁，可以在适配器层用显式 hold-down settle 绕过，不必取消 JSBSim `SuspendIntegration()`。
- 但这只解决第一类问题：初始化首帧能量注入。
- L410 剩余问题是地面静态支撑/几何穿透；OV10 剩余问题是起飞后低速姿态/横滚稳定性。
- 下一步不应继续把三类失败混在一起调阈值，应分别处理：
  1. L410：检查接触点、支柱压缩、重量/地面反力是否能平衡。
  2. OV10：检查 `EngineType` 为 `FGTurbine` 的 XML/发动机模型是否合理，以及起飞 roll/yaw/rotate 控制链。
  3. 通用起飞：保留 idle/runup/ramp，但需按机型验证 throttle schedule。
