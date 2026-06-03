# 发现：JSBSim 飞行机动模块

## 引擎类型与属性

6 类引擎，按 `propulsion/magneto_cmd` 存在与否区分活塞/非活塞：

| 类型 | JSBSim 类 | magneto | starter | mixture | 机型 |
|------|----------|:---:|:---:|:---:|------|
| 活塞 | FGPiston | ✅ | ✅ | ✅ | c172x, c310, B17, J3Cub, p51d |
| 涡喷 | FGTurbine | ❌ | ❌ | ❌ | f16, f22, f15, 737, 747, Concorde |
| 涡桨 | FGTurboprop | ❌ | ✅ | ❌ | L410, DHC6, PC7 |
| 火箭 | FGRocket | ❌ | ❌ | ❌ | X15, x24b |
| 电动 | FGElectric | ❌ | ❌ | ❌ | 无飞机引用 |

抬轮速度公式：`Vr = 1.1~1.2 × sqrt(2×Weight / (ρ × WingArea × CLmax))`，用 `metrics/Sw-sqft` + `inertia/weight-lbs` + `atmosphere/rho-slugs_ft3` 实时计算。

## JSBSim 内部机制

- **FCS 组件内部状态**：`FGFCSComponent::Output` 成员变量不通过 property tree 暴露。`SetProperty()` 只改 SGPropertyNode，不影响 integrator accumulator。必须用 `FGFCS::InitModel()` → `FGFCSChannel::Reset()` → `ResetPastStates()` 重置。
- **DoTrim(0) 对 FBW 机型必然失败**：JSBSim trim solver 无法处理闭环 FBW 状态。f22/c310 等机型每次 trim 都抛异常。
- **刹车模型有限**：JSBSim 地面接触中，满油门 + 轻型飞机（c172x）时刹车几乎无效，引擎启动阶段飞机会弹跳。
- **Debug 系统**：只有位掩码 `debug_lvl`（1=启动消息/2=实例化/4=Run入口/8=周期状态/16=越界检查），无结构化日志。

## 控制架构

```
1Q C++ Autopilot (机动语义层)
  → 写入 ap/* hold 标志 + fcs/elevator-cmd-norm
  → JSBSim 原生 AP/FCS (XML 控制链)
    → PID/积分器/滤波器/LQR
    → 舵面执行器 → 气动模型
```

- `kOwnAutopilot`（c172x, c310）：AP 更新只写 hold 标志，原生 XML AP 执行控制
- `kFbwRateCommand`（f16, f22）：直接写 `fcs/aileron-cmd-norm`；pitch 通过 `fcs/elevator-cmd-norm` 但不被 FBW 充分响应
- `kGenericAutopilotBridge`（737, Concorde）：写 `ap/*` 属性到 guidance 层

## RunIC 地面弹跳根因（2026-06-02 深入分析）

### 机制

JSBSim `FGFDMExec::RunIC()` 调用 `SuspendIntegration()` 将 dt 设为 0：
- 内部所有 `Run()` 调用计算力但不积分（零位移、零速度变化）
- 飞机在 `SetInitialState` 中被放置在 altitude=0（参考点在地面）
- 起落架支柱在参考点下方 2-5m → gear contact point 深入地下
- RunIC 内部 Run() 计算地面穿透力但 dt=0 不积分
- `InitRunning(-1)` 设置 throttle=1.0 并运行 `GetSteadyState()`（发动机稳态迭代）
- 第一个外部 `Run()` (dt=0.01) 同时积分满推力 + 地面穿透力 → 弹跳

### 机型差异

| 因素 | c172x (PASS) | L410 (CRASH) |
|------|-------------|-------------|
| 发动机 | 单发活塞 | 双发涡桨 |
| 静态推力 | 中 (~500N) | 极大 (~5000N×2) |
| 初始 vc (t=0.01) | 7.7 kts | 20.8 kts |
| 刹停能力 | 可刹停 | 推力 > 刹车力 |

涡桨在 V=0 产生最大静态拉力（propeller 在低速效率最高）；涡扇在低速拉力最小（737: 2.2 kts）。

### 尝试的方案

| 方案 | 结果 |
|------|------|
| velocity=0 | RunIC 后 UVW=0 ✓，但第一个 Run() 积分弹跳 |
| 高度偏移 0.001~0.5m | 齿轮支柱 2-5m，偏移不够 |
| InitRunning 移前移后 | 无效，弹跳是地面力非推力 |
| 多帧沉降 (50~300帧) | 发散振荡，越弹越高 |
| throttle 重置为 0 | StartEngine 又设回 1.0 |
| JSBSim RunIC 加 dt>0 Run() | release 用预编译库，修改无效 |
| jsbsim-source-debug-local | ENABLE_EXAMPLES=OFF 无 takeoff_land_csv |
| 源码取消 RunIC SuspendIntegration | 源码版可编译，但 L410 首帧 vc 20.8→41.5 kts，OV10 8.1→17.4 kts；否定 |
| RunIC 按最大起落架压缩抬升 AGL | L410 AGL=2.27m、WOW=0，0.4s crash；OV10 推迟到 61.9s crash；否定 |
| RunIC 按 50%/25% 压缩释放 | L410 1.2s/1.8s crash，OV10 63.5s/3.3s crash；否定 |
| RunIC 按地面反力/重量估计释放压缩 | L410 仍 AGL≈2.25m、0.4s crash；OV10 仍约 61.9s crash；否定 |

### 源码调试结论（2026-06-02）

`jsbsim-source-debug-local -DENABLE_EXAMPLES=ON` 可以构建源码版 `takeoff_land_csv`，并复现 release 问题：
- 未改源码：L410 3.16s crash，首帧 20.8 kts；OV10 3.86s crash，首帧 8.1 kts。
- 取消 `SuspendIntegration()` 不是修复，等价于把地面穿透力提前积分，首帧速度更大。
- 简单用起落架压缩量或地面反力/重量抬高 `position/h-agl-ft` 也不是稳定解：L410 会被放到空中再坠回地面；OV10 只被推迟失败。

下一步若继续源码线，应求解真实静态支柱平衡或引入显式 hold-down/ground-initialization 语义；若不改 JSBSim，则应在适配器/机动层处理发动机启动油门 ramp、刹车保持和涡桨地面启动。

## 现实起飞程序对照（2026-06-02）

公开资料核实：
- FAA Airplane Flying Handbook Chapter 6 将起飞拆成 takeoff roll、lift-off、initial climb；正常起飞在松刹车后平滑连续加油，避免 abrupt throttle；方向控制通过 rudder/aileron 随速度逐步建立，避免用刹车转向；到起飞姿态时逐步施加 back elevator；正爬升后才收轮/襟翼。
- FAA 同章强调 lift-off 后保持合适 pitch attitude，过多 back elevator 会导致过大 AOA、settle back 或 stall；initial climb 中以姿态控制爬升速度，takeoff power 通常保持到安全高度。
- T-38 公开 AFMAN 训练手册允许 static takeoff，但过程是 brakes hold、MIL power runup、检查不 creeping/不偏拉、松刹车后 MAX、起飞滑跑中确认发动机工作；约 135 KIAS 平滑后杆，145-150 KIAS 前轮离地，约 160 KIAS 飞离，正爬升后收轮。

与当前实现对照：
- `ManeuverExecutor::StartEngine()` 当前直接 `SetBrakes(true)` + `SetThrottle(1.0)` + `Start()`，把 engine start/runup 与 takeoff thrust 合并为满油门阶跃。
- `ConfigureForTakeoffRoll()` 直接松刹车并继续 `SetThrottle(1.0)`；`kTakeoffRoll` 每帧也继续写满油门。
- `EngineManager::SetThrottle()` 同时写 cmd 和 pos，绕过了 JSBSim FCS throttle position 的动态滞后；对涡桨尤其激进。
- JSBSim `FGPropulsion::InitRunning()` 内部 `SetEngineRunning()` 会把 `ThrottleCmd/ThrottlePos` 设为 1 并 `GetSteadyState()`，适配器再清 unindexed throttle，但 takeoff maneuver 首帧又写回满油门。

评估：
- 当前 1q 起飞逻辑不符合通用现实程序的“平滑加油、随速度建立控制、到 Vr 平滑抬轮”节奏。
- 对 T-38/F-16 这类喷气机，static runup 可以存在，但应显式建模为 runup/engine-check，再 release brake/advance thrust，而不是 adapter 初始化后的满推力残留。
- 对 L410/OV10/DHC6/PC7 这类涡桨，下一步优先在适配器/机动层做 takeoff power ramp、地面刹车保持和早期姿态/方向控制；源码层 RunIC 几何修正已被实验否定，不应继续调压缩比例。

## 适配器 HoldDown settle 结果（2026-06-02 14:36 CST）

### 新证据

在 `InitRunning(-1)` 后：
- 清理 FCS indexed/unindexed throttle cmd/pos。
- 清理 `FGPropulsion::in.ThrottleCmd/ThrottlePos`。
- 对地面初始状态执行 5 帧 `SetHoldDown(true)` settle，期间 throttle=0、brakes=1。

实测：

| 机型 | baseline 首帧 vc | HoldDown settle 首帧 vc | 后续结果 |
|------|------------------|--------------------------|----------|
| L410 | 20.8 kts | 0.0 kts | 1.03s hard crash，WOW=1 下沉到 h-agl=-5.02m |
| OV10 | 8.1 kts | 0.0 kts | 4.01s 翻转，roll≈176.6deg，WOW=0 |
| c172x | 可完成 | 可完成 | completed at 1144.6s |

### 结论

- 首帧速度跃迁不是 maneuver 层“来不及设置油门/刹车”；`FlightManager::Step()` 在第一帧 `Run()` 之前已经执行 AP 和 maneuver update。
- 首帧速度跃迁来自 JSBSim 初始化后第一步传播前的地面/推进状态不同步。`HoldDown` settle 能消除该首帧能量注入。
- L410 和 OV10 现在必须分开看：
  - L410：初始化弹跳消失后仍在 WOW=1 时持续下沉，指向起落架接触点、支柱压缩、地面反力与重量平衡问题。
  - OV10：初始化弹跳消失后仍起飞翻转，指向低速姿态/横侧控制、发动机/XML 类型或推力线问题。
- 因此，后续应保留“首帧弹跳修复”作为独立改动，再分别建立 L410 地面支撑测试和 OV10 起飞控制测试。

## L410 地面与推进链分层（2026-06-02 续）

### Gear 输入面

L410 的 gear kinematic channel 不是标准输入：

```xml
<input>/controls/gear/gear-down-cond</input>
<output>gear/gear-pos-norm</output>
```

因此只设置 `gear/gear-cmd-norm` 对 L410 不够。适配器应在属性存在时同步设置 `/controls/gear/gear-down-cond`。

### 初始高度合同

`ExternalKinematics` 默认 `position_frame=kEcef`，因此示例程序只写 `position_lla_deg_m.altitude_m` 而不设置 `position_frame=kLla` 时，高度不会生效。

L410 自带 `reset00.xml`：

```xml
<altitude unit="FT">7.1</altitude>
```

该值与 L410 起落架几何匹配。用 `altitude=0` 会造成约 7 ft 初始压缩和百万磅级地面反力；用 `7.1 ft` 后首帧压缩变为合理范围，地面支撑稳定。

### 推进链剩余问题

修正 gear input 和初始高度后，L410 不再 1s 穿地，也不再 3.9s 翻转 crash，而是稳定滑行到 2500s 上限但无法加速起飞：

- `vc≈6 kt`
- `WOW=1`
- `throttle=1.0`
- propeller RPM 基本在 `0~10 rpm`
- propulsive force 在 `0~650 lbs` 量级波动

`FGTurboProp` 将 `PropAdvance` 传给 `FGPropeller`，但 L410 仍不能从近零 RPM 自行建立正常转速。直接设置 propeller RPM=1600 会产生约 `32M lbs` 首帧推力，已否定。

结论：L410 当前剩余问题是涡桨/螺旋桨 spool-up 初始化或模型表格零 RPM 区间问题，不应再归入 RunIC/gear 地面弹跳。

### LLDB 调试数据

```
RunIC 后:  UVW=(0, 0, 0) fps     ✓
Run() 后:  UVW=(-0, -0, -35.18)  ← W=-35: 向上弹跳
Step 2:    UVW=(0, -0, -58.29)   ← 继续加速向上
Step 3:    UVW=(1, -0, -72.29)   ← 发散
```

## AP Profile 动态检测架构（2026-06-02 重构）

### 设计原则

**零机型名硬编码**——所有 lateral_interface / pitch_interface / fbw_subtype 通过 JSBSim property tree 动态检测。

### 检测优先级

```
has_fbw_override || has_roll_rate_command → kFbwRateCommand (f16)
has_own_autopilot                        → kOwnAutopilot      (c172x, c310)
has_generic_autopilot                    → kGenericAutopilotBridge (Concorde, B17, C130)
default                                  → kDirectSurface     (OV10, F4N, 737, ...)
```

### GenericAutopilot Bridge Guard

`ap/autopilot-roll-on` 从共享 `Autopilot.xml` 泄漏到未加载完整 AP 的机型（OV10）。Guard 规则：

> `kGenericAutopilotBridge` ∧ `!has_own_autopilot` ∧ `!has_fbw_override` ∧ `engine_count < 4` → 降级 `kDirectSurface`

### 能量管理分类

```
has_fbw                         → Fighter  (200m/s, 45°roll, 0.35min_thr)
has_gen_ap ∧ heavy ∧ !mixture   → Heavy jet (500m/s, 35°roll, 0.55min_thr)
heavy ∧ mixture                 → Heavy piston (85m/s, 25°roll)
mixture ∧ !heavy                → GA piston (50-65m/s, 30°roll)
default                         → struct defaults (45°roll)
```

### kOwnAutopilot 安全网

`kOwnAutopilot` 路径在 `UpdateOwnAutopilot()` 后调用 `UpdateDirectHeadingLateral()`——对没有功能性 XML autopilot 但有 `ap/heading_hold` 属性的机型（F4N）提供直接副翼控制。

## 起降问题根因（原始）

| 问题 | 根因 | 数据来源 |
|------|------|---------|
| c310 着陆坠毁 | 着陆 PD 增益未调优，高度/speed 振荡 | CSV: H=323→233→285m 反复 |
| f22 高空失控 | FBW pitch 链在爬升后段发散，50s 坠毁 | CSV: Hmax=357m, 14s 离地 |
| f16/737 超速 | 无最大速度保护，f16=834节，737=560节 | CSV |
| B17 推力不足 | 4×活塞 82 节 < Vr 91 节。需验证 per-engine 油门 | CSV |
| 引擎启动弹跳 | JSBSim 满油门+刹车不能阻止轻型飞机离地 | 所有 c172x 起飞 CSV |

## 调试方法

- **gtest**：仅用于合同/profile/smoke/鲁棒性测试（<10s per test）
- **CSV 示例程序**：飞行性能分析——`takeoff_land_csv`（时间序列）、`maneuver_sweep_csv`（机动扫描）
- **Python 分析**：`analyze_takeoff.py` 读 CSV，输出阶段时间线、速度/高度统计、matplotlib 图表
- **Release 预设**：JSBSim 仿真 ~6× 快于 debug（fd_ci ~5s vs ~34s）

## 阶段 8.1：重型机起飞稳定化发现（2026-06-03）

### 三个根因链

**根因 1：地面 heading hold → 滚转发散**
- `ConfigureForClimb()` 在 Vr 时立即启用 heading hold + `UpdateRollAnglePD`
- B747 仍在地面 (wow=1) 时，roll PD 产生满偏副翼 (-1.000)，速度 171+ kts
- Roll 从 0° → -70° 发散 → 无法抬头 → 70s 坠毁
- 修复：延迟 heading hold 到 `!WOW && agl > 5m`

**根因 2：Rotation ramp 在 AGL=10m 中断**
- 旧代码：`if (agl_m < 10.0) { ramp } else { climb rate }`
- 重型机 Iyy>1e7 → ramp_sec=6s，但 agl>10m 时 ramp 只走了 ~67%（4s/6s）
- Elevator 从 -0.300 瞬变到 +0.103（+0.4 阶跃）→ MD11 坠毁
- 修复：ramp 持续到完成，不再依赖 agl 阈值

**根因 3：Climb rate P 控制器振荡**
- `elevator = clamp(-0.05 * climb_err, -0.5, 0.3)` — 纯 P 控制器，无阻尼
- 重型机大惯量 → climb rate 变化慢 → P 控制器超调 → 全偏舵面振荡
- MD11: el 在 ±0.5 间跳动，agl 在 9.8↔78.8m 间振荡
- 修复：ramp 完成后启用 AP pitch hold（PD + pitch rate 阻尼），150m AGL 后交接到 altitude hold

### Iyy 分级体系

| log10(Iyy) | 类别 | ramp_sec | Vr factor | climb_rate | 机型 |
|------------|------|----------|-----------|------------|------|
| >7.0 | 重型运输 | 6.0s | 1.08 | 3.0 mps | B747, MD11, Concorde, XB-70 |
| >6.0 | 中型运输 | 4.0s | 1.15 | 4.0 mps | 737, C130 |
| ≤6.0 | 轻型 | 3.0s (default) | 1.20 | 5.0 mps (default) | c172x, f16 |

### 引擎/燃油兼容性问题（三机型地面不动）

**Concorde — collector tank 燃油耗尽**
- Tanks 13-16（collector tanks）各 46 lbs 容量/初始量
- 4× Olympus 引擎满推力消耗 ~11.7 lb/s → 46 lbs 仅撑 ~4 秒
- XML 注释："LP valve to emulate cross feed... not empty, to avoid stop at launch"
- 需要 cross-feed：737 引擎配置 `<feed>0</feed><feed>2</feed>`（多 feed 源），Concorde 只配单个 feed
- 可通过 `propulsion/tank[n]/contents-lbs` 属性实时补充

**C130 — 螺旋桨缺 gearratio**
- t56 涡轮 ~13800 RPM，t56_prop maxrpm=1400，需要 gearratio≈13.5:1
- t56_prop.xml 无 `<gearratio>` 标签 → 默认 1.0 → 螺旋桨不匹配
- 推力仅 1328 lbs（4×332），应为 ~40000 lbs
- 简化的 C_THRUST 表（仅 2 列 pitch=10/30 vs DHC6 的 17 列）

**L410 — cutoff-cmd + betarangeend**
- engTM601 有 `betarangeend=64`：油门<64% → 仅怠速功率
- L410 XML 暴露 `/controls/engines/engine[n]/cutoff-cmd` 属性
- 两个引擎交替正负振荡（eng0 和 eng1 交替输出/归零）
- 可能 cutoff-cmd 默认=1（燃油切断）导致引擎间歇运行

## 阶段 8.3 发现：MD11 fly-to/landing 四根因（2026-06-03）

### 根因 1：Iyy 属性名错误
- 代码读 `inertia/iyy-lbsft2`，JSBSim 只提供 `inertia/iyy-slugs_ft2`
- 所有飞机 XML 用 `unit="SLUG*FT2"`，JSBSim 内部属性单位为 slugs·ft²
- 结果：`pitch_moi_lbsft2=0` → Iyy 分级不生效 → MD11 不被识别为重型

### 根因 2：has_mixture 误判
- `FGFCS::CreateIndexedPropertyName("fcs/mixture-cmd-norm", num)` 为**所有**引擎类型创建 indexed 属性
- 仅靠 `pm->GetNode("fcs/mixture-cmd-norm")` 无法区分活塞/非活塞
- 修复：同时检查 `propulsion/magneto_cmd`（仅 `FGPiston` 创建，由 `HavePistonEngine` 控制）
- 影响 f16, f15, Concorde, C130 的分类——全部从"伪活塞"恢复为正确类型

### 根因 3：惯性速度含地球自转
- `GetInertialVelocityMagnitude()` 返回 ECEF 惯性速度，赤道附近 +~465 m/s
- IsManeuverComplete 用惯性速度计算最小转弯半径 → 捕获半径 = 63km（远超航路点距离 50km）
- Orbit 机动同样受影响
- 修复：改用 TAS (`velocities/vtrue-fps`)

### 根因 4：高空着陆 throttle 骤降
- kDecelerate 无差别 `engines_.SetThrottle(0.1)` 
- 8000m 高空 0.1 throttle + 微小 nose-up → 动能转势能 → zoom climb 到 10965m
- Pitch ±90°，roll ±180° 发散 → crash
- 修复：AGL>3000m 时用 AP altitude hold + 0.7 throttle 下降到 intermediate altitude
