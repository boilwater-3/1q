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

### 副作用修复（2026-06-03）

has_mixture 修正和能量分类变更产生了额外修复效果：

| 机型 | 之前 | 之后 | 原因 |
|------|------|------|------|
| F80C | TIMEOUT (475m) | COMPLETED (1740s) | has_mixture 修正 → 不再落入 piston 分支 → fly-to 航路点距离正确 |
| T38 | 着陆 crash (pitch 66°) | COMPLETED (572s) | 能量分类修正 → 着陆下降受控，不再在高空 throttle 骤降 |

F80C 和 T38 的表观症状不同但根因相同：它们之前被误判为 piston（has_mixture=true），导致：
- ref_speed_mps=0（struct default）→ 航路点太近 → fly-to 瞬间完成 → throttle 骤降到 0.1
- 现在 has_mixture=false → 正确落入结构默认（kDirectSurface, 非 FBW）→ 能量管理正常

当前剩余问题（非引擎兼容性）：
- **XB-70** (CRASH)：JSBSim delta wing 模型俯仰不稳定。Vr 从 115→83 kts 可到达，但旋转后不可控抬头 69°+。**已知模型限制。**
- **OV10** ✅ (COMPLETED 1991s)：见 8.3c 发现

## 阶段 8.3b 发现：DHC6/CLmax/非指令升空（2026-06-03）

### 非指令升空检测
- 问题：XB-70、DHC6 在达到 Vr 前自然升空（delta wing 涡升力/涡桨高升力），卡在 kTakeoffRoll 永不完全
- 检测条件：`!WOW && AGL>10m && vc>25kts`（vc 门槛过滤 JSBSim 初始化弹跳——MD11 回归）
- 触发后：跳过 rotation ramp，直接进入 AP pitch hold 管理
- Heading hold 延迟到 vc ≥ Vr（非指令升空需 100% Vr 确保横侧操纵面有足够 authority）

### CLmax 分档（GetRotationSpeedKts）
- `kClMaxTakeoff = 1.6` 对涡桨（带襟翼）和 delta wing（涡升力）过保守
- 涡桨 (kTurboprop)：CLmax=2.0 → Vr 降低约 11%
- Delta wing (AR<2.5)：CLmax=2.5 → Vr 降低约 20%（XB-70: 115→83 kts）
- 属性名修复：`inertia/iyy-lbsft2` → `inertia/iyy-slugs_ft2`（EngineManager 中也存在同样错误）

### Delta wing 处理
- 跳过 pre-rotation：涡升力使升降舵后拉不必要且危险
- XB-70 Vr 可达但 JSBSim 模型仍 crash——中性升降舵下 pitch 从 0° 飙到 69°+
- 结论：JSBSim XB-70 模型有根本性俯仰不稳定，非控制逻辑可修复

### 高空着陆下降油门分档
- DHC6 turboprop throttle=0.70 在 3800m 仍产生足够推力维持平飞，降不下来
- 涡桨 descent throttle=0.50，涡扇=0.70

### max_steps 扩展
- 250000 (2500s) → 350000 (3500s)：DHC6 慢速涡桨需要 3031s 完成全部机动

## 阶段 8.3c 发现：OV10 引擎类型误分类（2026-06-03）

### 根因
- OV10 T76 引擎 XML 使用 `<turbine_engine>` 而非 `<turboprop_engine>`（注释写 turboprop 但根元素是 turbine）
- JSBSim `FGEngine::GetType()` 返回 `etTurbine`，导致 EngineManager 分类为 kTurbine
- 测试程序对 kTurbine 默认分配 8000m 巡航高度，远超 OV10 实际能力
- OV10 持续爬升到 6988m 后 TIMEOUT（3500s），但 takeoff 目标 8000m×0.95=7600m 始终未达

### 修复
- 测试程序的 turbine 巡航高度从型号名硬编码改为 `inertia/weight-lbs` 动态分类：
  - < 15k lbs → 4000m（OV10 ~6500 lbs）
  - < 100k lbs → 8000m（737 ~130k lbs）
  - ≥ 100k lbs → 10000m（B747 ~600k lbs）
- 同时移除 B17/C130 的型号名硬编码

### 副作用
- B747/MD11 巡航高度 8000→10000m，完成时间增加但仍在上限内
- B747: 2452→2901s, MD11: 2401→2530s

### JSBSim 引擎类型准确性
JSBSim 引擎类型由 XML 根元素决定，而非注释或实际行为：
- `<piston_engine>` → etPiston (FGPiston)
- `<turbine_engine>` → etTurbine (FGTurbine) — **包括被错误标记为 turbine 的 turboprop**
- `<turboprop_engine>` → etTurboprop (FGTurboprop)
- `<rocket_engine>` → etRocket (FGRocket)
- `<electric_engine>` → etElectric (FGElectric)

部分机型的引擎 XML 类型与实际不符（如 OV10 T76）。重量分类可作为通用 fallback。

## 飞行包线与航路点速度分析（2026-06-03）

### JSBSim 提供的运行时属性
| 属性 | 说明 | 可靠性 |
|------|------|--------|
| `aero/alpha-max-rad` | 失速迎角 | ⚠️ 仅 c172x/c172p/Concorde 有 XML 定义 |
| `aero/qbar-area` | 动压×翼面积 | ✅ |
| `forces/fwx-aero-lbs` | 气动阻力 | ✅ |
| `forces/fwz-aero-lbs` | 气动升力 | ✅ |
| `forces/lod-norm` | 升阻比 | ✅ |
| `propulsion/engine[n]/thrust-lbs` | 当前推力 | ✅ |
| `atmosphere/rho-slugs_ft3` | 当前空气密度 | ✅ |
| `limits/vne-kts` | 最大速度 | ⚠️ 大部分机型未定义 |

### 架构缺陷
1. `Waypoint` 无 `speed_mps` 字段 — 航路点不携带目标速度
2. `ExecuteFlyTo()` 用 `GetTrueSpeedMps()`（惯性速度含地球自转 ~465 m/s）作速度目标
3. 对 `ref_speed_mps=0` 的机型无 ref_speed cap → 能量管理用惯性速度作归一化因子
4. JSBSim 不提供飞行包线数据结构 — 需从力平衡运行时推导

### 可行的运行时速度边界估算
- **Vmin**：从当前升力/重量反推 `V_stall`，乘以 1.2 安全系数
- **Vmax**：从当前推力+阻力插值估计推力平衡速度，或 fallback 到机型分类默认值
- **Vcruise**：机型分类默认（活塞 50-65 m/s、涡桨 80 m/s、中型涡扇 230 m/s、重型涡扇 250 m/s、战斗机 200 m/s）

## Fly-Past Detection 机制（2026-06-04）

### 问题
- `WaypointManager::HasPassedActiveWaypoint()` 用 leg_start→target 向量与 target→aircraft 向量的点积判断是否飞越
- 当 `ExecuteFlyTo()` 开始时飞机已超过航路点（如 F80C 长距离起飞），`SetLegStartFromCurrentLocation()` 记录的 leg_start=飞机当前位置，target 在身后
- leg 向量指向后方，aircraft 向前移动 → `target·(aircraft - target)` = 负数 → `HasPassedActiveWaypoint()` 返回 false
- fly-to 永不完全

### 修复
在 `IsManeuverComplete()` 的 kFlyToWaypoint 分支添加飞越检测：
```cpp
if (elapsed_sec_ > 10.0) {
    double dist_m = wp_manager_.GetDistanceToActiveM();
    double heading_to_wp = wp_manager_.GetHeadingToActiveRad();
    double current_heading = adapter_.GetPropagate().GetEuler(3);
    double angle_off_nose = std::abs(NormalizeRad(heading_to_wp - current_heading));
    if (angle_off_nose > 2.0 && dist_m > effective_radius_m * 3.0) return true;
}
```
- 条件：航路点 >~115° 偏机头 + 距离 >3× 捕获半径 + 已飞行 >10s
- 对正常 fly-to 无影响（正常情况下飞机朝向航路点，角度 <90°）

## B747 着陆问题深入分析（2026-06-04）

### 症状
- kFinalDescent 阶段飞机以 150-160 kts (290 km/h) 进场，pitch=-12°, sink≈-7.6 m/s
- Flare 起始于 AGL≈20m，电梯 -0.08 至 -0.12（轻度拉杆）
- 飞机 2s 内降至 5m AGL，主轮触地（B747 主轮高~5m）
- 起落架压缩弹起（sink rate +0.8→+1.0 m/s）→ 飞机回到 7-8m AGL
- 浮空 30s+，滚转逐渐发散至 90°+ → pitch 俯冲到 -60° → 坠毁

### 根因链
1. **高空段（10000→3000m）空气密度低**：descent_thr=0.70 维持 220-230 kts，几乎不减速
2. **进近段（3000→200m）油门=0.10**：但重力分量平衡阻力，速度仅从 210→200 kts
3. **kFinalDescent 油门=0.42**：基于 sink rate 的油门控制（`0.25 + 0.05*sink_err`），维持 155-160 kts
4. **全襟翼升力≈重量**：150 kts 时 B747 翼面产生≈重量的升力，翼载低导致无法下降
5. **Flare 电梯有限**：即使 -0.25 满拉杆也需要 30s+ 才能减到 130 kts 以下

### 尝试的修复（均未成功）
| 方案 | 结果 |
|------|------|
| 进近速度 Vr×1.0 (145 kts) | kApproach 高空分支不依赖 approach_speed，无效 |
| 高空下降油门 0.30 | 仍维持 225→200 kts，高空空气稀薄 |
| 高空下降高度 1500m | 减少高空段但低空段减速仍不足 |
| sink rate 闭环 flare 控制器 | 饱含在 -0.12 极限（sink error 过大） |
| bounce recovery (sink>0→+0.10) | 浮空但滚转仍然发散 |

### 结论
B747 着陆是 **进近阶段设计问题**，非 flare 阶段可单独修复。进出方案需要：
- 在高空段增加盘旋减速（orbit at 3000m）
- 或使用减速板（需 JSBSim 模型支持）
- 或引入襟翼管理的多段放襟翼进近

### XML 属性驱动配置方案讨论（2026-06-04）
用户提出：硬编码 MOI 阈值 (log10(Iyy) > 7.0 = 重型) 后期容易出现机动异常。

建议方案：扩展 `AircraftControlProfile` 支持 JSBSim 属性树读取：
```cpp
// 优先读取 XML 配置值，不存在时 fallback 到 MOI 分类默认值
auto* node = pm->GetNode("guidance/approach-speed-mps");
land_approach_speed_mps_ = node ? node->getDoubleValue() : Vr * 1.3;
```
每个机型 XML 可定义自己的指导属性，C++ 代码优先读取，不存在时用动态检测的分类默认值。

## B747 着陆最终修复结论（2026-06-04）

### 新根因拆分
- flare 阶段曾出现 pitch/elevator 失控，但清理 AP hold 后可排除为主根因。
- touchdown AGL 阈值需要 XML 配置：B747 主轮 WOW 可在机身 AGL 3m 以上出现，默认 3m 对高起落架机型偏低。
- 真正阻塞完成的是 decelerate 阶段中低空速度窗口：速度低于旧硬编码 `200m/s` 后不再 orbit，也不保持 pattern altitude，导致 B747 在 `200kt+` 时贴近地面并长时间 WOW。

### 已验证修复
- 默认开启 `landing_high_descent_orbit`。
- 当 `vc_mps > landing_approach_speed_mps * 1.35` 时，不进入直线低空减速；继续围绕着陆点 orbit，并由 AP altitude hold 保持 `landing_pattern_agl_m`。
- B747 XML 配置 `landing-pattern-agl-m=450`、`landing-touchdown-agl-m=5.5`、`landing-approach-flaps-norm=0.25`、`landing-final-flaps-norm=0.75`、`landing-final-throttle-cap=0.05`。
- 验证命令 `takeoff_land_csv B747 /tmp/1q_fd/b747_stage10_mid_speed_orbit.csv` 完成，landing 诊断 `completed`，最低 AGL `4.10m`，无 crash。

## 阶段 11 研究：XML 配置契约与硬编码清理（2026-06-04）

### XML guidance 属性现状
- 当前 aircraft XML 中已有 `guidance/*` 不只 B747：
  - `B747/B747.xml`：landing/approach override。
  - `c172x/c172x.xml`：`guidance/roll-angle-limit`、`guidance/roll-rate-limit`。
  - `global5000/global5000.xml`：`guidance/roll-angle-limit`、`guidance/roll-rate-limit`。
  - `c310/c310ap.xml`：`guidance/wp-heading-deg` 被 XML AP 测试使用，但不是当前 C++ profile override。
- 现有 C++ `ApplyXmlProfileOverrides()` 覆盖 energy、rotation、landing 三组属性；测试目前只验证 B747 landing override。
- `JsbsimAdapter::ConfigureIntegrators()` 已改为 XML 存在 roll limit/rate limit 时不覆盖，因此 c172x/global5000 可用于验证“非 B747 XML 属性优先级”。

### 当前硬编码分布
- `Autopilot.cpp`
  - `ApplyEnergyDefaults()` 仍按 FBW/heavy/medium/piston/light fallback 分档设置速度、pitch、roll、throttle。
  - `ApplyRotationDefaults()` 仍按 `log10(Iyy)>7/>6` 设置 rotation ramp/climb rate。
- `Maneuver.cpp`
  - takeoff：非指令升空门槛、rotation handoff、gear/flap retract、climb pitch recovery 仍为控制律常量。
  - landing：虽然 landing profile 已覆盖一部分，但 approach/final/flare 控制律仍有 `1.35/1.15/1.3/1.05`、`-3deg`、`0.3/0.005/0.1/0.6`、`30m` flare scale、touchdown rollout speed 等常量。
  - flare 仍用 `log10(Iyy)>7` 区分 heavy transport，这应迁移为 profile capability 或显式 XML override。
- `EngineManager.cpp`
  - CLmax、delta wing AR 阈值、Vr factor、approach speed、climb pitch 仍硬编码。
- `takeoff_land_csv.cpp`
  - cruise altitude、waypoint distance、landing target、skip 机型和型号名分支仍承担测试场景策略；这不应混入核心 flight_dynamic，但也需要独立配置化。

### 风险判断
- 直接把所有数值放进 XML 会把算法参数和 aircraft capability 混在一起，后期更难验证。
- 更稳妥的分层：
  1. 物理/单位常量保留在代码中。
  2. 通用控制律参数先命名化并用测试保护。
  3. aircraft-specific 能力参数进入 `AircraftControlProfile` 并允许 XML override。
- 下一阶段应先补“非 B747 XML override 测试”和属性表，再迁移更多硬编码。

## 阶段 11 执行发现：非 B747 XML guidance 契约（2026-06-04）

### 修正的优先级缺口
- `Autopilot` 原先在 `ApplyEnergyDefaults()` 之前读取 `guidance/roll-angle-limit`。
- 对 `c172x` 这类已有 aircraft XML roll limit 的机型，后续 GA 动态默认值会把 `max_roll_angle_deg` 覆盖回 `30 deg`，导致 XML priority 实际未生效。
- 修正后顺序为：
  1. `ApplyEnergyDefaults()`
  2. structural roll limit 映射
  3. `ApplyXmlProfileOverrides()` 的显式 profile override

### 避免的误覆盖
- 初次无条件移动 roll-limit override 后，`ProfileSnapshotTest` 发现 f16、Concorde、B17、C130、c310 的 `max_roll_angle_deg` 被意外改变。
- 根因：`systems/Autopilot.xml` 和 adapter fallback 也会创建 `guidance/roll-angle-limit`，不能仅凭属性存在判定为 aircraft-specific profile tuning。
- 最终边界：
  - adapter fallback `0.785 rad` 不映射为 profile override。
  - FBW 机型不从 generic roll limit 改写 dynamic profile。
  - 重型机型不从 generic `systems/Autopilot.xml` roll limit 改写 dynamic profile。
  - 若 FBW/重型机型确实需要 XML 驱动 profile bank limit，应使用更明确的 `guidance/max-roll-angle-deg`。

### 已验证合同
- 新增 `FlightDynamicTest.AutopilotPreservesC172xXmlRollGuidanceOverrides`：
  - 验证 `c172x` XML `guidance/roll-angle-limit=0.523` 与 `guidance/roll-rate-limit=0.174` 被 adapter 保留。
  - 验证 `c172x` profile `max_roll_angle_deg` 来自 `0.523 rad * 180/pi * 0.7`，不是 GA 默认 `30 deg`。
- `docs/finding/jsbsim_aircraft_control_contract.md` 已补充 guidance profile 属性表和覆盖优先级。

### 验证结果
- `cmake --build --preset llvm-ninja-release-local --target 1q_fd_tests`：通过。
- focused gtest：
  - `FlightDynamicTest.AutopilotPreservesC172xXmlRollGuidanceOverrides`
  - `FlightDynamicTest.AutopilotReadsB747LandingGuidanceOverrides`
  - `AircraftProfiles/ProfileSnapshotTest.MatchesExpectedProfile/*`
  - 结果：9/9 passed。
- full `1q_fd_tests`：162 tests，159 passed，3 skipped。

## 阶段 12a 执行发现：Maneuver landing/flare capability 迁移（2026-06-04）

### 清理目标
- `Maneuver.cpp` 的 flare 入口和 flare 控制律仍直接读取 `pitch_moi_lbsft2` 并用 `log10(Iyy)>7` 判断是否走 heavy transport flare law。
- 这属于 aircraft capability 判断，不应散落在 landing maneuver 控制流程里。

### 已迁移边界
- 新增 `AircraftControlProfile::landing_heavy_flare`。
- `ApplyRotationDefaults()` 仍按当前兼容逻辑用 `log10(Iyy)>7` 推导默认值，保证未配置 XML 的机型行为不变。
- `ApplyXmlProfileOverrides()` 支持 `guidance/landing-heavy-flare` 显式覆盖。
- `Maneuver.cpp` 只读取 `profile.landing_heavy_flare`，不再直接读取 `pitch_moi_lbsft2` 或 `log10()`。
- B747 aircraft XML 显式声明：
  ```xml
  <property value="1"> guidance/landing-heavy-flare </property>
  ```

### 兼容性判断
- 当前 profile snapshot 样本中，旧 `log10(Iyy)>7` heavy flare 行为只覆盖 Concorde；B17 和 C130 虽然在 energy defaults 中被分到 heavy/transport，但 `Iyy` 分别为 `2.5e5` 和 `2.38552e6`，旧 flare law 不会进入 heavy 分支。
- Snapshot 因此锁定 `landing_heavy_flare == true` 仅对 Concorde 成立，避免把 energy-heavy 与 flare-heavy 混为一类。
- B747 不在当前 snapshot 参数集中，使用专门的 B747 XML override 测试覆盖。

### 验证结果
- `cmake --build --preset llvm-ninja-release-local --target 1q_fd_tests takeoff_land_csv`：通过。
- focused gtest：
  - `FlightDynamicTest.AutopilotReadsB747LandingGuidanceOverrides`
  - `FlightDynamicTest.AutopilotPreservesC172xXmlRollGuidanceOverrides`
  - `AircraftProfiles/ProfileSnapshotTest.MatchesExpectedProfile/*`
  - `FlightDynamicTest.OrbitManeuver`
  - `FlightDynamicTest.FlyToMultipleWaypointsThenOrbit`
  - 结果：11/11 passed。
- `takeoff_land_csv B747 /tmp/1q_fd/b747_stage12_heavy_flare.csv`：completed at `1475.0s`，landing 诊断 `outcome=completed`、`alt_min=4.10202m`、`crashed=no`。
- full `1q_fd_tests`：162 tests，159 passed，3 skipped。

## 阶段 12b 执行发现：Maneuver landing 控制律常量命名化（2026-06-04）

### 清理目标
- 阶段 12a 后，landing maneuver 行为已经通过 profile capability 控制 heavy flare，但 landing 控制律中仍有大量未命名常量。
- 本阶段只做命名化和语义分组，不把控制律参数移入 XML，避免把算法参数误当 aircraft capability。

### 已命名化范围
- Decelerate 阶段：
  - high-speed orbit 进入因子 `1.35`
  - 高空 orbit 最小速度 `120 m/s`
  - orbit 半径下限/速度比例 `3000m/20s`、pattern orbit `2000m/15s`
  - approach 进入速度因子 `1.15`
- Approach / FinalDescent：
  - pattern 高度 band `500m`
  - FPA 目标 `-3 deg`
  - speed/FPA elevator gains 和 throttle gains
  - final-too-fast 因子 `1.30`
  - final throttle cap speed factor `1.05`
  - sink-rate throttle target/base/gain/clamp
- Flare / Rollout：
  - flare altitude scale `max(15m, vc*0.60)`
  - heavy/standard flare scale `30m`
  - bounce/float recovery AGL、sink、delay、电梯阈值
  - touchdown/rollout completion speed gates

### 风险边界
- 这是行为保持修改：公式、阈值和 XML override 都未改变。
- `ConfigureForApproach()` 的 `Vr*1.3` 仍属于 approach speed derivation fallback；它更接近 aircraft capability/EngineManager 边界，留到阶段 12c 统一判断，不在 12b 中迁移。
- Takeoff/climb 数字常量未纳入本轮，避免扩大阶段范围。

### 验证结果
- `cmake --build --preset llvm-ninja-release-local --target 1q_fd_tests takeoff_land_csv`：通过。
- focused gtest：11/11 passed。
- `takeoff_land_csv B747 /tmp/1q_fd/b747_stage12b_named_landing_constants.csv`：completed at `1475.0s`，landing `outcome=completed`、`alt_min=4.10202m`、`crashed=no`。
- full `1q_fd_tests`：162 tests，159 passed，3 skipped。

## 阶段 12c 执行发现：EngineManager 参数边界确认（2026-06-04）

### 审计范围

`EngineManager.cpp` 中 3 个 public 方法的参数来源和分类：

| 方法 | 参数 | 当前来源 | 类别 |
|------|------|----------|------|
| `GetRotationSpeedKts()` | CLmax (1.6/2.0/2.5) | 引擎类型 + AR 检测 | **aircraft capability** |
| `GetRotationSpeedKts()` | AR 阈值 (2.5) | 气动分类常量 | **通用分类边界** |
| `GetRotationSpeedKts()` | Vr factor (1.08/1.10/1.15/1.20) | 引擎类型 × Iyy 分档 | **capability + safety margin** |
| `GetRotationSpeedKts()` | Iyy 阈值 (1e6, 1e7) | 重量级分类 | **与 profile rotation defaults 共享** |
| `GetRotationSpeedKts()` | Vr floor (40 kts) | 安全门槛 | **通用安全常量** |
| `GetRotationSpeedKts()` | Fallback Vr (50 kts) | 兜底 | **输入验证 fallback** |
| `GetDefaultApproachSpeedMps()` | 28/41/62/80 m/s | 引擎类型分档 | **最后兜底 fallback** |
| `GetClimbPitchDeg()` | 10/15/10/10 deg | 引擎类型分档 | **aircraft capability** |

### 参数分类决策

#### 第一类：aircraft capability（建议迁移到 profile）

| 参数 | 当前值 | 建议 profile 字段 | 优先级 |
|------|--------|-------------------|--------|
| CLmax takeoff | 1.6/2.0/2.5 | `takeoff_cl_max` (0=auto-detect) | 中 |
| Vr factor | 1.08/1.10/1.15/1.20 | `rotation_vr_factor` (0=auto-detect) | 中 |
| Climb pitch | 10/15/10 deg | `climb_pitch_deg` (0=engine default) | 低 |

**迁移策略**：保持 engine type + AR 检测作为 fallback，仅在 XML 显式声明时覆盖。这样现有 18 机型自动继承当前行为，B747/Concorde 等特殊机型可 XML 微调。

#### 第二类：通用分类常量（建议仅命名化，不 XML 化）

| 参数 | 当前值 | 理由 |
|------|--------|------|
| AR 阈值 | 2.5 | 气动学通用定义，非 per-aircraft 调参 |
| Iyy 阈值 | 1e6, 1e7 | 重量级分类边界，与 Autopilot rotation defaults 已有对应 |
| Vr floor | 40 kts | 安全硬约束 |
| Fallback Vr | 50 kts | 输入验证兜底 |

#### 第三类：控制律 fallback（保留为 last-resort）

| 参数 | 理由 |
|------|------|
| Approach speed 28/41/62/80 m/s | Maneuver 层已优先使用 profile `landing_approach_speed_mps` → 命令行参数 `approach_speed_mps` → Vr×1.3 → 此 fallback。实际触发频率极低。 |

### 已添加命名常量

`EngineManager.cpp` 匿名命名空间新增：
- CLmax: `kClMaxTakeoffDefault` (1.6), `kClMaxTakeoffTurboprop` (2.0), `kClMaxTakeoffDeltaWing` (2.5)
- AR: `kDeltaWingArThreshold` (2.5)
- Iyy: `kIyyHeavyThreshold` (1e7), `kIyyMediumThreshold` (1e6)
- Vr factor: `kVrFactorHeavyTurbine` (1.08), `kVrFactorMediumTurbine` (1.15), `kVrFactorLightTurbine` (1.20), `kVrFactorPiston` (1.10), `kVrFactorDefault` (1.15)
- Safety: `kMinVrKts` (40.0), `kFallbackVrKts` (50.0), `kClMaxLowerBound` (1.0), `kClMaxUpperBound` (3.5)
- Approach speed: `kApproachSpeedPistonMps` (28.0), `kApproachSpeedTurbopropMps` (41.0), `kApproachSpeedTurbineMps` (62.0), `kApproachSpeedRocketMps` (80.0), `kApproachSpeedDefaultMps` (36.0)
- Climb pitch: `kClimbPitchPistonDeg` (10.0), `kClimbPitchTurbineDeg` (15.0), `kClimbPitchTurbopropDeg` (10.0), `kClimbPitchDefaultDeg` (10.0)

### 已添加诊断日志

- `GetRotationSpeedKts()`：`spdlog::debug` 输出 Vr、V_stall、CLmax、Vr factor、Iyy、AR、weight、wing area、rho
- `GetRotationSpeedKts()`：`spdlog::warn` 当 CLmax 超出 [1.0, 3.5] 或输入无效
- `GetDefaultApproachSpeedMps()`：`spdlog::debug` 输出速度和类型
- `GetClimbPitchDeg()`：`spdlog::debug` 输出俯仰角和类型

### 已添加 contract 测试

`EngineManagerContractTest` 参数化测试，覆盖 4 个代表机型：

| 机型 | 引擎类型 | CLmax | Vr factor | Iyy 分类 |
|------|----------|-------|-----------|----------|
| c172x | piston | 1.6 | 1.10 | light |
| DHC6 | turboprop | 2.0 | 1.15 | n/a |
| B747 | turbine | 1.6 | 1.08 | heavy (3.31e7) |
| XB-70 | turbine (delta) | 2.5 | 1.20 | light (delta) |

每个机型 3 个断言：`RotationSpeedReasonable`（Vr 范围 + 类型匹配）、`ClimbPitchMatchesEngineType`（俯仰角正确）、`ApproachSpeedFallbackReasonable`（进近速度正确）。

### 验证结果

- `cmake --build --preset llvm-ninja-release-local --target 1q_fd_tests`：通过。
- `EngineManager*` focused gtest：12/12 passed。
- 回归测试（B747 XML override + c172x roll + snapshot + orbit + fly-to）：11/11 passed。
- full `1q_fd_tests`：174 tests，171 passed，3 skipped。
- `takeoff_land_csv B747`：completed at `1475.0s`，landing `outcome=completed`、`alt_min=4.10202m`、`crashed=no`，无回归。

### 建议后续路径

1. **下一个阶段**（12d）：示例程序策略外移规划（`takeoff_land_csv.cpp` 的 cruise altitude、waypoint distance、landing target、skip/model 分支）。
2. **CLmax/Vr factor 迁移**：当前命名化+合同测试已足够。若后续发现某机型需要调 CLmax/Vr factor，在 profile 中新增字段并保持 engine type fallback。
3. **Iyy 阈值统一**：EngineManager 和 Autopilot 共享相同的 Iyy 阈值但各自独立读取。不急于统一——两者在语义上独立（一个用于 Vr factor，一个用于 rotation ramp/climb rate），强行共享一个字段会增加耦合。
4. **Climb pitch**：低优先级——当前 10/15 deg 覆盖所有机型且无已知问题。


## 阶段 12d+12e 执行发现：物理推导包线 + 场景外移（2026-06-04）

### 物理包线推导

速度包线现在由 V_stall（从 aircraft XML 属性计算）推导：

```
V_stall = sqrt(2 × weight / (ρ × wing_area × CLmax))
min_speed = V_stall × stall_margin (1.20-1.30)
cruise_speed = V_stall × cruise_factor (2.8-3.5，取决于引擎类型)
max_speed = V_stall × max_factor (3.5-5.5)
```

每架飞机的包线不同——c172x (V_stall=25.7m/s) 和 c310 (V_stall=34.8m/s) 自然得到不同的巡航速度。类别标志只决定乘数。

### 升限

保持类别默认值（FBW 15200m, 重型涡扇 13700m, 中型 12500m, 涡桨 7600m, 活塞 4300m），支持 XML `guidance/ceiling-m` 覆盖。ExecuteTakeoff/ExecuteFlyTo/ExecuteOrbit 统一 clamp。

### 场景外移

`takeoff_land_csv.cpp` 移除所有型号名硬编码。ScenarioConfig 结构体收纳场景参数，MakeScenario() 用 profile 布尔值推导默认值。

### 验证
- 174 tests, 171 passed, 3 skipped
- 15/16 机型 takeoff_land_csv 完成
- MD11 abort 待确认为 baseline issue


## 阶段 13 规划：基于物理推导的硬编码消减（2026-06-04）

### 背景

基于 `docs/finding/hardcoded_parameter_audit.md` 全 31 机型 JSBSim XML 审计结论。

核心发现：
- JSBSim 属性树直接暴露的物理属性只有 4 个：weight、wing_area、wingspan、Iyy
- 引擎推力/功率数据在 XML 文件中存在（milthrust/maxhp），但不在属性树中
- 31 机型全无 aero/alpha-max-rad 和 limits/vne-kts
- 只有 3 机型有 guidance XML 覆盖（B747、c172x、global5000）

### 分阶段执行

13a: 获取推重比（运行时测量满油门静态推力）
13b: 推重比+翼载连续化速度包线（替代 5 档硬编码 factor）
13c: log10(Iyy) 连续化 rotation ramp/climb rate
13d: wing_loading 替代 speed_energy_priority；V_stall×1.3 替代 approach_speed fallback 引擎类型表
13e: CLmax / climb_pitch / Vr_factor 加 XML override
13f: B747 着陆回归修复

### 预期改进

速度包线从"所有活塞都 ×2.8"变为"每架飞机根据推重比和翼载独立计算"。
例如 c310（推重比 0.18 hp/lb, 翼载 31）和 c172x（0.11, 14）会得到不同的 cruise_speed。
