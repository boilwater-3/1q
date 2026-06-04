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
