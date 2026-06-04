# JSBSim 代表机型控制合同初稿

> 日期：2026-05-29
> 范围：阶段 2 首批 XML 审查，覆盖 `f16`、`f22`、`c310`、`c172x`、`Concorde`、`f15`。

## 1. 结论摘要

当前 6 个代表机型不能用同一个“写 `fcs/*-cmd-norm` 就等价于舵面位置”的假设处理。

| 机型 | 横向接口 | 纵向接口 | 油门接口 | 注入系统 | 输出副作用 |
|------|----------|----------|----------|----------|------------|
| `f16` | FBW roll-rate command | FBW pitch/g-load scheduler | 单发 `fcs/throttle-cmd-norm` | `Navigation`、`GNCUtilities`、`Autopilot` | datalog output 在注释块内 |
| `f22` | FBW roll-rate integrator + actuator | FBW pitch-rate/g-load integrator | 双发 indexed throttle，另有 thrust normalize | `Navigation`、`GNCUtilities`、`Autopilot` | datalog output 在注释块内 |
| `c310` | native AP + direct surface 混合 | native AP + direct surface 混合 | 双发 common throttle，indexed mixture | `GNCUtilities` + `c310ap` | active CSV/socket |
| `c172x` | generic AP/native `c172ap` + direct surface 混合 | generic AP/native `c172ap` + direct surface 混合 | 单发 throttle + mixture system | `Navigation`、`GNCUtilities`、`Autopilot`、`c172ap` | active CSV/socket |
| `Concorde` | direct surface | direct surface | 四发 common throttle | `Navigation`、`GNCUtilities`、`Autopilot` | 未见 active output |
| `f15` | direct surface | direct surface | 双发 throttle | `Navigation`、`GNCUtilities`、`Autopilot` | 未见 active output |

阶段 2 后续代码化 profile 时，至少需要把 `f16` 与 `f22` 拆成不同 FBW subtype：`f16` 的 `fcs/aileron-cmd-norm` 进入 roll-rate PID，再生成舵面；`f22` 的 `fcs/aileron-cmd-norm` 进入 roll-rate command/filter/integrator/actuator 链，且存在独立 `fcs/roll-cmd` 函数和 `fcs/aileron-act`。

## 2. 控制链合同

### f16

来源：`third_party/jsbsim/aircraft/f16/f16.xml`。

| 通道 | producer/consumer 链 | 语义 |
|------|----------------------|------|
| Roll | `fcs/aileron-cmd-norm` + `-fcs/roll-rate-norm` -> `fcs/roll-trim-error` -> `fcs/roll-rate-pid` -> `fcs/roll-rate-command` -> `fcs/aileron-pos-rad`/flaperon | `aileron-cmd-norm` 更像 roll-rate command，不是直接舵面位置 |
| Pitch | `fcs/elevator-cmd-norm` + trim -> limiter -> alpha scheduler -> pitch-rate/g-load PID -> `fcs/elevator-pos-rad` | elevator command 被 FBW g-load/pitch-rate/alpha 限制器重写 |
| Yaw | `fcs/rudder-cmd-norm` + yaw-rate/yaw-load -> `fcs/yaw-load-pid`/scheduler -> `fcs/rudder-pos-rad` | rudder command 不是纯直通 |
| Throttle | `fcs/throttle-cmd-norm` -> `fcs/throttle-pos-norm` | 单发直通放大 |

关键风险：现有 C++ profile 把 `has_fbw_override` 或 `has_roll_rate_command` 归到 `kFbwRateCommand` 是合理方向，但不能把 f16 的 command 当 direct surface。

### f22

来源：`third_party/jsbsim/aircraft/f22/f22.xml`。

| 通道 | producer/consumer 链 | 语义 |
|------|----------------------|------|
| Roll | `fcs/aileron-cmd-norm` + trim -> limiter/filter -> `fcs/roll-rate-cmd` -> rate error -> integrator -> `fcs/roll-cmd` -> `fcs/aileron-act` -> aileron surfaces | roll command 链比 f16 更深，核心输出是 actuator 后的舵面 |
| Pitch | `fcs/elevator-cmd-norm` + trim -> pitch command limiter -> `fcs/pitch-rate-cmd` -> integrator -> `fcs/pitch-cmd-summer` -> elevator actuator | pitch command 被 pitch-rate/g-load 表和 integrator 管理 |
| Yaw | `fcs/rudder-cmd-norm` -> yaw-rate command/filter/integrator -> `fcs/yaw-cmd` -> rudder actuator | yaw command 进入 rate/integrator 链 |
| Throttle | `fcs/throttle-cmd-norm`、`fcs/throttle-cmd-norm[1]` -> filters -> `fcs/throttle-pos-norm`、`[1]`; `fcs/thrust-norm` 参与气动/推力向量项 | 双发 indexed，且有 thrust normalize |

关键风险：f22 当前 `FlyToWaypoint` 失败不能只按“FBW 增益不够”处理；它同时涉及 trim 异常、FBW integrator 初态、速度/高度包线和任务几何。

### c310

来源：`third_party/jsbsim/aircraft/c310/c310.xml`。

| 通道 | producer/consumer 链 | 语义 |
|------|----------------------|------|
| Roll | `ap/aileron_cmd` + `fcs/aileron-cmd-norm` + trim -> surface rad/norm | native AP 与 C++ direct command 叠加 |
| Pitch | `ap/elevator_cmd` + `fcs/elevator-cmd-norm` + trim -> elevator rad/norm | native AP 与 C++ direct command 叠加 |
| Yaw | `ap/rudder_cmd` + `fcs/rudder-cmd-norm` + trim -> rudder rad/norm | native AP 也可写 rudder |
| Throttle/Mixture | indexed mixture `fcs/mixture-cmd-norm[0..1]`; throttle 未发现 aircraft XML 消费 `fcs/throttle-cmd-norm[0..1]` | 双发 common throttle + indexed mixture |

关键风险：它有 `autopilot file="c310ap"`，但不是项目统一 `system file="Autopilot"`；profile 探测不能只看 `ap/autopilot-roll-on`，还要记录 native AP 文件和 `ap/*_cmd` consumer。

### c172x

来源：`third_party/jsbsim/aircraft/c172x/c172x.xml`。

| 通道 | producer/consumer 链 | 语义 |
|------|----------------------|------|
| Roll | `ap/aileron_cmd` + `ap/roll-cmd-norm-output` + `fcs/aileron-cmd-norm` + trim -> left/right aileron | generic AP、native AP 和 direct command 叠加 |
| Pitch | `ap/elevator_cmd` + `fcs/elevator-cmd-norm` + trim -> elevator | native AP 与 direct command 叠加 |
| Yaw | `fcs/rudder-cmd-norm` + trim -> rudder | direct surface |
| Throttle/Mixture | mixture system 输出 `fcs/mixture-cmd-norm`; throttle 单发 | 需区分 mixture 与 throttle |

关键风险：c172x 同时包含项目 `system file="Autopilot"` 与 `autopilot file="c172ap"`；后续 `NativeAutopilotBridge` 和 `GenericAutopilotBridge` 必须明确优先级。

### Concorde

来源：`third_party/jsbsim/aircraft/Concorde/Concorde.xml`。

| 通道 | producer/consumer 链 | 语义 |
|------|----------------------|------|
| Roll | `fcs/aileron-cmd-norm` + trim -> actuator -> `fcs/aileron-surface` -> left/right aileron | direct surface |
| Pitch | `fcs/elevator-cmd-norm` + trim -> actuator -> `fcs/elevator-surface` -> elevator | direct surface |
| Yaw | `fcs/rudder-cmd-norm` -> nose-wheel steering schedule -> `fcs/rudder-pedal-norm` + trim + yaw damper -> rudder actuator | C++ 应写 `rudder-cmd-norm` 命令入口，`rudder-pedal-norm` 是 XML 中间输出 |
| Throttle | four Olympus engines; 未发现 aircraft XML 消费 `fcs/throttle-cmd-norm[0..3]` | 四发 common throttle |

关键风险：当前 Concorde `Orbit` 失败表现为高度/能量问题，横向 direct surface 不是唯一主因；yaw path 经过 `rudder-cmd-norm -> rudder-pedal-norm -> rudder actuator`，不应绕过上游命令入口直接写中间量。

### f15

来源：`third_party/jsbsim/aircraft/f15/f15.xml`。

| 通道 | producer/consumer 链 | 语义 |
|------|----------------------|------|
| Roll | `fcs/aileron-cmd-norm` + trim -> kinematic -> aileron rad | direct surface |
| Pitch | `fcs/elevator-cmd-norm` + trim -> kinematic -> elevator rad | direct surface |
| Yaw | `fcs/rudder-cmd-norm` + trim + yaw damper -> rudder rad | direct surface with damper |
| Throttle | two F100 engines | 双发控制 |

关键风险：f15 可作为 direct-surface jet baseline，但也注入了项目 `Autopilot`，后续测试要区分 XML direct control 与项目 guidance/AP 写入。

## 3. 下一步实现要求

1. `AircraftControlProfile` 需要从布尔探测扩展为可解释合同：lateral subtype、pitch subtype、yaw input、throttle indexing、native AP 文件、project injected AP、output side effects。
2. f16/f22 的 FBW subtype 必须拆开，至少区分 `fbw_roll_rate_pid` 与 `fbw_rate_integrator_actuator`。
3. c310/c172x 的 native AP consumer 要进入 profile，不应只靠 `ap/autopilot-roll-on` 判断 generic AP。
4. profile 中 `yaw_input_property` 应表示 C++ 应写入的命令入口；Concorde 应为 `fcs/rudder-cmd-norm`，不要把 XML 中间输出 `fcs/rudder-pedal-norm` 当外部控制输入。
5. `indexed_throttle` 应表示 aircraft XML 实际消费 indexed throttle 输入；当前首批样本中只有 `f22` 需要 C++ 同步写 `fcs/throttle-cmd-norm[1]`。
6. XML output 需要在 adapter/test fixture 层统一禁用或重定向；c172x/c310 是当前 active output 的重点。

## 4. 阶段 9 更新：快照测试确认的合同修正

> 日期：2026-05-31
> 来源：`AircraftProfiles/ProfileSnapshotTest` 参数化快照测试，8 机型 × 11 字段精确匹配。

### 4.1 关键修正

| 原假设（阶段 2） | 快照修正 | 原因 |
|------------------|---------|------|
| `f15` = kDirectSurface | **kGenericAutopilotBridge** | 项目注入 `Autopilot.xml` 创建 `ap/autopilot-roll-on` |
| `B17` = kDirectSurface | **kGenericAutopilotBridge** | 同上 |
| `C130` = kDirectSurface | **kGenericAutopilotBridge** | 同上 |
| `f22` pitch = kFbwScheduled | **kNativeAutopilot** | 项目 `Autopilot.xml` 设置 `has_generic_autopilot=true` |
| `has_mixture` 仅活塞机型 | **全部机型均为 true** | JSBSim 自动创建 `fcs/mixture-cmd-norm` |
| `Concorde` 横向 = direct surface | **kGenericAutopilotBridge** | 项目 `Autopilot.xml` 提供 `ap/autopilot-roll-on` |

### 4.2 8 机型 Profile 快照（快照测试 2026-05-31 确立）

| 字段 | f16 | f22 | c172x | c310 | f15 | Concorde | B17 | C130 |
|------|-----|-----|-------|------|-----|----------|-----|------|
| **lateral_interface** | kFbwRateCommand | kFbwRateCommand | kOwnAutopilot | kOwnAutopilot | kGenericAutopilotBridge | kGenericAutopilotBridge | kGenericAutopilotBridge | kGenericAutopilotBridge |
| **pitch_interface** | kNativeAutopilot | kNativeAutopilot | kNativeAutopilot | kNativeAutopilot | kNativeAutopilot | kNativeAutopilot | kNativeAutopilot | kNativeAutopilot |
| **fbw_subtype** | kRollRatePid | kRateIntegratorActuator | kNone | kNone | kNone | kNone | kNone | kNone |
| **has_own_autopilot** | false | false | true | true | false | false | false | false |
| **has_generic_autopilot** | true | true | true | false | true | true | true | true |
| **has_fbw_override** | true | false | false | false | false | false | false | false |
| **has_roll_rate_command** | true | true | false | false | false | false | false | false |
| **has_aileron_command** | true | true | true | true | true | true | true | true |
| **indexed_throttle** | false | true | false | false | false | false | false | false |
| **engine_count** | 1 | 2 | 1 | 2 | 2 | 4 | 4 | 4 |
| **has_mixture** | true | true | true | true | true | true | true | true |
| **yaw_input_property** | fcs/rudder-cmd-norm | fcs/rudder-cmd-norm | fcs/rudder-cmd-norm | fcs/rudder-cmd-norm | fcs/rudder-cmd-norm | fcs/rudder-cmd-norm | fcs/rudder-cmd-norm | fcs/rudder-cmd-norm |

### 4.3 控制接口 Profile 分布画像

```
kOwnAutopilot (2):       c172x, c310           ← 有原生 AP XML，创建 ap/heading_hold
kFbwRateCommand (2):     f16, f22              ← 有 FBW 系统（f16 PID, f22 LQR+integrator）
kGenericAutopilotBridge (4): f15, Concorde, B17, C130  ← 只有项目 Autopilot.xml，无原生 AP/FBW
kDirectSurface (0):      (当前 8 机型无)        ← 仅当没有 Autopilot.xml 且没有 FBW/native AP
```

注意：由于项目对几乎所有机型注入了 `systems/Autopilot.xml`，`kDirectSurface` 在当前测试集中不存在。`kGenericAutopilotBridge` 成为**没有 FBW 也没有原生 AP 的机型的默认 fallback**。

### 4.4 后续注意事项

1. `has_mixture` 不是有效区分器——JSBSim 对所有机型都创建该属性，应与实际 mixture 控制逻辑分离。
2. `indexed_throttle` 仍由 `UsesIndexedThrottleInput()` 白名单控制，仅 `f22` 启用——后续应进入合同表而非散落函数。
3. `c310` 是唯一不同时包含项目 `Autopilot.xml` 和原生 AP 的机型——它的 `has_generic_autopilot = false` 但不走 `kDirectSurface`，因为 `ap/heading_hold` 来自独立的 `c310ap.xml`。
4. 新增机型后续纳入时，应优先运行 `ProfileSnapshotTest` 获取 profile 快照，再决定适配策略。

## 5. 阶段 11 更新：XML guidance profile 覆盖合同

> 日期：2026-06-04
> 来源：`Autopilot::ApplyEnergyDefaults()`、`ApplyXmlProfileOverrides()`、`JsbsimAdapter::ConfigureIntegrators()` 与 `AutopilotPreservesC172xXmlRollGuidanceOverrides` 回归测试。

### 5.1 配置原则

`AircraftControlProfile` 的配置来源按以下顺序生效：

1. C++ 结构体默认值：保持没有 aircraft XML tuning 的机型可运行。
2. 动态默认值：根据 FBW、发动机数、MOI、mixture 等属性树探测结果推导通用包线。
3. XML structural guidance：例如 `guidance/roll-angle-limit`，表示 aircraft XML 暴露的结构/系统约束；当前只映射到非 FBW、非重型、非 adapter fallback 的 sustained profile roll limit。
4. XML explicit profile override：例如 `guidance/max-roll-angle-deg`，表示直接覆盖 C++ profile 字段，优先级最高。

这意味着 XML 配置不是所有机型的必填项。没有专用 XML guidance 属性的机型继续使用动态默认值；只有已知模型限制、真实系统约束或测试证明需要机型特化时，才在 aircraft XML 中声明覆盖项。

### 5.2 当前 guidance 属性清单

| 属性 | 目标字段/用途 | 单位/语义 | 当前来源 |
|------|---------------|-----------|----------|
| `guidance/ref-speed-mps` | `ref_speed_mps` | m/s | optional XML override |
| `guidance/cruise-speed-mps` | `cruise_speed_mps` | m/s | optional XML override |
| `guidance/min-speed-mps` | `min_speed_mps` | m/s | optional XML override |
| `guidance/max-speed-mps` | `max_speed_mps` | m/s | optional XML override |
| `guidance/max-pitch-command-deg` | `max_pitch_command_deg` | deg | optional XML override |
| `guidance/roll-angle-limit` | `max_roll_angle_deg` | rad structural limit；非 FBW、非重型、非 fallback 时 C++ 乘 `0.7` sustained-turn factor | `c172x`、`global5000`、`systems/Autopilot.xml`、adapter fallback |
| `guidance/roll-rate-limit` | JSBSim AP/guidance property | rad/s，adapter 保留 XML 值 | `c172x`、`global5000`、adapter fallback |
| `guidance/max-roll-angle-deg` | `max_roll_angle_deg` | deg，显式 profile 覆盖，优先于 structural roll limit | optional XML override |
| `guidance/min-throttle` | `min_throttle` | normalized throttle | optional XML override |
| `guidance/max-throttle` | `max_throttle` | normalized throttle | optional XML override |
| `guidance/speed-energy-priority` | `speed_energy_priority` | bool-like number | optional XML override |
| `guidance/rotation-ramp-sec` | `rotation_ramp_sec` | sec | optional XML override |
| `guidance/rotation-max-elevator` | `rotation_max_elevator` | normalized elevator | optional XML override |
| `guidance/rotation-climb-rate-mps` | `rotation_climb_rate_mps` | m/s | optional XML override |
| `guidance/landing-approach-speed-mps` | `landing_approach_speed_mps` | m/s | `B747` |
| `guidance/landing-high-descent-agl-m` | `landing_high_descent_agl_m` | m AGL | `B747` |
| `guidance/landing-staging-agl-m` | `landing_staging_agl_m` | m AGL | `B747` |
| `guidance/landing-pattern-agl-m` | `landing_pattern_agl_m` | m AGL | `B747` |
| `guidance/landing-high-descent-orbit` | `landing_high_descent_orbit` | bool-like number；默认 true | optional XML override |
| `guidance/landing-descent-throttle` | `landing_descent_throttle` | normalized throttle | `B747` |
| `guidance/landing-approach-flaps-norm` | `landing_approach_flaps_norm` | normalized flaps | `B747` |
| `guidance/landing-final-flaps-norm` | `landing_final_flaps_norm` | normalized flaps | `B747` |
| `guidance/landing-final-throttle-cap` | `landing_final_throttle_cap` | normalized throttle cap | `B747` |
| `guidance/landing-flare-initial-elevator` | `landing_flare_initial_elevator` | normalized elevator | `B747` |
| `guidance/landing-heavy-flare` | `landing_heavy_flare` | bool-like number；启用 transport bounce/float flare law | `B747` |
| `guidance/landing-touchdown-agl-m` | `landing_touchdown_agl_m` | m AGL | `B747` |

### 5.3 已覆盖的非 B747 XML 合同

`c172x` aircraft XML 已声明：

```xml
<property value="0.523"> guidance/roll-angle-limit </property>
<property value="0.174"> guidance/roll-rate-limit </property>
```

阶段 11 回归测试验证两层合同：

1. adapter 不覆盖 XML 已声明的 roll limit / roll rate limit。
2. `Autopilot` 在动态默认值之后应用 aircraft structural roll limit，因此 `c172x` 的最终 `max_roll_angle_deg` 来自 XML structural limit 乘 sustained-turn factor，而不是 GA 默认的 `30 deg`。
3. adapter fallback `0.785 rad` 不被解释为 profile override；FBW 和重型机型也不从 generic `systems/Autopilot.xml` 的 roll limit 自动改写 dynamic profile。若这些机型需要 XML 驱动的 profile bank limit，应使用更明确的 `guidance/max-roll-angle-deg`。
