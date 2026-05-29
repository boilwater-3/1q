# JSBSim 代表机型控制合同初稿

> 日期：2026-05-29
> 范围：阶段 2 首批 XML 审查，覆盖 `f16`、`f22`、`c310`、`c172x`、`Concorde`、`f15`。

## 1. 结论摘要

当前 6 个代表机型不能用同一个“写 `fcs/*-cmd-norm` 就等价于舵面位置”的假设处理。

| 机型 | 横向接口 | 纵向接口 | 油门接口 | 注入系统 | 输出副作用 |
|------|----------|----------|----------|----------|------------|
| `f16` | FBW roll-rate command | FBW pitch/g-load scheduler | 单发 `fcs/throttle-cmd-norm` | `Navigation`、`GNCUtilities`、`Autopilot` | datalog output 在注释块内 |
| `f22` | FBW roll-rate integrator + actuator | FBW pitch-rate/g-load integrator | 双发 indexed throttle，另有 thrust normalize | `Navigation`、`GNCUtilities`、`Autopilot` | datalog output 在注释块内 |
| `c310` | native AP + direct surface 混合 | native AP + direct surface 混合 | 双发 indexed throttle/mixture | `GNCUtilities` + `c310ap` | active CSV/socket |
| `c172x` | generic AP/native `c172ap` + direct surface 混合 | generic AP/native `c172ap` + direct surface 混合 | 单发 throttle + mixture system | `Navigation`、`GNCUtilities`、`Autopilot`、`c172ap` | active CSV/socket |
| `Concorde` | direct surface | direct surface | 四发 indexed throttle | `Navigation`、`GNCUtilities`、`Autopilot` | 未见 active output |
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
| Throttle/Mixture | indexed mixture `fcs/mixture-cmd-norm[0..1]`; throttle 由 engine count 与 C++ indexed write 驱动 | 双发 indexed 控制 |

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
| Yaw | `fcs/rudder-pedal-norm` + trim + yaw damper -> rudder actuator | yaw uses pedal property; `fcs/rudder-cmd-norm` first feeds nose wheel steering |
| Throttle | four Olympus engines | 四发 indexed 控制需求 |

关键风险：当前 Concorde `Orbit` 失败表现为高度/能量问题，横向 direct surface 不是唯一主因；yaw path 也不是单纯 `fcs/rudder-cmd-norm` 到 rudder。

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
4. Concorde 的 yaw 输入应标注为 `fcs/rudder-pedal-norm`，避免 C++ 只写 `fcs/rudder-cmd-norm` 后误判 yaw 有效。
5. XML output 需要在 adapter/test fixture 层统一禁用或重定向；c172x/c310 是当前 active output 的重点。
