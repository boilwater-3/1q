# Flight Dynamic AP 调试状态报告

> 日期: 2026-05-27  
> 基线提交: `2fed395` feat(flight_dynamic): upgrade JSBSim to 1.3.1

## 1. 当前测试结果

### 2026-05-27 release 复核

`CMakeUserPresets.json` 中 `llvm-ninja-release-local` 当前已经是 `ENABLE_TESTING=ON`，并已用该预设重新配置、构建和运行 flight_dynamic 相关用例：

```bash
cmake --preset llvm-ninja-release-local
cmake --build --preset llvm-ninja-release-local
build/llvm-ninja-release-local/bin/1q_unit_tests \
  --gtest_filter='FlightDynamicTest.*:FlightDynamicRobustnessTest.*:*AircraftManeuverTest*'
```

初始结果：189 个 flight_dynamic 相关用例中 175 个通过、14 个失败。失败列表与 debug 口径一致。因此“release 测试二进制未重编译”这个问题已经可以从当前工作区状态中排除，但历史报告中的假阳性结论仍然成立。

经过本轮 waypoint/orbit 证据驱动的测试参数分层，以及 c310 自有 AP 集成修复后，当前结果为 **189 个 flight_dynamic 相关用例中 186 个通过、3 个失败**。

### 2026-05-27 深入复核增量

本轮使用 `llvm-ninja-release-local` 重新构建后，单独运行参数化机动测试：

```bash
build/llvm-ninja-release-local/bin/1q_unit_tests \
  --gtest_filter='*AircraftManeuverTest*'
```

初始结果仍为 **168 个用例中 154 个通过、14 个失败**，失败集合没有变化。随后根据 waypoint sweep 和 orbit 轨迹结果修正 mission 参数，并修复 c310 自有 AP 集成后，当前参数化机动测试为 **168 个用例中 165 个通过、3 个失败**。

已查询并对照 JSBSim 资料和源码：

- DeepWiki `Executive (FGFDMExec)`：`FGFDMExec` 是 JSBSim 主执行器，每个仿真步按固定模型顺序运行；
- 本地 `third_party/jsbsim/src/FGFDMExec.h`：模型顺序为 `ePropagate -> eInput -> eInertial -> eAtmosphere -> eWinds -> eSystems -> eMassBalance -> eAuxiliary -> ePropulsion -> eAerodynamics -> eGroundReactions -> eExternalReactions -> eBuoyantForces -> eAircraft -> eAccelerations -> eOutput`；
- 本地 `third_party/jsbsim/src/FGFDMExec.cpp`：`Run()` 每步 `IncrTime()` 后依次 `LoadInputs(i)` 和 `Models[i]->Run(holding)`；
- 本地 `third_party/jsbsim/src/initialization/FGTrim.h`：`tLongitudinal` 只做 `wdot/alpha`、`udot/thrust`、`qdot/elevator` 三轴配平，且 trim 失败常见原因包括初始速度、高度、构型、重量、CG 等条件不合理；
- 本地 `third_party/jsbsim/src/initialization/FGTrim.cpp`：trim 失败时会恢复初始控制量、重新 `Initialize()` 并 `Run()` 一次，因此不能把失败前的 partial trim 当作可保留状态。

本轮已做两个代码侧处理：

- `FlightManager::Step()` 改为在 `adapter_->Run()` 前执行 `maneuver_exec_->Update()` 和 `ap_->Update()`，使 AP/FCS 属性在 JSBSim 本帧 `eSystems` 阶段前写入。复验结果：失败集合仍为 14 个，说明一帧滞后不是当前 14 个失败的主因；
- `JsbsimAdapter` 接入 `config.silent_mode`，在静默模式下调用 `FGFDMExec::SetDebugLevel(0)`。单个 f15 失败日志从完整机型加载输出缩减为 33 行，后续诊断可读性明显改善。
- `Autopilot` 自有 AP 路径不再叠加 C++ pitch channel，避免 c172x/c310 XML 内置 `ap/elevator_cmd` 与项目 `fcs/elevator-cmd-norm` 双俯仰控制相加；同时仅在自有 AP 路径下按 JSBSim engine 数同步写入 `fcs/throttle-cmd-norm[i]`，修复 c310 双发油门只写未索引属性导致的长航路点掉高问题。复验结果：c310 `FlyToWaypoint/4` 通过，完整核心回归从 185/189 提升到 186/189。

本轮新增一个默认跳过的诊断探针 `FdAircraftProbe.EmitsAircraftProfileCsv`。该探针不作为常规 pass/fail 判据，只在显式设置环境变量时输出机型剖面 CSV：

```bash
FD_RUN_AIRCRAFT_PROBE=1 \
FD_AIRCRAFT_PROBE_CSV=/tmp/1q_aircraft_probe.csv \
build/llvm-ninja-release-local/bin/1q_unit_tests \
  --gtest_filter='FdAircraftProbe.*'
```

当前 CSV 覆盖 21 个机型，每个机型输出三类场景：

- `project_air_trim_on`：本项目空中初态 + `do_trim=true` + 项目 JSBSim 集成；
- `project_air_trim_off`：同一空中初态 + `do_trim=false`；
- `jsbsim_reset00`：不注入本项目系统，直接使用机型自带 `reset00.xml` 短时自由飞行。

首轮探针结果：

| 指标 | 结果 |
|------|------|
| 机型数 | 21 |
| 每机型场景 | 3 |
| `project_air_trim_on` trim 成功 | 16 |
| `project_air_trim_on` trim 失败 | 5: f22, B17, C130, L410, c310 |
| `project_air_trim_on` 10s 高度漂移超过 20m | 5: f22, B17, C130, L410, c172x |
| `project_air_trim_on` 正/负副翼探针滚转同号 | 1: f22 |

关键样本：

- A4、F4N、F80C、f15、f16、737、B747、Concorde、MD11 在当前空中初态下 `DoTrim(0)` 成功，且 10s 自由飞行保持有限状态。这些机型的 `FlyToWaypoint` 失败不应再粗略归类为“JSBSim 模型不可用”；
- f15 的 trim 成功、舵面响应存在且正/负副翼滚转方向相反，但 3.5km `FlyToWaypoint` 失败。这更像是目标几何、航向闭环、滚转增益或完成阈值问题，而不是基本配平失败；
- f22 的 trim 失败，且当前探针下 `fcs/left-aileron-pos-rad` / `fcs/right-aileron-pos-rad` 对 `fcs/aileron-cmd-norm` 没有响应，正/负副翼命令后的滚转变化同号。这强烈指向 FBW/FCS 属性语义或 XML 接口映射需要单独适配；
- B17、C130、L410 的 trim 失败与此前观察一致，但 `reset00.xml` 裸机短跑可进入有限状态，说明当前证据更支持“空中初态/配平流程/XML注入组合不合适”，不能直接判定为 JSBSim 库本身错误；
- c310 的 trim 失败，但它走自有 AP 路径，且 `reset00.xml` 裸机短跑保持有限状态。后续应优先查 `c310ap.xml` 的激活属性、输出属性和本项目桥接属性是否一致；
- `reset00.xml` 场景多数是地面或低速初态，只能证明模型可加载并短时运行，不能直接证明空中任务场景可控。

探针运行还暴露了新的测试环境副作用：

- 部分 JSBSim XML 自带 output 配置会尝试写当前目录，例如 B17 的 `JSBoutB17.csv`；
- 部分模型仍会输出 engine XML deprecation warning 或 socket bind error；
- `SetDebugLevel(0)` 不能完全静默所有 JSBSim 输出，只能减少模型加载和调试噪声。

本轮还新增一个默认跳过的 waypoint sweep 探针 `FdAircraftProbe.EmitsWaypointSweepCsv`：

```bash
FD_RUN_WAYPOINT_PROBE=1 \
FD_WAYPOINT_PROBE_CSV=/tmp/1q_waypoint_probe.csv \
build/llvm-ninja-release-local/bin/1q_unit_tests \
  --gtest_filter='FdAircraftProbe.EmitsWaypointSweepCsv'
```

该探针对 11 个失败代表机型分别跑 5km、10km、20km `FlyToWaypoint`，输出最小距离、最终距离、完成状态、高度、航向误差、最大滚转角和最大副翼命令。首轮结果如下：

| 机型 | 5km | 10km | 20km | 初步判断 |
|------|-----|------|------|----------|
| A4 | 未完成，最小约 433m | 未完成，最小约 191m | 完成，约 107s | 短距目标/完成半径与转弯半径不匹配 |
| F4N | 未完成，最小约 208m | 完成，约 69s | 完成，约 127s | 5km 过紧，10km 起可完成 |
| F80C | 未完成，最小约 119m | 完成，约 79s | 完成，约 152s | 5km 过紧，10km 起可完成 |
| f15 | 未完成，最小约 507m | 未完成，最小约 250m | 完成，约 107s | 控制链可用，5km/10km 测试几何过紧 |
| f16 | 5/10/20km 均未完成，最终飞离到 26-32km | 均未完成 | 均未完成 | FBW/FCS 命令语义或航向闭环问题 |
| f22 | 5/10/20km 均坠毁，最小距离仍很大 | 均坠毁 | 均坠毁 | trim 失败叠加 FBW/FCS 接口问题 |
| 737 | 未完成，最小约 183m | 完成，约 73s | 完成，约 139s | 5km 过紧，10km 起可完成 |
| B747 | 未完成，最小约 236m | 完成，约 70s | 完成，约 135s | 5km 过紧，10km 起可完成 |
| Concorde | 5km 坠毁 | 10km 完成 | 20km 坠毁 | 既有短距/远距任务问题，也有高度/俯仰稳定性问题 |
| MD11 | 未完成，最小约 257m | 完成，约 69s | 完成，约 129s | 5km 过紧，10km 起可完成 |
| c310 | 均未完成，最终接近地面且 `aileron-cmd` 最大值为 0 | 均未完成 | 均未完成 | 已定位为自有 AP 双俯仰控制叠加 + 双发油门 indexed 属性未同步，代码侧已修复 |

该结果显著缩小了归因范围：

- A4、F4N、F80C、f15、737、B747、MD11 不是“无法飞向目标”，而是在 5km 目标和 100m 默认完成半径下经常掠过目标后重新拉大距离；
- f15 20km 可完成，且探针显示 trim、舵面响应、航向闭环都能工作，因此 f15 不应继续作为 JSBSim 基础模型或集成顺序错误的主要证据；
- f16 的 `FlyToWaypoint` 在 10km 目标和 3km 完成半径下可完成，说明 waypoint 失败主要是完成半径/转弯半径问题；但 f16 Orbit 放大到 6km 半径后仍不能保持，应继续查 FBW orbit 控制；
- c310 的 `max_abs_aileron_cmd=0` 符合自有 AP 路径特征；后续证实失败主因不在 C++ 副翼 PD，而在自有 AP 集成：C++ pitch channel 与 `c310ap.xml` 的 `ap/elevator_cmd` 叠加，同时双发模型需要同步 indexed throttle 命令。

本轮继续新增一个默认跳过的 orbit sweep 探针 `FdAircraftProbe.EmitsOrbitSweepCsv`：

```bash
FD_RUN_ORBIT_PROBE=1 \
FD_ORBIT_PROBE_CSV=/tmp/1q_orbit_probe.csv \
build/llvm-ninja-release-local/bin/1q_unit_tests \
  --gtest_filter='FdAircraftProbe.EmitsOrbitSweepCsv'
```

当前结果：

| 机型/初态 | 半径范围 | 结果 | 初步判断 |
|-----------|----------|------|----------|
| f16, 3000m / 200mps | 1/3/6/10/20km | 全部不坠毁，但最终距圆心 31-40km；20km 半径仍偏约 11.5km | 横向 orbit 引导/FBW 横向语义不收敛，不是单纯半径太小 |
| Concorde, 5000m / 150mps | 1/3/6/10/20km | 约 87-122s 坠毁；1-10km 半径下最终半径误差约 1.7-3.5km | 横向半径可接近，失败主因是长期高度/能量保持 |
| Concorde, 10000m / 250mps | 1/3/6/10/20km | 约 217-255s 坠毁；1-20km 半径下最终半径误差约 1.4-4.2km | 提高初始能量只能延缓坠毁，不能解决高度通道 |

两个负向实验也已完成并撤回：

- f16 启用 `fcs/fbw-override=1` 后，`FlyToWaypoint`、`SetHeading`、`SetAltitude`、`SetRoll` 仍通过，但 `Orbit/4` 半径误差从约 14.7km 变为约 15.7km，未改善；
- 将通用高度保持油门基线从 `0.70` 提到 `0.85` 没有改善 Concorde `Orbit/7`，且会增加跨机型回归风险。

当前新增判断：

- 3.5km 非 DUMP 目标对高速战斗机和大型运输机过近，已经混入测试场景设计问题。f15 在 DUMP 模式的 20km 目标上可通过同一测试，而 3.5km 目标失败，说明不能把该项直接归类为 AP 库或 JSBSim 库错误；
- Trim 失败结论应表述为“当前空中初态 + 当前集成流程 + JSBSim 1.3.1 下 `DoTrim(0)` 纵向配平失败”，不应扩大为“模型任何速度/高度组合或任何运行方式都失败”；
- 仍需通过裸机 reset 脚本、XML 注入差分、按机型目标距离分层测试继续拆分依赖库、集成和参数 XML 三类问题。

### 关键发现

**之前声称的 "168 测试全部通过" 是假象**。当时 `llvm-ninja-release-local` 预设的 `ENABLE_TESTING=OFF`（在 CMakeUserPresets.json 中），导致 release 测试二进制从未重新编译。实际运行的测试二进制是旧版本。

使用 `llvm-ninja-debug-local`（`ENABLE_TESTING=ON`）构建的测试二进制显示 **14 个失败**。

> 注意：当前工作区已将 `llvm-ninja-release-local` 的 `ENABLE_TESTING` 改为 `ON`。后续调试建议优先使用 release-local 复验，再用 debug-local 做单点诊断。

### 初始失败清单 (debug build, 非 DUMP 模式, 5km 目标)

| # | 测试 | 机型 | 失败原因 |
|---|------|------|----------|
| 1 | FlyToWaypoint | A4 | 距离缩减不足 (final > init*0.5) |
| 2 | FlyToWaypoint | F4N | 同上 |
| 3 | FlyToWaypoint | F80C | 同上 |
| 4 | FlyToWaypoint | f15 | 飞离目标 (final=5068m vs init*0.5=2500m) |
| 5 | FlyToWaypoint | f16 | 飞离目标 |
| 6 | FlyToWaypoint | f22 | 飞离目标 |
| 7 | Orbit | A4 | 未知 |
| 8 | Orbit | f16 | 未知 |
| 9 | FlyToWaypoint | 737 | 距离缩减不足 |
| 10 | FlyToWaypoint | B747 | 距离缩减不足 |
| 11 | FlyToWaypoint | Concorde | 坠毁 |
| 12 | FlyToWaypoint | MD11 | 飞离目标 |
| 13 | Orbit | Concorde | 未知 |
| 14 | FlyToWaypoint | c310 | 自有 AP 不收敛 |

### 当前剩余失败清单 (release-local, 分层目标/半径和 c310 集成修复后)

| # | 测试 | 机型 | 失败原因 |
|---|------|------|----------|
| 1 | FlyToWaypoint | f22 | trim 失败，最终坠毁，FBW/FCS 适配问题 |
| 2 | Orbit | f16 | 即使放大 orbit 半径仍无法保持，FBW/FCS orbit 控制待查 |
| 3 | Orbit | Concorde | 坠毁，高度/俯仰控制不稳定 |

### 非 DUMP 口径未失败的机型 (10/20)

B17, Boeing314, C130, DHC6, L410, OV10, c172p, c172r, c172x, c182

---

## 2. AP 三层架构

```
检测 ap/heading_hold 存在？
├─ 是 → use_own_ap_=true（c172x, c310）
│   └─ 使用机型自带 AP（c172ap.xml / c310ap.xml）
│       C++ 只设 setpoint，不控制舵面
│
└─ 否 → 检测 ap/autopilot-roll-on 存在？
    ├─ 是 → use_cpp_ap_=false（generic AP bridge）
    │   └─ GNCUtilities 计算航向误差 → C++ PD+I 控制
    │       读取 guidance/angle-to-heading-rad
    │       写入 fcs/aileron-cmd-norm
    │
    └─ 否 → use_cpp_ap_=true（纯 C++ PD）
        └─ 直接从 FGPropagate 计算航向
            速度自适应增益
```

---

## 3. 当前假设与证据边界

本节不能视为最终根因。当前证据仍混合了三类变量：

1. JSBSim 1.3.1 模型本身在给定初始条件下是否可配平、可控；
2. 本项目的集成顺序、属性桥接、AP/FCS 接口是否正确；
3. 修改过的机型 XML 参数、系统引用和测试目标是否合理。

因此下面的结论应按“待验证假设”处理，而不是直接归咎于依赖库或某个控制器。

### 3.1 Trim 失败机型 (f22, B17, C130, L410, c310)

- **现象**: `DoTrim(0)` 抛异常 `udot doesn't appear to be trimmable`
- **当前假设**: 这些机型模型在当前空中初始条件、当前集成顺序和 JSBSim 1.3.1 组合下无法纵向配平
  - 已验证：移除所有添加的系统（GNCUtilities/Autopilot/Navigation sensor）后仍然 trim 失败
  - 已验证：调整速度（80-200 m/s）和高度（500-3000m）均无效
- **重要边界**: 这些验证都是从空中状态直接配平，不等价于模型不能从 reset 脚本、地面起飞、爬升后稳定飞行，也不等价于依赖库本身错误
- **新增证据**: `FdAircraftProbe` 显示 B17、C130、L410、c310 的 `reset00.xml` 裸机短跑仍可进入有限状态；f22 缺少 `reset00.xml`
- **影响**: B17/C130/L410 在任务场景中可能以非平衡态起飞并持续掉高度；f22 还叠加了 FBW/FCS 属性语义问题；c310 还叠加自有 AP 桥接问题
- **当前状态**: B17/C130/L410 非 DUMP 测试（3.5km 目标）通过——飞机在坠毁前到达目标；c310 已通过自有 AP 集成修复移出失败集合；f22 非 DUMP 仍失败
- **DUMP 测试**（20km 目标）失败

需要补充的验证：

- 使用机型自带 `reset00.xml` 或 JSBSim 推荐初始条件跑同模型裸机稳定性；
- 在不执行 `DoTrim(0)` 的情况下执行短时自由飞行，区分“trim 失败”和“模型立即失稳”；
- 分别测试地面初态、空中初态、先 warm-up 再 trim、先 RunIC 再系统注入等流程；
- 记录 trim 失败时 `u/v/w/p/q/r`、迎角、油门、俯仰配平、发动机状态，而不是只记录异常文本。

### 3.2 战斗机群控失败 (A4, F4N, F80C, f15, f16, f22)

- **现象**: FlyToWaypoint 飞离目标或距离缩减不足
- **f15 详细调查**:
  - FCS 是简单的直接映射（和 B17/C130 一样），**没有 FBW**
  - Trim 成功（Pitch Trim: -0.052, Passed）
  - `FdAircraftProbe` 显示 10s 空中自由飞行高度漂移约 +0.37m，正/负副翼命令产生相反滚转响应
  - 移除 GNCUtilities + Autopilot.xml 让其走纯 C++ PD 路径后仍然失败（final=28741m）
  - `FdAircraftProbe.EmitsWaypointSweepCsv` 显示 f15 在 20km 目标可完成，5km/10km 会接近目标后掠过再远离
  - 当前更应把 f15 归为测试目标几何/完成半径问题，而不是基础控制链或 JSBSim 模型问题
- **f16/f22**:
  - 有 FBW/LQR 飞控系统
  - f16: `fcs/aileron-cmd-norm` 被当作滚转速率命令（经过 PID + feedforward）
  - f22: 完整 LQR tracker，同样把 aileron-cmd-norm 当速率命令
  - 新探针显示 f16 在当前空中初态可 trim 且正/负命令滚转方向相反；f22 trim 失败且当前副翼位置属性无响应，应拆成两个不同适配问题处理
  - Orbit sweep 显示 f16 即使放大到 20km orbit 半径仍不收敛，且 `fcs/fbw-override=1` 实验无效；下一步应优先查 orbit heading law 与 FBW 滚转率命令匹配，而不是继续放宽半径

### 3.3 大型运输机失败 (737, B747, Concorde, MD11)

- **737, B747, MD11**: 5km 距离缩减不足，但 10km/20km waypoint sweep 可完成
- **Concorde**: 俯仰/能量保持失效，Orbit sweep 中横向半径可接近但约 87-255s 坠毁
- **新增证据**: 737、B747、Concorde、MD11 在 `project_air_trim_on` 探针下均能 trim 成功，并在 10s 空中自由飞行中保持有限状态。该组更应优先从任务几何、速度/转弯半径、俯仰/高度闭环增益入手，而不是先怀疑 JSBSim 模型加载或基础配平。

### 3.4 c310 自有 AP 问题

- **现象**: 自有 AP（c310ap.xml）在长航路点任务中控制收敛差，修复前最终几乎贴地且无法完成
- **Trim 失败**: `udot doesn't appear to be trimmable`
- **已确认原因**:
  - c310ap 需要 `ap/attitude_hold`（不仅仅是 `ap/heading_hold`）来激活滚转 PID；
  - c310 XML 的 elevator summer 同时累加 `ap/elevator_cmd`、`fcs/elevator-cmd-norm` 和 trim，项目自有 AP 路径不应再叠加 C++ pitch channel；
  - c310 是双发模型，长任务中的油门命令需要同步写入 `fcs/throttle-cmd-norm[0]` / `[1]`，只写未索引属性不能覆盖该集成缺口。

当前代码侧修复后，c310 `FlyToWaypoint/4`、`SetAltitudeClimb/4` 与 c172x 对照用例均通过；完整核心回归剩余失败已不包含 c310。

---

## 4. 已验证无效的修复尝试

| 尝试 | 结果 |
|------|------|
| 调整初始速度/高度（80-200 m/s, 500-3000m） | B17/C130/L410 trim 仍然失败 |
| 保留 partial pitch trim（DoTrim 异常后） | JSBSim 在抛异常前已重置属性值为 0 |
| 迭代 pitch trim（10轮×50步 warm-up） | 破坏了其他 14 个测试（状态重置不完整） |
| 从 f15 移除 Autopilot.xml 走纯 C++ PD | 仍然失败（final=28741m） |

---

## 5. 待解决问题优先级

### P0: 拆分测试用例语义

waypoint sweep 已证明 A4、F4N、F80C、f15、737、B747、MD11 在更长目标距离下可完成，原 5km 任务把转弯半径、完成半径、任务距离和 AP 能否工作混在一个断言里。

**需要落地**:
1. 将 `FlyToWaypoint` 拆成 smoke / controllability / mission 三层；
2. mission 层按机型类别设置目标距离、完成半径和最大时长；
3. 保留 5km 近距用例，但改成“最小距离曾经接近目标”或专门命名为 near-pass probe，而不是要求所有机型完成。

### P1: FBW 机型适配 (f16, f22)

f16/f22 的 FCS 把 `fcs/aileron-cmd-norm` 解释为滚转速率命令而非副翼位置。C++ AP 写入的是副翼角度命令。

本地 XML 证据：

- `third_party/jsbsim/aircraft/f16/f16.xml`：`fcs/aileron-cmd-norm` 先参与 `fcs/roll-trim-error` 和 `fcs/roll-rate-command`，默认路径是滚转率闭环；`fcs/fbw-override == 1` 时才把 `fcs/aileron-cmd-norm` 作为直接输入送入 `fcs/roll-rate-command-switch`；
- `third_party/jsbsim/aircraft/f22/f22.xml`：`fcs/aileron-cmd-norm` 经 `fcs/roll-cmd-filter`、`fcs/roll-rate-cmd`、`fcs/roll-rate-error`、LQR tracker，再输出到 `fcs/aileron-act`；没有 f16 那种 `fcs/fbw-override` 旁路；
- 当前 generic AP 分支仍按“目标滚转角 -> 副翼位置命令”的思路写 `fcs/aileron-cmd-norm`，对 f16/f22 语义不匹配。

**方案**:
- f16：`fcs/fbw-override=1` 直接面控制实验已经验证无效；下一步应在默认 FBW 下按滚转率命令设计 orbit 横向控制；
- f22：不能靠 override，必须按其 LQR tracker 期望输出滚转率/归一化滚转命令；
- Autopilot 内部应区分 direct-surface、generic-AP、FBW-rate-command、自有 AP 四类横滚输出语义。

当前边界：

- f16 `FlyToWaypoint` 已通过 10km 目标 + 3km 完成半径验证，剩余问题集中在 Orbit；
- f16 Orbit sweep 在 1/3/6/10/20km 半径下全部横向不收敛，因此它不是单纯测试半径过紧；
- f22 `FlyToWaypoint` 仍失败且 trim 失败，不能按 f16 的完成半径问题处理。

### P2: Trim 失败机型 (B17, C130, L410)

当前是“空中初态 + 当前集成流程 + JSBSim 1.3.1”组合下的配平失败。非 DUMP 测试通过，DUMP 测试失败。

**方案**: 标记为已知限制，或在 FlightManager 层面增加 warm-up 阶段（AP 创建后）

### P3: c310 自有 AP 已修复，保留回归覆盖

waypoint sweep 曾显示 c310 在 5/10/20km 都不能完成，最终接近地面，且本项目侧 `fcs/aileron-cmd-norm` 最大值为 0。后续确认这是自有 AP 路径下的集成问题：c310 XML 自己输出 `ap/elevator_cmd`，项目不应叠加 C++ pitch channel；双发油门也需要同步 indexed throttle。当前 c310 `FlyToWaypoint/4` 已通过，后续重点是保留 c172x/c310 自有 AP 对照回归。

### P4: Concorde 俯仰震荡

Concorde 在 10km 可完成，但 5km 和 20km 会坠毁。pitch PD 控制器或高度保持在该模型上不稳定，可能需要机型分层增益或限制俯仰/油门策略。

Orbit sweep 补充显示：Concorde 的横向 orbit 半径在 1-10km 档可接近目标环，但最终都因高度降到地面失败；把初始条件提高到 10000m / 250mps 只能把坠毁时间延后到约 217-255s，不能解决高度保持。因此下一步应聚焦 pitch/energy 管理，而不是继续调整 orbit 横向半径。

---

## 6. 测试基础设施问题

### CMakeUserPresets.json

历史问题是 `llvm-ninja-release-local` 预设中 `ENABLE_TESTING=OFF`。该项需要保持为 `ON`，以确保测试二进制随代码更新。

当前工作区已修改为 `ON`，release-local 可正常生成测试目标，并已给 release-local test preset 补上 `outputOnFailure=true` 与 `jobs=4`。

### 调试建议

优先使用 `llvm-ninja-release-local` 复验整体失败清单；使用 `llvm-ninja-debug-local` 做单机型、单通道诊断。

### 已暴露的测试编写问题

当前测试本身会影响判断质量：

- `config_.silent_mode = true` 原本没有被 `JsbsimAdapter` 使用，JSBSim 加载模型的输出会淹没真正的失败信息；当前已接入 `FGFDMExec::SetDebugLevel(0)`；
- CTest 只注册 `unit::1q_unit_tests` 这种二进制级测试，不能直接用 `ctest -R AircraftManeuverTest` 过滤参数化用例；
- `FlyToWaypoint` 同时断言距离缩短、完成状态和未坠毁，导致“控制不收敛”“目标阈值过严”“高度控制失效”混在一起；
- DUMP 模式改变目标距离和运行时长，不只是“输出轨迹”，因此它不是纯诊断开关；
- 所有机型共享同一类目标几何和通过阈值，未按机型性能区分可达性、转弯半径、爬升能力和稳态速度；
- `RunUntilDone` 忽略 `Step()` 返回值，模型停止运行和任务未完成可能被混合；
- 部分 JSBSim XML 自带 output 文件（例如 `JSBoutB17.csv`）会尝试写当前目录，测试环境没有统一重定向或禁用输出。

当前已补三个默认跳过的诊断探针：

- `FdAircraftProbe.EmitsAircraftProfileCsv`：输出 trim、AP 路径、FCS 接口、自由飞行和副翼响应；
- `FdAircraftProbe.EmitsWaypointSweepCsv`：输出不同 waypoint 距离下的完成状态、最小距离、航向误差、滚转和副翼命令。
- `FdAircraftProbe.EmitsOrbitSweepCsv`：输出不同 orbit 半径/初态下的半径误差、高度下限、滚转和坠毁状态。

后续正式测试应从这些探针中提炼稳定、语义单一的断言，而不是直接依赖一组跨所有机型的 5km mission 断言。

---

## 7. 后续探索矩阵

目标是把问题归类到“依赖库本身问题 / 项目集成问题 / 参数 XML 问题 / 测试用例问题”，而不是一次性调 AP 增益。

### A. JSBSim 裸机基线

对失败机型先跑不注入本项目新增 Navigation/GNCUtilities/Autopilot 的裸机实验：

- 自带 reset 脚本初态；
- 当前测试空中初态；
- `do_trim=true` 与 `do_trim=false`；
- 固定油门、固定舵面中立、短时自由飞行。

判定标准：

- 裸机自带 reset 可稳定，但当前空中初态失败：优先怀疑测试初态或配平流程；
- 裸机当前空中初态不可控，但自带 reset 可控：不应直接归咎 JSBSim 库；
- 裸机在推荐初态也失稳：再考虑模型/库兼容性。

### B. 集成顺序与控制链验证

历史 `FlightManager::Step()` 顺序是先 `adapter_->Run()`，再 `ap_->Update()`，再 `maneuver_exec_->Update()`。这意味着 AP 命令至少晚一帧生效，Orbit 的 heading target 也在本帧动力学之后才更新。

当前已改为：

```text
maneuver_exec_->Update()
ap_->Update()
adapter_->Run()
Map vehicle state
IsManeuverComplete()
```

release-local 复验后失败集合未变化，因此顺序问题是应修复的集成瑕疵，但不是当前 14 个失败的主因。

### C. AP/FCS 接口类型识别

按机型记录 `fcs/aileron-cmd-norm` 的语义：

- 直接副翼位置命令：A4、737、B747、MD11、F80C、f15 等目前看是位置类输入；
- 滚转速率命令或 FBW 输入：f16、f22；
- 自有 AP 命令混合输入：c172x、c310。

如果同一 C++ AP 对位置类 FCS 仍错误，需要检查舵面符号、航向误差符号和滚转闭环；如果仅 FBW 机型失败，再做速率命令适配。

### D. XML 注入差分

对 19 个修改过的 XML 建立三路对照：

- 原始 XML；
- 只注入 Navigation + GNCUtilities；
- 注入 Navigation + GNCUtilities + Autopilot。

若只注入 Navigation/GNCUtilities 就改变 trim 或自由飞行稳定性，优先怀疑 XML 注入位置、属性覆盖或系统副作用。若只有 Autopilot 注入后变化，再查 `Autopilot.xml` 参数和属性连接。

### E. 测试目标合理性

把 `FlyToWaypoint` 拆成至少三层：

- smoke：能运行、无 NaN、质量和时间有效；
- controllability：航向误差、滚转响应和舵面响应方向正确；
- mission：按机型性能给出目标距离、完成阈值、最大时长和最低高度。

只有 mission 层失败时，才讨论目标距离或增益；controllability 层失败时，优先查接口和符号。

---

## 8. 已修改的机型 XML 文件 (19 个)

以下机型在 XML 末尾添加了 Navigation sensor + GNCUtilities + Autopilot 系统引用：

```
A4, F4N, F80C, f15, f16, f22, OV10,
737, B17, B747, Boeing314, C130, Concorde, DHC6, L410, MD11,
c172p, c172r, c182
```

未修改: c172x（已有完整 AP）、c310（有自有 AP）

---

## 9. DUMP 测试结果 (20km 目标, 供参考)

非 DUMP 通过但 DUMP 失败的机型（隐蔽缺陷）:

| 机型 | 距离缩减 | 最终高度 | DUMP 状态 |
|------|----------|---------|-----------|
| B17 | 45.2% | -0.8m | 坠毁 |
| C130 | 99.3% | -1738m | 到达但坠毁 |
| L410 | 91.5% | -1936m | 到达但坠毁 |
| c310 | 15.1% | 0.3m | 几乎未动 |
| Concorde | 44.2% | -0.9m | 坠毁 |
| f16 | -33% | 2935m | 飞离 |
| f22 | 0.7% | -3677m | 飞离+坠毁 |
