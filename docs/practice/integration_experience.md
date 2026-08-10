# 示例集成体验分析（边界划分与工具缺位）

Status: active
Last-reviewed: 2026-08-05
Authority: examples 集成实践（component_entt EnTT 模式 + component_attachment 自定义实体-组件模式，两次三传感器 + 融合 + 飞行动力学全链集成）

本文件是对"集成者视角"的库体验复盘：在两次全链示例集成中，哪些样板本应在库内
（边界划分不清）、哪些是库内能力存在但外部不可见（文档缺失）、哪些是有意的外部
职责（维持现状）。**P0/P1 行动项已于 2026-08-05 实施完毕**（见各条目标记与
行动建议表），P2 维持待办；实施后各条目已同步对应模块设计文档与头文件注释。

## 判据

- **重复度**：≥2 个示例（component_entt / component_attachment / batch_validation）
  重写同一段逻辑 = 库面缺位的强信号；仅一处出现 = 示例特定，倾向维持。
- **性质分类**：
  1. 边界划错：库内该有而未提供，集成者被迫外部重写；
  2. 文档缺失：库内已有能力，但语义/用法外部不可见，集成者靠读源码或试错；
  3. 有意职责：场景编排/调参/输出格式属业务层（冻结契约），外部实现正确。

## 发现清单（按优先级）

### F1 传感器生命周期事件判定被集成者重造（边界划错）✅ 已实施 2026-08-05

- **库内已有**：`ArTrackLifecycleRecorder`（`include/1q/airborne_radar/session/`，
  kFirstConfirmed / kUpdated / kLost / kNotTracked + reason + `GetLastEvents()`，
  经 `ArSession::AttachTrackLifecycleRecorder` 接线）与
  `EosDetectionLifecycleRecorder`（kFirstDetected / kUpdated / kLost，语义对称）。
  两处均内置"状态迁移差分"，即事件流设施。
- **实际**：component_entt 与 component_attachment **两代示例均未使用**；
  component_attachment 在 ArSensorComponent 自研 `confirmed_keys_` / `lost_keys_`
  集合判定，审查发现**静默掉轨漏报 bug**（轨迹直接缺帧无 kLost 时键滞留，
  重捕不报 confirm；集合无界增长）——正是库内 recorder 已解决的状态迁移问题。
- **根因**：recorder 需显式 `Attach` 接线、头文件淹没在 session 目录、集成者从
  `StepWithResult` 返回值出发自然走"自己维护状态"的路；库侧没有指向 recorder 的
  指引注释。
- **建议（已实施）**：
  1. ✅ 示例改用 LifecycleRecorder（commit 4d66b1b4）：ArSensorComponent /
     EosSensorComponent 均 Attach 对应 recorder，事件转发自 `GetLastEvents()`
     （AR kFirstConfirmed/kLost → 示例事件，位置从本周期帧回查；EOS
     kFirstDetected/kUpdated/kLost → EosDetectionEvent.kind，方位回查）；
     自研集合与掉轨漏报 bug 消除，双配置（FD ON/OFF）回归通过；
  2. ✅ Session 头文件注释补充（同 commit）：`ArSession.h` / `EosSession.h`
     Attach 注释注明"轨迹/探测事件通知建议通过本机制，recorder 内建跨周期
     状态迁移差分"。

### F2 平台运动学 → ECEF 与方位/距离投影缺位（工具缺位，2 处重复）✅ 已实施 2026-08-05

- `ResolvePlatformEcef`（LLA/航向/速度 → ECEF 位置+速度）：component_entt
  `systems.cpp` 与 component_attachment `sensor_utils.h` 各写一遍，逻辑逐字相同
  ——`TryLlaToEcef` + `TryEnuToEcefVelocity` 两段式，"航向（北偏东）→ ENU 分量"
  的业务约定藏在两处示例代码里。
- `MakeEnuBasis` / `MakeTargetStates`（方位/距离 → ECEF 偏移）：两示例目标脚本
  各写一遍同一套 ENU 基投影。
- **建议（已实施）**：`coordinate` 域（纯函数共享域）补两个单函数：
  `TryMakeEcefVelocityFromHeading(heading_deg, speed_mps, origin_lla, *out)` 与
  `TryBearingRangeToEnuOffset(azimuth_deg, range_m, *out)`（ENU 偏移，与既有
  `TryEnuToEcef` 组合成目标位置）。两示例改用（commit cb33d91c + 2464ff1e），
  行为逐位等价（审查确认，含 z 语义）；目标**脚本编排**仍是业务层（冻结契约
  §7），仅上移几何单函数。

### F3 FlightManager 集成契约文档缺失（文档缺失，试错成本最高）✅ 已实施 2026-08-05

- **kTakeoff 目标航向**：`ManeuverCommand.target.latitude_rad` 对 kTakeoff 的
  语义是**目标航向**（默认 0 = 正北）。示例未设导致起飞后转向西北、破坏场景几何
  （审查 H1）；该语义只能读 `FlightManager.cpp` 源码确认，头文件注释未说明。
  （注：该注释已在集成期补充于 FlightManager.h 的 ManeuverCommand 说明块，
  本次复核确认在位。）
- **步长上限**：0.1s 子步进在滑跑/起落架段发散（roll 达 172° 后数值崩溃），
  10ms 稳定；`examples/flight_dynamic/takeoff_land_csv.cpp` 用 0.01 但未说明
  "为什么"。集成者只能试错。
- **状态机语义**：`PushManeuver` 会推进 kReady → kExecuting；按"构造后查 kReady"
  的模式在 Push 之后检查会误判初始化失败（示例一度误报 aircraft 数据缺失）。
- **建议（已实施，commit 6e2bcc86）**：`FlightManager.h` 的 `Step(dt)` 补步长
  上限 @note（建议 ≤10-20 ms，100 ms 起飞段发散，权威示例 10 ms）、`PushManeuver`
  补状态迁移说明（仅 kReady 派发并转 kExecuting，kExecuting 排队）；`Maneuver.h`
  补 kTakeoff 的 latitude_rad 语义交叉引用与 TakeoffPhase 相位注释；
  `takeoff_land_csv.cpp` kDt=0.01 补理由；`docs/flight_dynamic/algorithms.md`
  起飞条目补步长上限实现边界。纯注释级修复，不改 API。
- **后续（2026-08-10）**：步长证据锚点从示例迁至库内单测
  `tests/unit/flight_dynamic/fd_takeoff_substep_test.cpp`（10 ms 起飞稳定完成；
  100 ms 起飞段发散在 20 个固定翼机型实测，11 个发散、roll 155°-180° 后坠毁，
  与上述 roll 172° 记录吻合）；algorithms.md 证据与 `FlightManager.h` @note
  改指向该回归测试，示例（takeoff_land_csv / component_attachment）只保留
  "消费同一契约"的引用方向。

### F4 EOS 默认配置与常见周期不兼容（隐性覆写样板）

- EOS 默认 `frame_rate_hz=30` → 周期校验要求 dt ≤ 10/frame ≈ 0.33 s，1 s 周期
  必须覆写为 10 Hz；默认 `boresight_depression_deg=45°`（下视）与空中场景不匹配。
- 两示例抄同一段覆写（frame_rate / 扫描范围 / boresight），校验 issue 虽有但
  集成者要自己翻输出。
- **建议**：config 头文件注释或校验文案提示"常见 1 s 周期需 frame_rate ≤ 10 Hz"；
  扫描坐标系语义问题已有 open_questions.md 既有条目（天线坐标系），不重复登记。
- 状态：P2 待办。

### F5 Session 周期输入显式填充样板（有意设计，维持）

- `ArCycleInput` / `EsrCycleInput` / `EosCycleInput` 的 cycle/time/platform 字段
  逐项显式填充（batch_validation 亦同）。这是**校验友好的有意设计**，不建议加
  builder 掩盖。
- 附带陷阱：`ArCycleResult.track_output_frame` 是**内部雷达局部帧**
  （TrackStateSnapshot 无 ECEF），消费必须经 `ArCycleOutputAdapter::Build` 转外部
  帧；示例集成者两次误用（编译期才发现）。建议 `ArCycleResult.h` 对
  `track_output_frame` 字段注释"内部帧，消费请用 ArCycleOutputAdapter"。
- 状态：P2 待办（头文件注释）。

## 有意的外部职责（不上移）

- **场景脚本**（目标真值构造、目标推进）：冻结契约 §7 明确场景/编队编排属业务层，
  两示例参数不同，上移会固化业务语义；
- **融合/扫描配置调参**（FusionConfig 覆写、EOS 扫描角）：业务调参（注释已声明
  "非库内标准"），默认值已存在于 config；
- **CSV/JSON/config_loaders**：已收敛到 `examples/common/`（示例层共享机制），
  CSV 非库职责（库内 output 域是序列化，不是报表）；
- **事件总线形态**：冻结契约 §5 不建全局事件总线；示例选型（EnTT observer /
  Boost.Signals2）属业务层选择，库不提供。

## 行动建议

| 优先级 | 行动 | 涉及面 | 性质 | 状态 |
|---|---|---|---|---|
| P0 | 示例改用 LifecycleRecorder，消除自研事件判定（F1） | examples/component_attachment + Session 头注释 | 边界修正 | ✅ 已实施（4d66b1b4） |
| P0 | FlightManager.h / Maneuver.h 补 kTakeoff 航向、步长、状态机注释（F3） | include/1q/flight_dynamic | 文档 | ✅ 已实施（6e2bcc86） |
| P1 | coordinate 补平台运动学 → ECEF 与方位/距离投影单函数，两示例改用（F2） | include/1q/coordinate + 2 示例 | 工具缺位 | ✅ 已实施（cb33d91c + 2464ff1e） |
| P2 | ArCycleResult 内部帧字段注释（F5 附带）；EOS 校验提示（F4） | 头文件注释 | 文档 | 待办 |
| 维持 | Session 输入样板 / 场景编排 / 配置调参 / 事件总线选型 | — | 有意职责 | 维持 |

若行动项涉及跨模块 API 变更或契约争议，按 `docs/common/open_questions.md` 流程
登记后再实施；P0 两项为示例侧改动与注释补充，可直接实施（已按此执行）。

## 变更规则

- 实施任一行动项后：更新本文件对应条目（标记已实施/日期/涉及 commit），并同步
  相关模块设计文档（boundaries.md 若涉及边界声明）。——P0/P1 已按此执行
  （F3 同步 docs/flight_dynamic/algorithms.md；F2 无 coordinate 文档集；
  F1 同步两个 Session 头注释）。
- 后续全链集成示例（如扩展 SBIRS/SAR 通道）产生的同类发现，按上述判据追加到
  发现清单。
