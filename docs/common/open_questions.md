# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

---

## OQ-1 飞行动力学局部 NE 投影 cos-lat 约定分叉

`WaypointManager` 与 `Maneuver` 各自实现了 lat/lon → 局部北/东米投影，但两者使用的 cos-纬度约定不同，对大 cross-track 偏移会产生不同几何结果。

- `WaypointManager` 用平均纬度 cos：`src/flight_dynamic/guidance/WaypointManager.cpp:24`（`mean_latitude_rad = 0.5 * (lat + origin_lat)`）、`:28`（`east_m = ... * cos(mean_latitude_rad)`）。
- `Maneuver` 用参考中心纬度 cos：`src/flight_dynamic/guidance/Maneuver.cpp:160`、`:203`（`cos_lat = std::cos(center.latitude_rad)`），并在文件内 6 处内联重复该投影。

为何未决：无法从代码判断哪个约定是有意为之。两者在小偏移下数值接近，分叉只在远场才显现；现有测试未覆盖"两套约定应一致"或"应不同"的断言。合并到单一投影需要先决定以哪个 cos-lat 约定为准。

推进需要：
- 领域知识确认：orbit / figure-8 / racetrack 几何中，cos(mean lat) 与 cos(center/reference lat) 哪个是正确意图；
- 决定后，要么统一为单一约定（带参数的 helper），要么显式记录"两者有意不同"并保留；
- 重测 orbit / figure-8 / racetrack 几何，确认无回归。

注：P2.5a（commit `ff0c9a2c`）只收拢了 `NormalizeRad`/`RadToDeg360`（角度归一化），明确未触碰此 NE 投影分叉。

## OQ-2 EOS replay 派生环境字段非对称 codec

`EosSessionConfig` 的 replay codec 对派生环境字段是非对称的：encode 写入、decode 丢弃。schema 注释自称这些字段"冗余记录以支持精确比对"，但 decode 端从不读取它们。

- encode 写入派生字段：`src/electro_optical_sensor/session/EosReplayFlatbufferCodec.cpp:338`（`BuildModelConfigFromScenario` 后写入 `radiative_transfer_model_derived` 等）。
- decode 不读派生字段：`DecodeEosSessionConfig` 只还原 `scenario_config`，从不读 `*_derived`（依赖 consumer 重新 `BuildModelConfigFromScenario`）。
- schema 注释自述冗余：`schemas/replay/eos_session_replay.fbs` 的 `EosEnvironmentConfig`。

为何未决：当前行为正确（consumer 会重新 derive），不是 bug。但这是 source-of-truth / drift 隐患——若 `BuildModelConfigFromScenario` 的 preset→factor 映射改变，replay buffer 的"冗余比对"字段会与 fresh decode 静默分叉，且现有 roundtrip 测试无法捕获（decode 不读这些字段）。

推进需要：先补一个断言"派生字段 encode/decode 一致"的 roundtrip 测试，再决定是让 decode 读回派生字段、还是从 schema 删除它们。

注：P2.2（commit `6ad98476`）只收拢了 detection-record 的 encode/decode，明确未触碰此派生字段非对称。

## OQ-3 飞行动力学失速速度 ρ 来源漂移

失速速度公式 `V_stall = sqrt(2W / (ρ·S·CLmax))` 的 ρ 在三个调用点来源不一致，是已知 bug 但尚无失败测试证据。

- Autopilot：硬编码 `kRhoSeaLevel = 0.002377`：`src/flight_dynamic/autopilot/Autopilot.cpp:339`。
- EngineManager `GetRotationSpeedKts`：读 property tree `atmosphere/rho-slugs_ft3`：`src/flight_dynamic/propulsion/EngineManager.cpp:184`。
- EngineManager `GetDefaultApproachSpeedMps`：读了 property tree ρ 做校验（`:290`），但 V_stall 计算又用硬编码 `kRhoSeaLevel`（`:311`）——同一函数内 ρ 来源自相矛盾。

为何未决：narrow 重构契约要求零行为变化，且无测试因 ρ 漂移而失败。修它等于改变至少一个调用点的数值输出，必须有测试兜底。

推进需要：
- 确认正确的 ρ 来源应是哪个（property tree 的实时大气密度，还是固定海平面常数）；
- 补一个针对 ρ 来源的失败/边界测试（如高海拔场景下 V_stall 应随 ρ 变化）；
- 在测试兜底下统一 ρ 来源。

注：P2.4（commit `65cc7fc4`）把 CLmax + V_stall 公式收拢为单一 `AircraftPerformanceDerivation` helper，但 ρ 作为入参透传，严格保留了三处现状——漂移本身未修。

## OQ-4 Session Impl 同时持有 owned_ptr 与同对象裸引用

AR 与 ESR 的 `Session::Impl` 同时存储 `std::unique_ptr<X> owned_x` 与 `X& x` 两份对同一对象的引用。当前安全的前提是 `Impl` 在堆上构造且不可移动赋值（移动构造已被 `= default` 但实际不发生 self-move）；一旦后续重构让 `Impl` 可移动赋值或转移所有权，裸引用会立即变成悬垂引用（use-after-free）。

- AR：`src/airborne_radar/session/ArSession.cpp:228-234` 同时持有 `owned_ar_context` + `radar_context`、`owned_signal_pipeline` + `signal_pipeline`、`owned_environment_service` + `environment_service`、`owned_controller` + `controller`。
- ESR：`src/electronic_surveillance_radar/session/EsrSession.cpp:66-70` 同样形态（`owned_pipeline`+`pipeline`、`owned_environment_service`+`environment_service`、`owned_controller`+`controller`）。
- EOS 形态更简单：`src/electro_optical_sensor/session/EosSession.cpp:43-44` 只持有 `owned_pipeline`/`owned_controller`，controller 引用经 `RequireCompositionDependency` 现场派生，不作为成员冗余存储。

为何未决：当前不构成 bug（`Impl` 不发生会导致悬垂的操作）。消除它需要统一改为"成员访问处 `owned_x.get()` / `*owned_x`"，或引入类似 EOS 的 access helper，属于跨模块的本地所有权重构，不属于语义修复，须单独批次并配套 contract 说明可移动性边界。

推进需要：以 EOS 为参照重构 AR/ESR 的 `Impl` 持有方式，并补充一条 contract 规则（`Impl` 的可移动性 / 所有权唯一性约束），或在代码中显式 `delete` 移动赋值以封堵风险。

注：源自 `src/` 架构与安全审查的 M2，原状态 verified-deferred。

## OQ-5 flight_dynamic 模块游离于统一 cycle/result 范式

`flight_dynamic::FlightManager` 暴露 `GetAdapter()`/`GetAutopilot()`/`GetEngineManager()` 等内部子系统直接引用（`include/1q/flight_dynamic/FlightManager.h:168-172`），并使用 `FlightManagerState` 枚举状态机，而其它四个传感器模块采用 Session→Controller→Pipeline 分层与 cycle/result/abort-reason 协议。

- 公开子系统所有权：`include/1q/flight_dynamic/FlightManager.h:168-172`。
- 枚举状态机：同文件 `FlightManagerState`（`:48`）与 `state_`（`:187`），区别于其它模块的 `*CycleResult` + `*PipelineAbortReason`。

为何未决：这是 `flight_dynamic` 的既有 public 边界设计，不是缺陷。是否引入统一的 session/cycle 外壳属于跨模块 API 形态决策，需要先确认飞行动力学作为"平台状态生产者"的特殊定位是否应当保留更宽的 public 接口。

推进需要：先在 `docs/flight_dynamic/design.md` 显式文档化该模块的特殊边界（与传感器模块的区别），再评估是否引入更统一的 cycle 外壳。

注：源自 `src/` 架构与安全审查的 M8，原状态 verified-deferred。

## OQ-6 数值/校验 helper 残余散落与跨模块语义分叉

工具函数下沉已大部分完成（时间戳、validation helper、`NormalizeAngle180` 改 `fmod`、`SafePositive` 合并、`oneq::internal::*` 兼容 alias 已全部删除），但仍有两类残余。

- 数值下限常量散落：`kNormFloor` / `kNumericFloor` 在多处定义且阈值不一致（约 20 处分布在 `src/common/` 与各传感器模块）。收敛前需先按"范数下界 vs 数值下界"区分语义层。
- `target_id=0` 严重级别在 AR/EOS/ESR 间不一致：因各模块 ID 语义不同（AR track id / EOS 目标 id / ESR emitter id），统一前需先形成公共 target-id contract。

为何未决：两者都不是数值正确性 bug，而是语义分层与跨模块 contract 决策。机械合并会引入隐式阈值漂移或 ID 语义混淆，必须有测试冻结共享语义后再收敛。

推进需要：先把 `kNormFloor`/`kNumericFloor` 按语义归类并补单测，再下沉到 `src/common/numerics/`；target-id 规则先在 `contract.md` 形成公共约定，再统一严重级别。

注：源自 `src/` 架构与安全审查的 M3 残余项；M3 主体（时间戳/validation helper/fmod/SafePositive）与 M4（命名空间统一、alias 删除）已完成。

## OQ-7 自研 JSON 解析器的 long-term 替换决策

`src/common/config/JsonReader.cpp` 的主要加固已完成（最大嵌套深度、尾随内容拒绝、`\uXXXX` 完整性校验、surrogate escape 拒绝，见 `:79`/`:85`/`:189-193`/`:241`）。剩余的是是否长期替换为成熟 JSON 库的策略决策。

- 缺失键返回静态全局 `kNullValue` 引用（`src/common/config/JsonReader.cpp:11`/`:253-262`），改为 `optional`/指针会改变 `JsonValue` public API。
- 完整 surrogate pair 合成、更严格的数字语法校验仍可补充，但当前加固已阻断主要解析风险。

为何未决：替换/重写 `JsonValue` public API 是独立的 public-surface 变更，须单独契约化，不应与安全加固混批。当前自研解析器的剩余缺口不影响已加固的路径。

推进需要：评估引入成熟 JSON 库 vs 继续 harden 自研解析器，若替换则需同步 `JsonValue` consumer 与 public API 契约。

注：源自 `src/` 架构与安全审查的 H5 残余与 L1。

## OQ-8 common 层局部代码质量收尾

若干低风险样式/编译成本项已验证但未在本轮修复，列出以便独立批次处理，避免与语义修复混批。

- L3：`src/common/geometry/GeometryTransform.h:9` 全量 `#include <Eigen/Core>`，可评估前向声明降低编译成本（纯编译优化，无语义影响）。
- L4：约 10 处 `ptr.reset(new T)` 可逐步替换为 `std::make_unique`（纯样式）。
- L6：`src/common/atmosphere/AtmospherePhysics.cpp:63-76` 的 `refractivity_index_n_*` 同函数并列 `tc_celsius` 与 `tk_kelvin` 两个温标参数，调用方易传错；改签名涉及 REOS 对齐与兼容迁移。

为何未决：三者均为样式/兼容性收尾，不改变运行时行为，混入语义修复批次会模糊变更意图。

推进需要：在独立的小步重构批次中处理，L6 需要配套 REOS 对齐签名迁移。

注：源自 `src/` 架构与安全审查的 L3/L4/L6，原状态 verified-deferred。
