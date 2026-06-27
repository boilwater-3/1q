# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

---

## OQ-1 四模块 runtime 配置提交策略不一致

各传感器模块对"runtime 配置修改如何安全生效"采用了四种不同策略，没有统一范式。这不是 bug——每个策略单独看都自洽——但是跨模块阅读时的主要认知负担来源，且新模块/新维护者缺少明确的基准。

| 模块 | 配置提交时机 | 提交失败处理 | 周期执行失败处理 |
| --- | --- | --- | --- |
| airborne_radar | 延迟到下个周期边界（`StepWithResult` 开头） | 4 子系统快照回滚 | 4 子系统快照回滚 |
| electronic_surveillance_radar | 立即（调用即生效） | 无回滚 | pipeline + controller 快照回滚 |
| electro_optical_sensor | 立即（调用即生效） | 无回滚 | 无回滚 |
| sar | 立即 patch 到 `runtime_config` | 无回滚 | 执行前 gate（`ValidateRuntimeConfigForStep`） |

证据：
- AR 延迟提交 + 回滚：`src/airborne_radar/session/RadarSession.cpp:117`（`CommitPendingRuntimeConfig`，在 `controller.RunOnce` 之前）、`:185-197`（快照 capture + 失败 restore）。
- ESR 立即提交 + 执行失败回滚：`src/electronic_surveillance_radar/session/EsrSession.cpp:94`（`TryApplyRuntimeConfig` 立即调 `UpdateConfig`）、`:48-58`（`RunCycle` 内 capture/restore）。
- EOS 立即提交无回滚：`src/electro_optical_sensor/session/EosSession.cpp:86`（`TryApplyRuntimeConfig` 立即 `ApplyInternalConfig`，无 capture/restore）。
- SAR 立即 patch + 执行前 gate：`src/sar/session/SarSession.cpp:191`（`TryApplyRuntimeConfig` 直接 patch）、`:127`（`ValidateRuntimeConfigForStep`）。

为何未决：四种策略各有合理性（AR 谨慎因信号 pipeline 状态多；EOS/SAR 轻量因 pipeline 相对简单），没有失败/竞态证据指向某一策略错误。统一需要先论证"哪种策略该作为基准"，属于架构级决策。

推进需要：
- 一个真实的失败案例（如某模块立即提交导致周期内配置中途变化、产生不可预料结果），或
- 明确的架构方向选择（如统一为 AR 式延迟提交 + 回滚，或统一为 EOS 式立即提交）。

注：P2.3（commit `a83928b3`）只收拢了 ESR 写路径绕 extension 的往返，未触碰任何模块的提交时机/安全策略。

## OQ-2 飞行动力学局部 NE 投影 cos-lat 约定分叉

`WaypointManager` 与 `Maneuver` 各自实现了 lat/lon → 局部北/东米投影，但两者使用的 cos-纬度约定不同，对大 cross-track 偏移会产生不同几何结果。

- `WaypointManager` 用平均纬度 cos：`src/flight_dynamic/guidance/WaypointManager.cpp:24`（`mean_latitude_rad = 0.5 * (lat + origin_lat)`）、`:28`（`east_m = ... * cos(mean_latitude_rad)`）。
- `Maneuver` 用参考中心纬度 cos：`src/flight_dynamic/guidance/Maneuver.cpp:160`、`:203`（`cos_lat = std::cos(center.latitude_rad)`），并在文件内 6 处内联重复该投影。

为何未决：无法从代码判断哪个约定是有意为之。两者在小偏移下数值接近，分叉只在远场才显现；现有测试未覆盖"两套约定应一致"或"应不同"的断言。合并到单一投影需要先决定以哪个 cos-lat 约定为准。

推进需要：
- 领域知识确认：orbit / figure-8 / racetrack 几何中，cos(mean lat) 与 cos(center/reference lat) 哪个是正确意图；
- 决定后，要么统一为单一约定（带参数的 helper），要么显式记录"两者有意不同"并保留；
- 重测 orbit / figure-8 / racetrack 几何，确认无回归。

注：P2.5a（commit `ff0c9a2c`）只收拢了 `NormalizeRad`/`RadToDeg360`（角度归一化），明确未触碰此 NE 投影分叉。

## OQ-3 EOS replay 派生环境字段非对称 codec

`EosSessionConfig` 的 replay codec 对派生环境字段是非对称的：encode 写入、decode 丢弃。schema 注释自称这些字段"冗余记录以支持精确比对"，但 decode 端从不读取它们。

- encode 写入派生字段：`src/electro_optical_sensor/session/EosReplayFlatbufferCodec.cpp:338`（`BuildModelConfigFromScenario` 后写入 `radiative_transfer_model_derived` 等）。
- decode 不读派生字段：`DecodeEosSessionConfig` 只还原 `scenario_config`，从不读 `*_derived`（依赖 consumer 重新 `BuildModelConfigFromScenario`）。
- schema 注释自述冗余：`schemas/replay/eos_session_replay.fbs` 的 `EosEnvironmentConfig`。

为何未决：当前行为正确（consumer 会重新 derive），不是 bug。但这是 source-of-truth / drift 隐患——若 `BuildModelConfigFromScenario` 的 preset→factor 映射改变，replay buffer 的"冗余比对"字段会与 fresh decode 静默分叉，且现有 roundtrip 测试无法捕获（decode 不读这些字段）。

推进需要：先补一个断言"派生字段 encode/decode 一致"的 roundtrip 测试，再决定是让 decode 读回派生字段、还是从 schema 删除它们。

注：P2.2（commit `6ad98476`）只收拢了 detection-record 的 encode/decode，明确未触碰此派生字段非对称。

## OQ-4 飞行动力学失速速度 ρ 来源漂移

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
