# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

## 当前修复优先级（2026-07-18 实时代码复核）

以下排序按“已经能证明存在运行时语义风险”优先于“需要 public API 迁移决策”，再优先于“纯机械重构”排列。这里的 P0/P1/P2 是修复顺序，不是线上安全等级；在完成对应失败测试和契约冻结前，不直接修改 public struct 或 replay schema。

| 优先级 | 条目 | 当前判断 | 首个交付物 |
|---|---|---|---|
| P2 | OQ-8、OQ-10a、OQ-10d、OQ-10i | 涉及 public API/ABI 或跨模块迁移，不能作为顺手清理 | Stage A 迁移契约和 consumer 影响清单 |
| P2 | OQ-9 | 机械 replay helper 收敛，必须在模块行为护栏稳定后进行 | SBIRS → SAR → AR 分批证明 |

排序依据是当前 checkout 的代码和测试，不代表这些条目已经获准实施。原 OQ-10h、OQ-3、OQ-10c、
OQ-10b、OQ-10e、OQ-10f、OQ-10g、OQ-10j、OQ-10k、OQ-10l、OQ-10m、OQ-1 已完成；对应运行时语义和证据已迁入
SBIRS、Flight Dynamic、AR、ESR、EOS、SAR 的模块设计权威。

---

## OQ-8 折射率成对温度输入的 public 迁移

原 OQ-8 的低风险收尾已复核：L3 不能移除 `GeometryTransform.h` 的 `Eigen/Core`，因为该头直接以 `Eigen::Vector3f` / `Eigen::Matrix3f` 作为函数返回值和参数；L4（`src/common` 的 `reset(new)`）已经不存在。两项均无需代码修复。

剩余的 L6 不再是低风险样式项：`refractivity_index_n_r4/r8` 和公开的 `RefractivityIndex` 同时接收摄氏与开氏温度。两个裸浮点参数可被调换，但改变为成对温度类型或单一温标会改变 REOS 对齐的 public 签名。

为何未决：仓库内只有转发实现和一致温标的单测，无法证明仓库外调用方不依赖当前签名。静默派生其中一个温度会改变不一致输入的数值语义，也不能可靠修复“参数被调换”。

推进需要：独立 Stage A 冻结 public migration（新 typed input/过渡入口、REOS 对齐、外部 consumer 期限），并补充温标不一致的拒绝或诊断契约。\
[evidence: `include/1q/environment/PropagationPhysics.h:RefractivityIndex` — public 六标量签名;\
 `src/common/atmosphere/AtmospherePhysics.cpp:refractivity_index_n_r8` — 同时消费 Celsius 与 Kelvin;\
 `tests/unit/airborne_radar/ar_atmosphere_physics_test.cpp` — 当前只覆盖一致温标]

## OQ-9 Replay FlatBuffer internal helper 的后续迁移边界

EOS/ESR 已把“完成 builder 后复制字节”和统一 `FailureMarker` 解码保护迁入
`src/common/replay/ReplayFlatbufferCodecSupport.h`。SBIRS、SAR、AR 仍保留相似机械代码，但三者的
schema、DTO、payload identifier、错误文本和 replay 行为必须继续由模块拥有，不能借迁移合并为万能 codec。

当前现场：

- SBIRS 同时重复 buffer 复制与 `FailureMarker` 空值/空 payload/verifier/错误传播，现有 roundtrip、损坏拒绝、
  多目标 IMM、coasting 和扰动 replay 可作为首批行为护栏。
- SAR 重复 buffer 复制和各 payload verifier；其 raw-IQ 外部数据边界及 L1/L2/L3 结果结构属于模块语义，
  不应进入公共 helper。
- AR 的 codec 对象图和 identifier 处理最复杂，并有更完整的 corruption 与 failure-marker 行为，因此只在
  SBIRS、SAR 两批证明 helper 边界稳定后最后迁移。

推荐 probe 顺序为 **SBIRS → SAR → AR**。每批只允许迁移无 schema 知识的机械路径；必须保持编码结果、
空值/截断/损坏拒绝、failure marker、错误文本和 divergence 行为，并通过对应 replay partition、contract、
public boundary、install manifest 与 C++11 compatibility。若复用需要接触模块 DTO 转换、payload identifier、
外部数据资格或改变错误语义，则停止迁移并保留模块实现，不扩大 helper。

[evidence: `tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test.cpp:SbirsReplayCodecRoundtripTest.DecodeFailureMarkerRejectsNullAndCorrupted` — SBIRS failure-marker 拒绝行为;\
 `tests/replay/sbirs_sensor/sbirs_replay_session_test.cpp:SbirsReplaySessionTest.ReplayPreservesTrackingCoastAndGateLoss` — SBIRS 结果重放语义;\
 `tests/replay/sar/sar_replay_codec_roundtrip_test.cpp:SarReplayCodecRoundtripTest.RejectsEmptyPayload` — SAR 空 payload 拒绝;\
 `tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp:ArReplayCodecRoundtripTest.DecodeFailureMarkerRejectsNullAndCorrupted` — AR failure-marker/corruption 行为]

## OQ-10 四域对外配置结构体成员合理性与反直觉审查

对 5 个传感器模块（AR / ESR / EOS / SAR / SBIRS）的对外公开四域配置（hardware / mission / policy / environment）共 20 个头文件做了逐字段审查与实际消费路径核验，登记以下反用户直觉问题。按严重度分级；每条均附代码证据。结论前缀含 🔴严重 / 🟠中等 / 🟡轻微。

### OQ-10a 🔴 SBIRS `sensor_enabled` 与其余三域 `power_on` 同概念跨域异名

AR/ESR/EOS 的开关机字段都叫 `power_on{true}`，唯独 SBIRS 叫 `sensor_enabled{true}`。

- AR `include/1q/airborne_radar/config/ArMissionConfig.h:23`、ESR `include/1q/electronic_surveillance_radar/config/EsrMissionConfig.h:50`、EOS `include/1q/electro_optical_sensor/config/EosMissionConfig.h:36`：`bool power_on{true}`。
- SBIRS `include/1q/sbirs_sensor/config/SbirsMissionConfig.h:24`：`bool sensor_enabled{true}`。
- 反差证据：`include/1q/airborne_radar/config/ArOrientationConfig.h:62-63` 注释明确写"命名对齐 EOS/ESR 的 work_mode…由 `check_cross_domain_naming.cmake` 守护不回归"——SBIRS 绕过了这套守护。

为何未决：改名是 ABI/源码兼容性破坏，需确认是否有外部消费方依赖当前字段名；同时需核对 `check_cross_domain_naming.cmake` 的守护范围是否本应覆盖此字段而漏检。

推进需要：
- 决定统一字段名（推荐 `power_on`，多数派）；
- 扩展或核对 `check_cross_domain_naming.cmake` 规则将该字段纳入守护，防回归；
- 评估改名对外部消费方的兼容性影响，必要时走别名过渡。

### OQ-10d 🟠 ScenarioConfig / ModelConfig "双胞胎 struct"，字段完全相同却禁止视为同型

AR 与 ESR 的环境域各有一对字段 100% 相同的 ScenarioConfig / ModelConfig，但注释禁止视为同型。

- AR：`EnvironmentScenarioConfig` 与 `EnvironmentModelConfig` 字段完全一致，`BuildModelConfigFromScenario` 为逐字段拷贝。证据 `include/1q/airborne_radar/config/ArEnvironmentConfig.h:131-188`（struct 定义在 `:131`/`:147`，映射函数在 `:180-188`，禁止 alias 的注释在 `:144-146`）。
- ESR：`EsrEnvironmentScenarioConfig` 与 `EsrEnvironmentModelConfig` 三字段全等。证据 `include/1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h:37-48`。
- EOS 已收敛为单一公开 ScenarioConfig，内部派生执行参数，不再公开同名 ModelConfig，因此不属于本议题。

为何未决：注释"禁止退化为 type alias""调用方不得假设同型"与当前实现的恒等映射自相矛盾；用户无从判断两者何时会有差异，也无法从代码证明差异不会发生。

推进需要：
- 决定 AR/ESR 的 ModelConfig 是否有未来差异化需求（增加派生字段或移除场景字段）；
- 若无需求，退化为 `using ModelConfig = ScenarioConfig;` 并移除 `BuildModelConfigFromScenario`；
- 若保留，补一条"两 struct 字段集差异"的契约测试，并在注释中给出差异化的具体计划而非"未来可能"。

### OQ-10i 🟡 SBIRS `detector_area_m2` vs EOS `detector_area_cm2` 同物理量单位不一致

同是"探测器面积"，SBIRS 用 m²，EOS 用 cm²，跨域复制易错 4 个数量级。

- SBIRS：`SbirsHardwareConfig.detector_area_m2{1.0e-4f}`，`include/1q/sbirs_sensor/config/SbirsHardwareConfig.h`。
- EOS：`EosHardwareConfig.detector_area_cm2{0.25f}`，`include/1q/electro_optical_sensor/config/EosHardwareConfig.h:23`。

为何未决：两个传感器物理量纲习惯不同（红外探测器传统用 cm²），但对外公开 API 单位不一致增加跨域误用风险。统一单位是源码兼容性变更。

推进需要：
- 决定是否统一到 SI（m²），评估对 EOS 现有消费方与文档的影响；
- 若不统一，在 `docs/common/contract.md` 补一条"跨域同物理量单位须在字段名后缀标明"的规则并加 lint 守护；
- 字段名后缀已带单位（`_m2` / `_cm2`），最低限度应确保文档显著标注差异。
