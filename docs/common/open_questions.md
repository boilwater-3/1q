# 跨模块开放议题

Status: active
Authority: 非规定性记录

本文登记调查中发现但尚未定论的跨模块架构议题，不构成契约约束。条目推进到有结论时，应回写为契约规则（进 contract.md）或模块设计（进对应 design.md），并从本文移除。

## 当前修复优先级（2026-07-17 实时代码复核）

以下排序按“已经能证明存在运行时语义风险”优先于“需要 public API 迁移决策”，再优先于“纯机械重构”排列。这里的 P0/P1/P2 是修复顺序，不是线上安全等级；在完成对应失败测试和契约冻结前，不直接修改 public struct 或 replay schema。

| 优先级 | 条目 | 当前判断 | 首个交付物 |
|---|---|---|---|
| P1 | OQ-10c、OQ-10g、OQ-10j、OQ-1 | 生效优先级或几何退化路径可由代码复现，但尚缺统一契约 | 组合矩阵/远场几何测试 + 设计文档 |
| P1 | OQ-10b、OQ-10e、OQ-10f、OQ-10k、OQ-10l | 主要是公开配置误导或跨域语义不一致，先补文档和守护 | 字段生效表、四域规则、no-op 标注 |
| P2 | OQ-8、OQ-10a、OQ-10d、OQ-10i、OQ-10m | 涉及 public API/ABI 或跨模块迁移，不能作为顺手清理 | Stage A 迁移契约和 consumer 影响清单 |
| P2 | OQ-9 | 机械 replay helper 收敛，必须在模块行为护栏稳定后进行 | SBIRS → SAR → AR 分批证明 |

排序依据是当前 checkout 的代码和测试，不代表这些条目已经获准实施。原 OQ-10h 已完成：SBIRS 物理 sigma 按 RSS 合成，legacy 仅在三项全零时生效，bearing/covariance 共用同一解析语义。原 OQ-3 也已完成：Autopilot 使用当前重量/密度刷新动态 TAS 包线，起飞旋转与默认进近速度统一使用标准海平面密度作为 CAS 基准。

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

### OQ-10b 🔴 SAR `SarEnvironmentConfig` 整域是 no-op 却作为对外公开四域之一

该域头文件注释自认："当前 Phase 1 计算链路（raw echo 生成、L1 RDA、L3 BP、质量摘要）**不消费本域任何字段**"。5 个字段全部标注"保留字段：当前不驱动任何计算阶段"。

- 证据：`include/1q/sar/config/SarEnvironmentConfig.h`（struct 级 `@note` 与各字段注释）。
- 影响字段：`terrain_reference_altitude_m` / `atmospheric_loss_db_per_km` / `surface_backscatter_sigma0_db` / `use_flat_earth_geometry` / `enable_atmospheric_attenuation`。
- 用户配置这些旋钮对成像输出零影响，仅 `SarReplayFlatbufferCodec` 用于 replay 保真与 config 透传。

为何未决：该域存在的合法理由是 replay 保真与未来按域接入，但作为对外公开四域之一会误导用户以为可调参。下沉到 internal 还是保留并显著标注，需结合对外 API 收敛策略决定。

推进需要：
- 决定该域是下沉到 `src/` 内部、还是保留在公开 API 但加 `[[deprecated("reserved for replay fidelity, no compute effect")]]` 并在 README/Builder 文档显著标注；
- 若保留，需在 `SarSessionConfigBuilder` 文档与 `docs/sar/design.md` 明确"本域当前不影响输出"；
- 同步检查同类"保留字段"先例（见 OQ-10g）的处置一致性。

### OQ-10c 🔴 AR 硬件域 `frequency_hz` 三处出现 + 隐式回退优先级

AR 硬件域 `frequency_hz` 出现在两个子配置中，且有第三处隐式回退逻辑：

- `TransmitterConfig.frequency_hz{3e9f}`：探测/波长/大气损耗主源。证据 `src/airborne_radar/signal/pipeline/DetectionExecution.cpp:70`、`:252-253`、`src/airborne_radar/signal/detection/RadarEquations.cpp:100`。
- `RcsPhysicsConfig.frequency_hz{0.0f}`：物理 RCS 估计频率。默认 `0.0f`。
- `ResolveRcsPhysicsFrequencyHz`（`src/airborne_radar/signal/pipeline/DetectionExecution.cpp:25-31`）：`rcs_physics.frequency_hz > 0` 则用之，否则**静默回退** `transmitter.frequency_hz`。

为何未决：用户面对两个 `frequency_hz` 不知道改哪个生效；`RcsPhysicsConfig.frequency_hz` 默认 `0.0f` 实为"继承 transmitter"的魔法值，但字段注释（`include/1q/airborne_radar/config/ArHardwareConfig.h:178`）只写"物理 RCS 估计使用的频率"，未披露这个 0→继承的隐藏契约。删除冗余字段会改变 struct 布局，需确认无外部代码按字段偏移访问。

推进需要：
- 决定方案：删除 `RcsPhysicsConfig.frequency_hz`（RCS 估计直接复用 transmitter），或在头文件注释显式写明"0 = 继承 transmitter.frequency_hz"契约；
- 若删除，更新 `ArReplayFlatbufferCodec` schema 与回放兼容策略；
- 补一条针对"rcs_physics.frequency_hz = 0 与 = transmitter 值应同效"的契约测试。

### OQ-10d 🟠 ScenarioConfig / ModelConfig "双胞胎 struct"，字段完全相同却禁止视为同型

AR 与 ESR 的环境域各有一对字段 100% 相同的 ScenarioConfig / ModelConfig，但注释禁止视为同型。

- AR：`EnvironmentScenarioConfig` 与 `EnvironmentModelConfig` 字段完全一致，`BuildModelConfigFromScenario` 为逐字段拷贝。证据 `include/1q/airborne_radar/config/ArEnvironmentConfig.h:131-188`（struct 定义在 `:131`/`:147`，映射函数在 `:180-188`，禁止 alias 的注释在 `:144-146`）。
- ESR：`EsrEnvironmentScenarioConfig` 与 `EsrEnvironmentModelConfig` 三字段全等。证据 `include/1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h:37-48`。
- 对比：EOS 是唯一真正有差异的——ModelConfig 把 ScenarioConfig 的 `custom_overrides` 子结构扁平化了（`include/1q/electro_optical_sensor/config/EosEnvironmentConfig.h:56-76`），算合理。

为何未决：注释"禁止退化为 type alias""调用方不得假设同型"与当前实现的恒等映射自相矛盾；用户无从判断两者何时会有差异，也无法从代码证明差异不会发生。

推进需要：
- 决定 AR/ESR 的 ModelConfig 是否有未来差异化需求（增加派生字段或移除场景字段）；
- 若无需求，退化为 `using ModelConfig = ScenarioConfig;` 并移除 `BuildModelConfigFromScenario`；
- 若保留，补一条"两 struct 字段集差异"的契约测试，并在注释中给出差异化的具体计划而非"未来可能"。

### OQ-10e 🟠 跨域"探测策略"归属不一致

AR 把 `DetectionPolicyConfig`（cfar_pfa / min_snr_db）放在**硬件域**的 `DetectionConfig` 内，ESR 把同概念放在**策略域**。

- AR：`DetectionPolicyConfig` 嵌在 `DetectionConfig`（硬件域），`include/1q/airborne_radar/config/ArHardwareConfig.h:148-151`（定义）、`:196`（聚合成员 `detection_policy`）。`ArHardwareConfig` 本身就是 `DetectionConfig` 的别名（`:208`）。
- ESR：`EsrDetectionPolicyConfig`（minimum_snr_db / pfa / pulse_count）独立放在策略域，`include/1q/electronic_surveillance_radar/config/EsrPolicyConfig.h`。

为何未决：四域划分的语义边界本应一致，但"探测门限/虚警概率"在 AR 被当作硬件固有能力、在 ESR 被当作策略。用户在 ESR 找探测门限去 policy，在 AR 却要去 hardware。迁移会改变 AR `ArHardwareConfig` 的结构（它是 alias），影响面大。

推进需要：
- 决定探测策略的归属标准（按"是否硬件固有能力"判定）；
- 若统一到 policy 域，规划 AR `DetectionPolicyConfig` 迁出 hardware 的迁移路径与 replay schema 兼容；
- 在 `docs/common/contract.md` 的四域划分规则中补一条"探测门限类参数归属"明确规则。

### OQ-10f 🟠 ESR `scan_rate_hz` 与 EOS/SBIRS `scan_rate_deg_per_sec` 跨域量纲不一致

同名 `scan_rate` 前缀，ESR 是数据率（Hz，标量刷新率），EOS/SBIRS 是角速度（deg/s），量纲不同。

- ESR：`EsrScanPolicyConfig.scan_rate_hz{1.0f}` 注释"扫描数据率（单位：Hz）"，`include/1q/electronic_surveillance_radar/config/EsrMissionConfig.h:36`。
- EOS：`scan_rate_deg_per_sec{20.0f}`（角速度），`include/1q/electro_optical_sensor/config/EosMissionConfig.h:30`。
- SBIRS：`scan_rate_deg_per_sec{10.0f}`（角速度），`include/1q/sbirs_sensor/config/SbirsMissionConfig.h`。

为何未决：ESR 作为电子侦察，"扫描数据率"与机械/电子扫描角速度确实是不同物理量，但共用 `scan_rate` 前缀会误导跨模块复用配置的用户。改名涉及对外字段名变更。

推进需要：
- 确认 ESR 的 `scan_rate_hz` 是否真的是刷新率而非角速度（核对 `src/electronic_surveillance_radar/` 消费点语义）；
- 若是刷新率，改名为更明确的 `scan_update_rate_hz` 或 `frame_rate_hz` 以与角速度区分；
- 评估跨模块配置复用场景是否真实存在，决定改名优先级。

### OQ-10g 🟠 ESR 扫描范围双重表达，靠布尔切换

`EsrScanPolicyConfig` 同时提供中心式与显式起止式两套表达，靠 `use_explicit_scan_bounds` 切换语义。

- 证据：`include/1q/electronic_surveillance_radar/config/EsrMissionConfig.h:33-44`。
- 字段：中心式 `scan_center_az_deg` / `scan_center_el_deg`；显式起止式 `scan_start_az_deg` / `scan_end_az_deg` / `scan_start_el_deg` / `scan_end_el_deg`；切换开关 `use_explicit_scan_bounds{false}`。

代码复核后的确定事实：当 `use_explicit_scan_bounds=true` 且四个起止角均为有限值时，显式起止边界直接生效并提前返回；否则才使用中心角和扫描范围推导边界。运行期 patch 设置中心角时会主动清除显式模式，设置显式边界时则切换到显式模式。因此当前未决点不是“谁优先未知”，而是 public 结构仍允许同时填写两套值，且静态配置缺少冲突校验/显式生效表。

推进需要：
- 在 `EsrMissionConfig`/`EsrSessionConfigBuilder` 文档中写明上述生效优先级，并补一条静态配置的冲突/退化组合测试；
- 决定二选一（推荐保留显式起止式，语义无歧义），或保留两套但在头文件显式写明优先级与一致性约束；
- 补一条"两套表达冲突时的解析行为"契约测试。

### OQ-10i 🟡 SBIRS `detector_area_m2` vs EOS `detector_area_cm2` 同物理量单位不一致

同是"探测器面积"，SBIRS 用 m²，EOS 用 cm²，跨域复制易错 4 个数量级。

- SBIRS：`SbirsHardwareConfig.detector_area_m2{1.0e-4f}`，`include/1q/sbirs_sensor/config/SbirsHardwareConfig.h`。
- EOS：`EosHardwareConfig.detector_area_cm2{0.25f}`，`include/1q/electro_optical_sensor/config/EosHardwareConfig.h:23`。

为何未决：两个传感器物理量纲习惯不同（红外探测器传统用 cm²），但对外公开 API 单位不一致增加跨域误用风险。统一单位是源码兼容性变更。

推进需要：
- 决定是否统一到 SI（m²），评估对 EOS 现有消费方与文档的影响；
- 若不统一，在 `docs/common/contract.md` 补一条"跨域同物理量单位须在字段名后缀标明"的规则并加 lint 守护；
- 字段名后缀已带单位（`_m2` / `_cm2`），最低限度应确保文档显著标注差异。

### OQ-10j 🟡 AR 硬件域多个"0=特殊语义"魔法值，且 0 在不同字段含义相反

同一个 0，在 `nominal_az_beamwidth_deg` 是"自动推导"，在 `antenna_length_m` 是"禁用功能"，方向相反，依赖组合判断。

- 证据：`include/1q/airborne_radar/config/ArHardwareConfig.h:137-140`。
- `nominal_az/el_beamwidth_deg{4.0f}`：注释"为 0 且 antenna_length_m>0 时从物理尺寸推导"。
- `antenna_length_m{0.0f}` / `antenna_width_m{0.0f}`：注释"0 = 不使用物理推导"。
- 组合语义：`beamwidth=0 && length>0` → 推导；`beamwidth!=0` → 直接用；`beamwidth=0 && length=0` → 语义未定义（无 backstop）。

为何未决：魔法值 0 在不同字段含义相反，且 `beamwidth=0 && length=0` 的退化组合无明确回退。用户难以推断生效路径。

推进需要：
- 决定是否引入显式枚举（如 `BeamwidthSource{ kNominal, kDerivedFromAperture }`）替代 0 魔法值；
- 或在头文件注释补一张"字段组合 → 生效路径"真值表，并明确 `beamwidth=0 && length=0` 的回退；
- 补一条覆盖三种组合的契约测试。

### OQ-10k 🟡 EOS 环境域三个正交枚举组合空间爆炸，优先级未说明

EOS 环境域有四套正交开关，头文件未说明哪个优先、谁覆盖谁。

- 证据：`include/1q/electro_optical_sensor/config/EosEnvironmentConfig.h:18-63`。
- 枚举/开关：`EosEnvironmentModelType`（2 项，`:27`）、`EosEnvironmentPreset`（5 项，`:35`）、`RadiativeTransferModel`（3 项，`:18`）、`has_custom_overrides` + `EosEnvironmentCustomOverrides`（`:59-60`、`:46-51`）。

为何未决：组合空间 2×5×3×2 = 60 种，许多组合语义重叠（如 `preset=kDusty` 与 `custom_overrides.aerosol_density_factor>1`），用户难以判断实际生效路径。ModelConfig 扁平化 custom_overrides 后优先级更隐晦。

推进需要：
- 核对 `src/electro_optical_sensor/` 中四套开关的解析优先级（preset → model_type → custom_overrides → radiative_transfer_model?）；
- 在头文件注释补一条优先级链说明，或在 `docs/electro_optical_sensor/design.md` 明确生效路径；
- 评估是否收敛枚举（如 preset 与 custom_overrides 二选一），减少组合空间。

### OQ-10l 对外公开 API 中的保留字段（注释为真，设计本身值得商榷）

以下字段注释明确标注"不进入计算链路"，经核验确实仅被对应 `*ReplayFlatbufferCodec.cpp` 序列化、不进计算。注释诚实，但作为对外公开 API 保留死字段本身反直觉——用户会尝试调它期望生效。

- AR `BeamPointingConfig.default_scan_center_deg`：注释"当前真实扫描中心唯一来源是 `ArMissionConfig::orientation.scan_center_deg`"。证据 `include/1q/airborne_radar/config/ArPolicyConfig.h:30`；消费仅 `src/airborne_radar/session/ArReplayFlatbufferCodec.cpp:545,759-760`。
- AR `AssociationConfig.use_distance_gate_hint` / `distance_gate_sigma_hint`：注释"当前关联门限由库内关联器自适应管理"。证据 `include/1q/airborne_radar/config/ArPolicyConfig.h:90,96`；消费仅 `src/airborne_radar/session/ArReplayFlatbufferCodec.cpp:557-558,777-778`。
- SAR `SarPolicyConfig.retain_raw_phase_history`：注释"当前 public result 不返回 raw phase history，仅 replay/config 保真"。证据 `include/1q/sar/config/SarPolicyConfig.h`。
- SAR `SarPolicyConfig.max_allowed_squint_angle_deg`：注释"当前 session 尚未实现 squint-angle runtime gate"。证据 `include/1q/sar/config/SarPolicyConfig.h`。

为何未决：这些字段为 replay/config 保真而保留，但放在公开 `include/` 下会诱导用户调参。下沉到 internal 会破坏 replay schema 的对外可见性；保留则需更强的"无效"标注。

推进需要：
- 决定统一策略：要么将这些字段下沉到 `src/` 内部 replay DTO（公开 API 只留活跃字段），要么在公开 API 加 `[[deprecated]]` 或统一的前缀（如 `reserved_`）显著标注；
- 与 OQ-10b（SAR 整域 no-op）的处置保持一致；
- 任何一种方案都需同步 replay codec 与 consumer 测试。

### OQ-10m 🟡 AR `AntennaConfig` 缺 frequency，隐式借用 transmitter

天线波束宽度推导、sinc² 方向图都需要波长（频率），但 `AntennaConfig` 无 frequency 字段，全部隐式借 `transmitter.frequency_hz`。

- 证据：`include/1q/airborne_radar/config/ArHardwareConfig.h:135-143`（`AntennaConfig` 无频率字段）。
- 消费侧耦合：`src/airborne_radar/signal/pipeline/DetectionExecution.cpp:252-253` 用 `transmitter.frequency_hz` 算波长。

为何未决：天线物理参数与频率强耦合，却不在同 struct 内表达，耦合关系隐式跨子配置。与 OQ-10c 的 frequency 重复问题方向相反——一个重复、一个缺失，根源都是 frequency 归属未理清。

推进需要：
- 与 OQ-10c 合并决策：理清 hardware 域 frequency 的唯一归属（推荐放 `DetectionConfig` 顶层或 `TransmitterConfig`，RCS 与天线均显式引用之，消除 `RcsPhysicsConfig.frequency_hz`）；
- 或在 `AntennaConfig` 注释显式声明"波长依赖 `transmitter.frequency_hz`"；
- 评估是否引入 `DetectionConfig::frequency_hz` 作为单一频率源。
