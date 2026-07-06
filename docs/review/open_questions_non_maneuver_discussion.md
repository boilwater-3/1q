# 非机动开放议题开发讨论

Status: draft
Last-reviewed: 2026-07-06
Authority: non-normative Stage A discussion for `docs/common/open_questions.md`

本文按 `skills/evidence-first-freeze-contract` 的 Stage A 流程讨论
`docs/common/open_questions.md` 中不属于机动/飞行动力学专题的开放议题。本文不是契约；条目只有在完成冻结契约、实现与验证后，才能回写到
`docs/common/contract.md` 或对应模块 `design.md`，并从 `open_questions.md` 移除。

本轮排除：OQ-1、OQ-3、OQ-5。它们分别涉及 flight_dynamic 局部 NE 投影、失速速度大气密度来源、以及 flight_dynamic public 边界形态，留给飞行动力学/机动专题处理。

## Stage A 证据矩阵

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|-------------|------------|-----------------|------------|----------------|--------------------|----------|
| OQ-2 EOS replay 派生环境字段 | Schema 声称派生字段用于精确比对，但 decode 不消费这些字段，存在 source-of-truth 漂移风险 | `schemas/replay/eos_session_replay.fbs:54-62`、`src/electro_optical_sensor/session/EosReplayFlatbufferCodec.cpp:338-353`、`:360+` | 先补一个 roundtrip/flatbuffer inspection 测试，证明 buffer 中派生字段与 decode 后重新 derive 的值保持一致或能被检测 | 测试能在 preset/custom override 组合下暴露派生字段与 scenario 重新 derive 之间的关系 | 现有 schema 注释被改为明确"write-only debug snapshot"，且测试证明 consumer 不需要精确比对 | narrow |
| OQ-4 AR/ESR Session Impl owned_ptr + 引用冗余 | AR/ESR `Impl` 同时持有 owning pointer 与同对象引用，当前可运行但移动/所有权重构边界不清 | `src/airborne_radar/session/ArSession.cpp:34-40`、`:228-235`，`src/electronic_surveillance_radar/session/EsrSession.cpp:15-20`、`:66-71`，EOS 对照 `src/electro_optical_sensor/session/EosSession.cpp:35-45` | 检查外部 move 语义与成员访问；设计小步改造，优先把 AR/ESR 成员访问改为 helper/owned object dereference，或显式锁死 `Impl` 移动赋值 | 可在不改 public `Session` move API、不改业务语义的前提下降低悬垂引用风险 | 若 AR/ESR composition 需要非 owned 注入长期存活，必须保留引用成员并补契约说明 | narrow |
| OQ-6A 数值下限常量归类 | `kNormFloor` 与 `kNumericFloor` 名称相近但语义不同，机械合并可能改变阈值语义 | `src/common/coordinate/*`、`src/electro_optical_sensor/session/EosExternalInputAdapter.cpp`、`src/common/numerics/NumericGuard.h` | 先列出调用点并分类为向量/范数退化、防除零数值保护、物理范围下限 | 每个调用点能归入一个明确语义桶，并有现有或新增单测冻结边界值 | 存在模块专属物理阈值，不能下沉到 common numerics | defer |
| OQ-6B target/emitter id=0 严重级别 | 当前 public DTO 允许实体 ID 为 0，EOS/ESR 不应把 0 视为 validation error | AR `external_target_id` 已允许 0；EOS/ESR validation 曾对 0 产生诊断 | 写 contract draft：`0` 是合法 ID 值；负数若未来从 signed 外部入口进入，必须在转成 public `uint64_t` DTO 前拒绝 | contract 和 tests 都证明 `target_id == 0` / `emitter_id == 0` 不产生 validation error | 发现下游生命周期/关联逻辑无法处理 0 且无替代键 | narrow |
| OQ-7 JSON parser 长期替换 | 自研 parser 已有安全加固，但 API 仍是 public surface；替换成熟库会牵动 `JsonValue` 契约 | `include/1q/foundation/json_reader.h:33-36`、`src/common/config/JsonReader.cpp:11`、`:113-148`、`:253-263`、`tests/unit/json_reader_test.cpp` | 先补负例测试覆盖数字语法缺口，再评估是否需要新 API；成熟库替换必须以兼容层/迁移契约为前置 | 发现真实配置消费路径需要 JSON 标准完整性或 API 表达力，且兼容迁移可控 | 当前 consumer 只需要轻量配置子集，继续 harden 的风险/成本更低 | defer |
| OQ-8 common 层收尾 | L3/L4 是低风险机械收尾，L6 是签名语义风险，不能混批 | `src/common/geometry/GeometryTransform.h:9`、`reset(new T)` 调用点、`src/common/atmosphere/AtmospherePhysics.cpp:63-76` | 拆成三个批次：Eigen include 编译优化、C++11 `reset(new)` 保守替换、refractivity 温标签名契约 | L3/L4 可用编译/单测证明零语义变化；L6 有 REOS 对齐契约与调用点测试 | L6 若无法在 REOS 侧同步或命名不能消除误传风险，则延期 | narrow |

## 讨论结论

### OQ-2：先测试冻结，再二选一处理 schema

现场证据支持这个问题继续存在：encode 端把 `BuildModelConfigFromScenario` 派生出的
`radiative_transfer_model_derived`、`aerosol_density_factor_derived`、
`turbulence_factor_derived` 写入 flatbuffer；schema 注释称其用于精确比对；decode 端则只还原
`scenario_config`，不读取这些派生字段。

这不应直接进入"让 decode 读回派生字段"实现，因为那会改变 source-of-truth：当前 EOS 运行时仍以
`scenario_config -> BuildModelConfigFromScenario` 为权威。最小开发批次应是：

1. 增加一个 EOS replay session config 测试，直接 inspect flatbuffer 中的派生字段，并断言它们等于当前 scenario 重新 derive 的结果。
2. 再决定 schema 方向：要么删除/降级这些字段为 debug snapshot，要么把"精确比对"变成显式 replay validation，不让 decode 静默忽略。

Stage B 候选边界：`tests/unit/eos_replay_codec_roundtrip_test.cpp`、`src/electro_optical_sensor/session/EosReplayFlatbufferCodec.cpp`、`schemas/replay/eos_session_replay.fbs`，以及 generated header 重新生成。不得顺手改变 EOS runtime config 解析语义。

### OQ-4：可以进入 AR/ESR 内部所有权小步重构

AR/ESR 的风险不是现有运行时 bug，而是对象所有权表达不干净：成员列表中既有 `std::unique_ptr`，又有从同一 composition 指针初始化出的引用成员。`Session` 的 move API 保持 public 可移动，但 `Impl` 自身的约束没有在代码/契约里显式表达。

推荐走窄实现：优先把 AR/ESR 的依赖访问收敛为 `RequireCompositionDependency(owned_x.get(), "...")` 风格，或提供小型 accessor，通过 owning member 取引用。这样不改变 public `Session` 的 move 形态，也不改变 controller/pipeline 语义。

Stage B 候选边界：`src/airborne_radar/session/ArSession.cpp`、`src/electronic_surveillance_radar/session/EsrSession.cpp`，必要时同步 `docs/common/contract.md` 的 session composition 所有权规则。不得改 public headers、不得改 AR decision SPI、不得改 ESR immediate-submit 语义。

### OQ-6：拆成两个 contract 问题

OQ-6 不能作为一个批次处理。`kNormFloor`/`kNumericFloor` 是数值语义分类问题；`target_id=0` 是跨模块实体标识 contract 问题。二者证据来源、测试入口和风险完全不同。

OQ-6B 的新结论是：当前 public DTO 中实体 ID 可以为 0，`target_id == 0` /
`emitter_id == 0` 不得触发 validation error。由于这些字段当前是 `std::uint64_t`，负数
无法在 DTO 内表达；若未来新增 signed 外部输入入口，负数必须在转换为 public DTO 前拒绝。
该结论已进入 `docs/common/contract.md`，OQ-6 的开放问题只保留数值 floor 分类。

本轮后续推进结果：OQ-6A 已完成证据归类并进入 `docs/common/contract.md`。当前 floor 分为三类：common numerics 通用数值防护、common/coordinate 坐标/姿态退化阈值、模块局部输入几何退化阈值。结论是拒绝机械合并为单一全局常量；OQ-6 已从 `open_questions.md` 移除。

### OQ-7：延期替换库，先补标准语法负例

JSON parser 当前已经有深度、尾随内容、`\uXXXX` 完整性和 surrogate 拒绝测试；剩余问题有两类：public API 设计（缺失键返回静态 null 引用）和 JSON 标准完整性（数字语法、surrogate pair 合成）。引入成熟库不是本轮可以直接证明的最小改动，因为它会牵动 `JsonValue` public API 或至少引入兼容层。

推荐下一步不是替换库，而是增加几个不改变 API 的负例测试，例如 `1.`、`1e`、`01` 这类数字语法；若测试暴露 parser 接受非法输入，再做局部 harden。只有当配置消费路径证明需要完整 JSON 标准或更强 API 表达力时，才起草替换契约。

本轮后续推进结果：已补 `1.`、`1e+`、`01` 负例，并在 `JsonReader::ParseNumber` 中拒绝小数点后无数字、指数后无数字和前导零。成熟 JSON 库替换仍未被证明为最小必要变更，继续保留为策略开放项。

### OQ-8：L3/L4 可独立收尾，L6 必须单独契约化

L3 Eigen include 与 L4 `reset(new T)` 是低风险维护项，可以拆成小 commit，用编译和相关单测证明零语义变化。L4 需要注意 C++11 环境：若没有 `std::make_unique`，应使用 repo 内兼容 helper 或保持 `reset(new)`，不要为了样式引入 C++14 依赖。

本轮检查结果：仓库公共头明确禁止 `std::make_unique` 等 C++14 特征，当前也没有 repo-local C++11 `make_unique` helper。L4 暂不做机械替换，避免为了样式引入新 helper 或 C++14 依赖。

L6 不属于样式收尾。`refractivity_index_n_*` 同时接收摄氏与开尔文温度，确实容易误传；但改签名会影响 common/REOS 对齐，应单独写 contract：命名、单位、调用点迁移、兼容策略、测试接受条件都要冻结后再动。

## 建议执行顺序

1. OQ-6B：起草并落地 target/entity id contract，更新 EOS/ESR validation 与相关测试，使实体 ID 为 0 成为合法输入。
2. OQ-2：加 EOS replay 派生字段 characterization test，再决定 schema 字段是 validation 还是 debug snapshot。
3. OQ-4：AR/ESR Session Impl 所有权表达重构，配合 session composition contract。
4. OQ-8 L3/L4：小步机械收尾；L6 另起冻结契约。
5. OQ-6A/OQ-7：保留为证据不足的后续调查项，先补调用点清单/负例测试。

## 冻结契约草案：第一批可执行项

Proven requirement:
- 非机动开放问题中，OQ-6B、OQ-2、OQ-4 已有足够 live evidence 支持进入下一批窄实现或契约化。

Allowed scope:
- OQ-6B：`docs/common/contract.md`、EOS/ESR input validation 相关实现与测试。
- OQ-2：EOS replay codec 测试、EOS replay schema 注释/字段策略、必要的 FlatBuffers generated 更新。
- OQ-4：AR/ESR session impl 内部持有方式与 session composition 所有权契约。

Explicitly out of scope:
- flight_dynamic / maneuver 相关 OQ-1、OQ-3、OQ-5。
- public `include/1q` 新增 API。
- AR decision SPI、ESR immediate-submit 运行期配置语义。
- 测试阈值、skip、known-limit 规则调整。

Acceptance gates:
- OQ-6B：相关 unit tests 和 contract/docs guard 通过。
- OQ-2：`eos_replay_codec_roundtrip_test` 覆盖派生字段语义，FlatBuffers 生成物与 schema 一致。
- OQ-4：AR/ESR focused session tests 通过，相关 contract/unit tests 通过。

Non-goals:
- 不在本批次合并所有 common numerics helper。
- 不在本批次替换 JSON parser。
- 不把 flight_dynamic 重新包装成统一 cycle/session facade。

## Stage C Result

Implemented scope:
- OQ-6B：`docs/common/contract.md` 明确当前实体 ID / external ID 均允许 `0`；EOS/ESR input validation 不再因 `target_id == 0` / `emitter_id == 0` 产生 issue；对应 unit/contract tests 改为断言无 validation error。
- OQ-2：EOS replay session-config 测试直接 inspect FlatBuffer 派生字段，断言其等于 `BuildModelConfigFromScenario` 当前输出；EOS design 与 schema 注释明确 decode 仍以 `scenario_config` 为 source of truth，派生字段是编码侧快照。
- OQ-4：AR/ESR `Session::Impl` 去掉 `owned_x` + `X&` 并存成员，改为 owning member + accessor；`docs/common/contract.md` 增加 session composition ownership 规则。OQ-2/OQ-4 已从 `open_questions.md` 移除。
- OQ-6A：数值 floor 归类为通用数值防护、坐标/姿态退化、模块局部几何退化三类；`docs/common/contract.md` 规定不得机械合并不同语义 floor，OQ-6 已从 `open_questions.md` 移除。
- OQ-7：补 JSON 数字语法负例并 harden `JsonReader::ParseNumber`，拒绝 `1.`、`1e+`、`01` 等非法数字；成熟库替换策略仍保留为 OQ-7。

Validation:
- `cmake --build build/llvm-ninja-release-local --target 1q_unit_tests 1q_contract_tests -j 4`: pass。
- `git diff --check`: pass。
- `./build/llvm-ninja-release-local/bin/1q_unit_tests --gtest_filter='RadarInputValidationTest.*:EosInputValidationTest.*:EsrInputValidationTest.*:EosReplayCodecRoundtripTest.*'`: pass，103 tests。
- `./build/llvm-ninja-release-local/bin/1q_contract_tests --gtest_filter='EosPublicApiConvenienceTest.*:EsrPublicApiConvenienceTest.*:ArPublicApiConvenienceTest.*:PublicHeadersSmokeTest.*'`: pass，46 tests；`ArPublicApiConvenienceTest` 当前不是实际 suite 名。
- `./build/llvm-ninja-release-local/bin/1q_contract_tests --gtest_filter='PublicApiConvenienceTest.*:ArPrimaryNamingContractTest.*'`: pass，23 tests。
- `./build/llvm-ninja-release-local/bin/1q_unit_tests --gtest_filter='RadarSessionConfigBuilderTest.*:RadarSessionCreateWithValidationTest.*:ArEnvironmentRuntimePatchBehaviorTest.*:CoreControllerTest.*'`: pass，32 tests。
- `ctest --test-dir build/llvm-ninja-release-local -L '^unit$' --output-on-failure -j 4`: pass。
- `ctest --test-dir build/llvm-ninja-release-local -L '^contract$' --output-on-failure -j 4`: pass。
- `./build/llvm-ninja-release-local/bin/1q_unit_tests --gtest_filter='JsonReaderTest.*'`: pass，35 tests。
- `cmake --build build/llvm-ninja-release-local --target 1q_unit_tests 1q_contract_tests -j 4`: pass after JSON hardening。
- `ctest --test-dir build/llvm-ninja-release-local -L '^unit$' --output-on-failure -j 4`: pass after JSON hardening。
- `ctest --test-dir build/llvm-ninja-release-local -L '^contract$' --output-on-failure -j 4`: pass after JSON hardening。

Residual risks:
- OQ-7 JSON parser 成熟库替换策略仍未进入实现；数字语法负例已在本轮补齐。
- OQ-8 的 L3/L4/L6 仍需独立批次；L4 因 C++11 与缺少 repo-local helper 暂缓。

Follow-up freeze items:
- common low-risk cleanup 与 refractivity 温标签名契约。
