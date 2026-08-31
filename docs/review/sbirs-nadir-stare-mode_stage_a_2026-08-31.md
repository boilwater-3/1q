---
Status: final
Date: 2026-08-31
Review-Baseline: `evidence/sbirs-nadir-stare-mode` @ `8badb255`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# sbirs-nadir-stare-mode：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

1、触发来源：集成方在集成环境照抄场景 `scan_start_az_deg` 后目标全程不可见（2026-08-31 会话现场报告"看不到目标"）。
- **证据**：[evidence: examples/scenes/sbirs_triple_sat_fix_messages/sbirs_triple_sat_fix_messages.json]

2、冻结什么：`scan_start_az_deg` 是 ECI 绝对方位（惯性系角度，需按卫星经度与儒略日推算），集成方无换算工具、配错无告警，表现为静默 `not_in_output`。
3、什么证据能证明：配置校验层无数值域之外的几何校验；场景值恰等于星下点方位公式（说明"正确值"确实需要推算）。
4、什么证据能否定：装载层或校验层已存在等效防护或自动换算。
5、通过后最小改动范围：仅 sbirs_sensor 配置域 + 管线方位基准计算 + replay 编解码 + 聚焦测试 + 模块设计文档。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 需求：ECI 绝对方位对集成方不可靠 | 正确值必须按卫星位置推算，照抄/手推易错且错后无告警 | 1、字段注释明示 ECI 方位约定 [evidence: include/1q/sbirs_sensor/config/SbirsMissionConfig.h] ::scan_start_az_deg<br>2、校验仅查数值域，无几何一致性 [evidence: src/sbirs_sensor/session/SbirsSessionConfigValidation.cpp]<br>3、集成失败现场（2026-08-31 会话） | 实测三星场景配置值 vs 星下点方位公式 atan2(-y,-x)：154.79/34.79/274.79 vs 154.8/34.8/274.8，最大偏差 0.012°（JSON 一位小数舍入） | 存在真实误配模式且配置层无防护 | 装载层已有换算或防护 | pass |
| 原料：管线每周期已具备按位置现算方位基准的全部输入 | 无需新增任何输入通道即可实现 | 1、每周期必填 utc_julian_day 与卫星 ECEF 位置 [evidence: include/1q/sbirs_sensor/session/SbirsCycleInput.h]<br>2、gmst 与卫星 ECI 位置在扫描方位消费前已算出 [evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp] ::TryComputeGmstRad ::TryEcefToEci | 源码通读确认计算顺序：gmst→卫星 ECI→扫描方位 | 星下点方位可在现有流程内联计算 | 需新增公共输入字段或改公共 API | pass |
| 语义：新增"方位基准"枚举，不复用 work_mode | work_mode 是探测流程模式（待机/仅宽搜/搜索+凝视），与指向基准正交，复用会混淆语义 | 枚举定义与注释无任何指向语义 [evidence: include/1q/sbirs_sensor/config/SbirsMissionConfig.h] ::SbirsWorkMode | 通读枚举三值及注释 | 独立枚举 scan_azimuth_reference：eci_absolute（默认，行为逐位不变）/ nadir_relative | 语义有重叠应复用 | pass（narrow：仅方位基准，俯仰基准不动） |
| 影响面：replay 与契约测试必须同步 | mission 配置全量进出 replay 编解码，新字段必须进 schema 与往返测试 | 1、mission 逐字段编解码 [evidence: src/sbirs_sensor/session/SbirsReplayFlatbufferCodec.cpp] ::EncodeMissionConfig ::DecodeMissionConfig<br>2、往返测试覆盖 mission 字段 [evidence: tests/replay/sbirs_sensor/sbirs_replay_codec_roundtrip_test.cpp]<br>3、契约测试引用扫描字段 [evidence: tests/contract/sbirs_sensor/sbirs_public_api_convenience_test.cpp] | grep 确认 scan_start_az_deg 在 codec 与测试中的出现点 | 验收门 = 构建 + sbirs 聚焦测试 + replay 往返 + 契约测试全绿 | 存在不受影响的独立通道 | pass |
| 边界：nadir 模式下静态限位检查的能力边界 | 有效起点依赖运行期位置，config 期无法静态校验方位弧段是否越限 | 限位弧段检查以静态 scan_start 为输入 [evidence: src/sbirs_sensor/session/SbirsSessionConfigValidation.cpp] ::kScanPathOutsideSensorLimits | 读校验实现确认输入依赖 | nadir 模式跳过方位弧段静态检查（代码注释与设计文档声明），俯仰与限位合法性检查保留 | 能在 config 期等效校验 | narrow |
| 处置：方案1（扇区不含地球告警）本轮不做 | 用户已选定意图式配置（方案2），告警属补丁式叠加 | 用户裁定（2026-08-31 会话） | — | — | nadir 模式下扇区不含地球基本不可能（el 极端误配除外） | defer（el 误配出现真实案例再立冻结项） |

## §2 判定汇总与待裁定问题

1、需求（pass）：ECI 绝对方位是真实集成痛点，场景值即星下点公式的推算结果，配置层零防护。
2、原料（pass）：每周期已有儒略日与卫星位置，星下点方位可内联计算，无需动公共输入契约。
3、语义（pass/narrow）：独立枚举 scan_azimuth_reference，默认 eci_absolute 保证既有配置行为逐位不变；仅方位基准，俯仰基准不动。
4、影响面（pass）：replay schema + 往返测试 + 契约测试同步为验收门。
5、限位边界（narrow）：nadir 模式下方位弧段静态检查跳过，其余检查保留。
6、方案1 告警（defer）：被意图式配置覆盖，不叠床架屋。
7、待用户拍板的问题：已当轮裁定完毕，见修订记录 1。

## §3 冻结契约（用户讨论结束后填写）

### 已证明的需求

1、集成方可以用"相对星下点的偏移"表达扫描方位起点，不必手推 ECI 绝对方位。
2、默认（eci_absolute）模式下既有全部配置行为逐位不变。

### 允许范围

1、公开头文件：include/1q/sbirs_sensor/config/SbirsMissionConfig.h（新枚举 SbirsScanAzimuthReference + 新字段 scan_azimuth_reference，默认 kEciAbsolute）。
2、模块源码：src/sbirs_sensor/session/SbirsSessionConfigValidation.cpp（nadir 分支校验）；src/sbirs_sensor/pipeline/SbirsPipeline.cpp（方位基准计算与消费点）；src/sbirs_sensor/runtime/SbirsRuntimeConfigResolver.cpp（变更集比较纳入新枚举）。
3、回放：src/sbirs_sensor/session 的 flatbuffer schema 与 SbirsReplayFlatbufferCodec.cpp（新字段编解码）。
4、测试：tests/unit/sbirs_sensor（校验与指向聚焦测试）、tests/replay/sbirs_sensor（往返测试扩展）。
5、文档：docs/sbirs_sensor/design.md（字段语义权威回写）。
6、示例装载：examples/common/config_loaders/sbirs_sensor/config_loader_detail.h（JSON 字符串→枚举）。

### 明确禁止范围

1、不新增公共 API 函数、不改跨模块类型。
2、不改既有字段语义（scan_start_az_deg 在 eci_absolute 下仍是 [0,360) ECI 方位）。
3、不做方案1 告警、不做兼容层/迁移垫片（用户裁定：项目未上线）。
4、俯仰基准（nadir 化 scan_center_el）不做。

### 行为边界

1、输入：kNadirRelative 时 scan_start_az_deg 为相对星下点方位的带符号偏移（deg，有限且 |v| < 360）；星下点方位 = atan2(-eci_y, -eci_x)，逐周期由卫星当前位置现算。
2、输出：有效起点 = normalize(星下点方位 + 偏移)；span/rate/direction/el 语义不变；start=0 且 rate=0 即视场钉在星下点。
3、错误回退：kNadirRelative 偏移非法（非有限或 |v| ≥ 360）→ 配置校验错误（新增 issue code，沿用 kInputValidation phase）；卫星位置非法仍按既有输入校验拒绝。
4、限位：nadir 模式跳过方位弧段静态检查（运行期才知道有效起点），俯仰/限位合法性检查保留。
5、生命周期/回放：scan_azimuth_reference 进 replay 往返；运行期配置热更比较集纳入该枚举。

### 验收门

1、构建：cmake --build build/VisualStudio.15.0-amd64 --config Release（sbirs 相关目标）。
2、聚焦测试：tests/unit/sbirs_sensor 校验新分支；星下点指向端到端（GEO 位置 + nadir 偏移 0 → 星下点目标检出；偏移 90° → not_in_output）。
3、契约/回放：sbirs replay 往返含新字段；既有 sbirs 契约测试全绿。
4、特征化：eci_absolute 默认下既有 sbirs 测试零改动通过。

### 非目标

1、俯仰基准 nadir 化。
2、扫描过程中基准漂移补偿（LEO 场景的地球边缘跟踪）。
3、方案1 启动告警。

## 修订记录

修订 1（2026-08-31，用户指令）：裁定执行方案 2（意图式方位基准）；"目前项目没上线，该处理处理干净" → 不做兼容层与迁移垫片。默认值仍取 eci_absolute 并非兼容层，而是两种并存语义之一的取值选择：它使场景集与验收基线零联动改动，属最小实现原则。
修订 2（2026-08-31，用户指令）：基于 evidence 技能开始执行方案 2（触发本 Stage A）。

## §4 运行记录（Stage C 后填写）

1、实现范围：按冻结契约全量落地——`SbirsMissionConfig` 新增 `SbirsScanAzimuthReference` 枚举与 `scan_azimuth_reference` 字段（默认 kEciAbsolute）；配置校验双域分支（nadir 偏移域 (-360,360)，绝对域 [0,360) 维持）；管线每周期现算星下点方位基准（gmst+卫星 ECI 已有计算提升复用，无新增输入）；ApplyConfig 相位重锚带基准；运行时热更变更集纳入基准切换；replay schema/编解码新字段（raw 非法值原子拒绝）；examples JSON 装载器新枚举串。
2、验证命令与结果：
   1、`cmake --build ... --target 1q_lib`: pass（无告警级错误）。
   2、`1q_sbirs_sensor_unit_tests.exe --gtest_filter=*Nadir*`: 5/5 pass（偏移 0 检出星下点目标、偏移 90° 门控排除、nadir 0° ≡ 绝对 180° 等价、校验双域契约）。
   3、全量 sbirs 套件：unit 260/260、replay 38/38（含新字段往返与非法枚举原子拒绝）、contract 14/14、integration 29/29 全 pass。
   4、特征化：`sbirs_triple_sat_fix_messages.exe` 综合分 0.447267、dual_sat_cycles=80/80，与改动前逐位一致（eci_absolute 默认零行为变化）。
3、权威回写去向：扫描方位基准语义（双域定义、现算公式、限位检查边界、免推算凝视配方）→ `docs/sbirs_sensor/algorithms.md` WFOV 搜索实现边界第 8 条。
4、残留风险：nadir 基准下 config 期无法静态校验方位弧段越限（运行期才知道有效起点），极端限位窗口 + nadir 的组合要到运行期钳制才暴露；俯仰基准未 nadir 化，非 GEO 高倾角轨道的星下点俯仰仍需手工配 el。
5、后续冻结项：1、el 误配真实案例出现时再立方案 1 告警冻结项（本轮 defer）。2、俯仰基准 nadir 化（非目标，需求出现再立项）。
