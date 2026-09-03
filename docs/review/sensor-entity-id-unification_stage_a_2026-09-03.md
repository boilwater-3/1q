---
Status: draft
Date: 2026-09-03
Review-Baseline: `evidence/sensor-entity-id-unification` @ `fba3ea13`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# sensor-entity-id-unification：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

触发来源：RIR 与 SBIRS 设置传感器身份的模式不一致（RIR 融合通道=库内编译期常量、平台号写死在 session_config；SBIRS=场景 JSON 显式分配 + 会话注入），用户要求全模块统一为 SBIRS 显式分配模式，且 ID 不进 session_config。
- **证据**：[evidence: include/1q/fusion/SensorAdapters.h]::kRirSourceId
- **证据**：[evidence: include/1q/remote_identification_radar/config/RirSessionConfig.h]::sensor_platform_id
- **证据**：[evidence: include/1q/sbirs_sensor/session/SbirsSession.h]::SetSatelliteEntityId

术语白话（全文沿用）：
1、设备 ID（统一物）：每个传感器实例一个唯一编号——它的"身份证"，进 1q.log 日志行、验收行、融合通道三处，供人区分"这条信息是谁发的"。不进物理计算。
2、RF 记账号：电磁仿真世界里的平台号/设备号，只用于物理耦合计算（剔除自身回波、co-site 隔离）。粒度与"传感器实例"对不齐：平台号比传感器粗（机载雷达与干扰机共用）、设备号比传感器细（一台雷达收发两个号）、红外卫星根本不在 RF 世界。
3、融合通道号：融合引擎区分量测来源的编号，同时是配权数组下标。

待裁定项（需求/风险是否成立；四问由 §1 矩阵列承载）：
1、AR/ESR/EOS 多实例日志无法区分来源，是否成立（设备 ID 需要进库内日志行）。
2、"session 构造后 Set*EntityId() 注入、不进 session_config"作为全模块统一注入面，是否成立。
3、缺省语义=未注入沿用融合常量（含 SBIRS 单站回退 host.id() 改为常量），是否安全。
4、RIR sensor_platform_id 迁出 session_config（deprecated 过渡一轮），是否成立。
5、集成主路径=甲方实体 ID 直用（0 保留、位宽上限、配权稀疏时集成侧映射兜底），是否成立。
6、RF 记账号豁免本轮迁移（equipment_id 留 hardware config、AR 平台号留每拍输入注入、RIR 号保持 uint64），是否成立。
7、融合 source_weights 按 source_id 数组索引的机制保留（不改容器），是否成立。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、AR/ESR/EOS 多实例日志无法区分来源 | 三模块周期摘要日志行不含任何设备标识，同型多实例时逐字相同 | src/airborne_radar/signal/pipeline/SignalPipeline.cpp；src/electronic_surveillance_radar/pipeline/InterceptPipeline.cpp；src/electro_optical_sensor/pipeline/EosPipeline.cpp | grep 三模块周期摘要日志格式串（2026-09-03 执行）：三处均为 `[XxxPipeline] cycle_index=…` 形态，行内无设备标识字段 | 格式串确认无设备标识 → 统一设备 ID 需求成立 | 任一模块日志行已携带稳定设备标识（该模块只需对齐不需新机制） | pass |
| 2、统一注入面=session 构造后注入 | SBIRS 既有链路（组件→session→controller→pipeline）稳定且纯标注不影响计算，可作全模块范本 | include/1q/sbirs_sensor/session/SbirsSession.h::SetSatelliteEntityId；src/sbirs_sensor/runtime/SbirsController.h；src/sbirs_sensor/pipeline/SbirsPipeline.h；examples/components/sbirs_sensor_component.cpp | grep 全链路（2026-09-03 执行）：session→controller 透传→pipeline 成员；管线成员注释明确"验收行标注用，不影响计算"；组件另持同值副本打融合通道 | 链路存在、纯标注、无计算耦合 | 链路依赖 SBIRS 特有结构（如双星配置）无法泛化到单传感器模块 | pass |
| 3、缺省语义=未注入沿用融合常量 | 现有场景全部不显式注入（SBIRS 多星除外）；SBIRS 单站回退改常量只影响标注数值，不影响融合与计算行为 | include/1q/fusion/SensorAdapters.h（常量 1-5）；examples/components/sbirs_sensor_component.cpp；examples/scenes/ 各场景 JSON | grep 回退点（2026-09-03 执行）：单站构造 ground_station_source_id==0 时 `SetSatelliteEntityId(static_cast<uint32_t>(host.id()))`；该值消费方=生命周期事件标注与验收行，无计算消费 | 回退值仅进标注面 | 存在按 host.id() 数值做关联的逻辑（改回退会破坏） | pass |
| 4、RIR sensor_platform_id 迁出 session_config（过渡一轮） | 库内全部消费点可改为"会话持有的有效值"，config 字段过渡期仅作种子默认；controller 既有 setter 可复用 | include/1q/remote_identification_radar/config/RirSessionConfig.h::sensor_platform_id；src/remote_identification_radar/session/RirSession.cpp；src/remote_identification_radar/runtime/RirController.h::SetSensorPlatformId；include/1q/remote_identification_radar/session/RirIssueCodes.h::kSensorPlatformIdInvalid；src/remote_identification_radar/runtime/RirAcceptanceRecords.h | grep 全部消费点（2026-09-03 执行）：controller RF 输入 platform_id、radar_id 快照、验收行日志前缀、创建校验非零（kSensorPlatformIdInvalid）；无其他消费点；RirController::SetSensorPlatformId 已存在 | 消费点全部经会话可达；setter 已存在 | 存在只能从 config 到达、无法延后到注入期的消费点 | pass |
| 5、集成主路径=甲方实体 ID 直用 | 注入面接受任意非零 ID；实体 ID 在甲方系统天然全场景唯一；仓内已有直用先例 | examples/scenes/sbirs_triple_sat_fix_messages/main.cpp；src/fusion/FusionEngine.cpp；include/1q/fusion/FusionConfig.h::source_weights | grep（2026-09-03 执行）：三号场景 satellites[].source_id 加载期查重（"must be unique"）；ground_station.id() 实体号直塞 impact_distribution_source_id；FusionEngine 权重按 source_id 索引、越界回退 1.0 | 机制上直用无阻（非零、类型可容、缺省权重正确） | 融合或验收逻辑对 ID 连续性/小值有硬假设 | pass |
| 6、RF 记账号豁免本轮迁移 | RF 号与传感器实例粒度不对齐且位宽为 uint64，迁移无收益且破坏 co-site 隔离路径配置 | include/1q/electromagnetics/RfScene.h；examples/components/ecm_sensor_component.h::kPlatformEntityId；examples/app/runner.cpp；include/1q/airborne_radar/session/ArPlatformInput.h::platform_entity_id | grep（2026-09-03 执行）：RfScene 平台号/设备号均 uint64；kPlatformEntityId=1 被 AR 与 ECM 两组件共用（平台号比传感器粗）；AR 平台号走 ArPlatformInput 每拍注入（非 config）；co_site_paths 以 equipment_id 对表达 | 粒度/位宽证据成立 → 豁免合理 | 存在"传感器实例级"的 RF 号可 1:1 复用（则直用方案更简） | pass（确认豁免） |
| 7、融合 source_weights 数组索引机制保留 | 缺项=1.0 使大号稀疏下行为正确；容器化属未证实收益的投机改造 | include/1q/fusion/FusionConfig.h::source_weights；src/fusion/FusionEngine.cpp | grep（2026-09-03 执行）：`source_weights[source_id]` 越界回退 1.0，行为正确 | 现有机制满足直用需求 | 存在因稀疏导致的实际故障（内存/配置爆炸） | reject（不做容器化；大号+差异化配权需求出现时由集成侧映射，已裁定） |

## §2 判定汇总与待裁定问题

判定汇总：
1、项 1-6 建议 pass（其中项 6 为"确认豁免"），项 7 建议 reject（拒绝容器化改造，保留数组索引）。
2、全部判定依据的探针已于 2026-09-03 实际执行，结果写入 §1。

需要用户拍板的问题：
1、**位宽调整（对既有裁定 2 的收窄提议，探针新证据）**：RfScene 的平台号/设备号均为 uint64，RIR 的 sensor_platform_id 是 RF 域身份（须与 RF 场景平台号同域匹配），若统一收窄为 uint32 会与 RF 场景类型不匹配。提议：uint32 只约束纯标注域注入面（AR/ESR/EOS/SBIRS 的 Set*EntityId）；RIR 的 SetSensorPlatformId 保持 uint64。请确认。
2、甲方实体 ID 的实际形态（起始编号、位宽）待补——不影响本轮冻结，影响集成条款写死还是留弹性；无输入时按"非零、≤uint32、大号配权走集成侧映射"三条通用核对写入契约。

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、爆炸半径与回滚：下游消费方影响、回退难度（无损/破坏性/回滚注意点）。
5、验收门：构建、聚焦测试、契约测试、特征化测试、探针转正（有回归价值的探针转正式测试）。
6、非目标。
-->

## 修订记录

- 修订 1（2026-09-03，用户裁定）：setter 不同名、机制统一——SBIRS 保留 SetSatelliteEntityId，RIR 复用 controller 既有 SetSensorPlatformId，AR/ESR/EOS 各取域语义名；统一的机制=构造后注入、不进 session_config、场景层查重。
- 修订 2（2026-09-03，用户裁定）：标注域注入面统一 uint32（与融合通道一致）。同日探针发现 RfScene 平台/设备号为 uint64，提议收窄其适用范围（RIR 保持 uint64），待用户确认——见 §2 问题 1。
- 修订 3（2026-09-03，用户裁定）：RIR sensor_platform_id 迁出 session_config 采用 deprecated 过渡一轮（字段标记废弃、保留可赋值并告警，下一轮删字段与校验码）。
- 修订 4（2026-09-03，用户裁定）：SBIRS 单站回退 host.id() 改为回退融合常量 kSbirsSourceId（消除 World 自增域混入融合域与 64→32 截断）。
- 修订 5（2026-09-03，用户裁定）：AR 的 platform_entity_id 每拍经 ArPlatformInput 注入保留不动，不迁移到 session setter。
- 修订 6（2026-09-03，用户裁定）：设备 ID 消费面=1q.log 行前缀、验收行标注、融合通道号三处；AR/ESR/EOS 库内消费点=管线周期摘要日志行（三模块无库内验收 writer，验收行落在 examples 组件层）。
- 修订 7（2026-09-03，用户裁定）：集成主路径=甲方实体 ID 直用（0 保留为未注入哨兵须避开、≤uint32、大号且需差异化配权时集成侧映射兜底，库不做映射）。

## §4 运行记录（Stage C 后填写）

<!-- 对照强制回写清单勾项，全部完成才允许拆脚手架（见 SKILL.md 收尾规则）：
1、实现范围：改动文件与接口。
2、验证命令与结果：命令: pass/fail（含转正的探针测试）。
3、权威回写去向：
   1、正向边界：docs/<module>/design.md（boundaries/data-flow/algorithms 按归属选）。
   2、否决记录：docs/<module>/design.md 的"架构裁定与否决记录"专节（体裁见 docs-governance-standard）。
   3、开放议题：docs/common/open_questions.md 登记（编号）。
   4、证据锁：新增/修订规则后附 - **证据**：[evidence: 路径]。
4、残留风险。
5、后续冻结项。
-->
